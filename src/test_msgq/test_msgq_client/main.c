#include <errno.h> 
#include <math.h>
#include <string.h> 
#include <stdio.h> 
#include <signal.h> // for SIGINT, SIGQUIT
#include <stdlib.h> 
#include <stdint.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/resource.h> // for setpriority()
#include <stdint.h>
#include <fcntl.h>
#include <sys/time.h>

#include "uquad_error_codes.h"

// kernel queues
#include <sys/ipc.h> // for IPC_NOWAIT
#include <sys/msg.h>

#include <time.h>

#define USAGE_STR "Incorrect arguments! Use no arguments (all enabled),"\
    "or 4, enabling each motor.\n\n\t./cmd ena9 enaA enaB ena8\nwhere"\
    "enaX enables motor X, valid is 0 or 1\n\n"

#define MAX_SPEED                 220 // i2c - must match mot_control.h
#define MIN_SPEED                 59  // i2c

/**
 * Die after MAX_ERR_CMD.
 * Almost no type of errors are tolerable:
 *   - speed update errors: Invalid arguments, etc. These are rare.
 *   - i2c problems - caused by failing to communicate with ESC controller,
 *   which is unnaccetable.
 */
#define MAX_ERR_CMD               20

#define DEBUG                     1

#if DEBUG
//#define LOG_VELS
#define DEBUG_TIMING
//#define LOG_TIMING_KERNEL_CALLS
#endif

#define TV_TH_US                  500UL

#define PRINT_COUNT               500
#define MOT_COUNT                 4
#define UQUAD_MOT_I2C_REG_ENABLE  0xA0
#define UQUAD_MOT_ENABLE_DAT      0x00
#define UQUAD_MOT_I2C_REG_DISABLE 0xA1
#define UQUAD_MOT_DISABLE_DAT     0x00
#define UQUAD_MOT_I2C_REG_VEL     0xA2
#define MOT_SELECTED              1
#define MOT_NOT_SELECTED          0

/// Startup parameters
#define RAMP_START                30
#define RAMP_END                  50
#define START_JITTER              25
#define STARTUP_SWAP              0

#define UQUAD_STARTUP_RETRIES     100
#define UQUAD_STOP_RETRIES        1000
#define UQUAD_USE_DIFF            0
#define UQUAD_USE_SIN             (1 && UQUAD_USE_DIFF)

#define LOOP_T_US                 2000UL
#define LOOP_T_US_INT             ((int) LOOP_T_US)
#define LOOP_TOLERANCE_US         300UL
#define MAX_LOOP_US               (LOOP_T_US + LOOP_TOLERANCE_US)

#define LOG_ERR                   stdout

#define backtrace()     err_log("backtrace:")

#define sleep_ms(ms)    usleep(1000*ms)

#define tv2double(db,tv)					\
    {								\
	db = ((double) tv.tv_sec) + ((double) tv.tv_usec)/1e6;	\
    }								\


void uquad_sig_handler(int signal_num){
    
    //int ret = ERROR_OK;
    err_log_num("[Client] Caught signal: ",signal_num);
    fflush(stderr);

}

int uquad_timeval_substract (struct timeval * result, struct timeval x, struct timeval y){
    /* Perform the carry for the later subtraction by updating y. */
    if (x.tv_usec < y.tv_usec) {
	int nsec = (y.tv_usec - x.tv_usec) / 1000000 + 1;
	y.tv_usec -= 1000000 * nsec;
	y.tv_sec += nsec;
    }
    if (x.tv_usec - y.tv_usec > 1000000) {
	int nsec = (y.tv_usec - x.tv_usec) / 1000000;
	y.tv_usec += 1000000 * nsec;
	y.tv_sec -= nsec;
    }
    
    /* Compute the time remaining to wait.
       tv_usec is certainly positive. */
    result->tv_sec = x.tv_sec - y.tv_sec;
    result->tv_usec = x.tv_usec - y.tv_usec;
    
    if(x.tv_sec < y.tv_sec)
	// -1 if diff is negative
	return -1;
    if(x.tv_sec > y.tv_sec)
	// 1 if diff is positive
	return 1;
    // second match, check usec
    if(x.tv_usec < y.tv_usec)
	// -1 if diff is negative
	return -1;
    if(x.tv_usec > y.tv_usec)
	// 1 if diff is positive
	return 1;

    // 0 if equal
    return 0;
}


/// Intercom via kernel msgq
typedef struct msgbuf {
    long    mtype;
    uint8_t mtext[MOT_COUNT];
} message_buf_t;

// Global vars
static int msqid; // Set by mot_control.h
static message_buf_t rbuf;

const static key_t key_s = 169; // must match MOT_SERVER_KEY (in mot_control.h)
const static key_t key_c = 170; // must match MOT_DRIVER_KEY (in mot_control.h)


static char ack_counter = 0;
int uquad_send_ack()
{
    int msqid;
    message_buf_t ack_msg;
    ack_msg.mtype = 1;
    if ((msqid = msgget(key_c, IPC_CREAT | 0666 )) < 0)
    {
	err_log_stderr("msgget()");
	fflush(stderr);
	return ERROR_FAIL;
    }
    ack_msg.mtext[0] = 'A';
    ack_msg.mtext[1] = 'C';
    ack_msg.mtext[2] = 'K';
    ack_msg.mtext[3] = ack_counter++;
    /// send msg
    if (msgsnd(msqid, &ack_msg, MOT_COUNT, IPC_NOWAIT) < 0)
    {
	err_log_stderr("msgsnd()");
	fflush(stderr);
	return ERROR_FAIL;
    }
    return ERROR_OK;
}

int uquad_read(void)
{
    int retval = ERROR_OK, i;
 
    uint8_t u8tmp;
    /// get speed data from kernel msgq
    if ((msqid = msgget(key_s, 0666)) < 0)
	return ERROR_FAIL;
    
    /*
     * Receive an answer of message type 1.
     */
    if (msgrcv(msqid, &rbuf, 4, 1, IPC_NOWAIT) < 0)
	return ERROR_FAIL;

    /*
     * Print the answer.
     */
    if(retval == ERROR_OK)
    {
	for(i=0; i < MOT_COUNT; ++i)
	{
	    u8tmp = 0xff & rbuf.mtext[i];
	    printf("%c", u8tmp);
	}
	printf("\n");
    }

    return retval;
}

int main(int argc, char *argv[])
{

    struct timeval
	tv_in,
	tv_diff,
	tv_end;

    /**
     * Inherit priority from main.c for correct IPC.
     */
    if(setpriority(PRIO_PROCESS, 0, -18) == -1)   //requires being superuser
    {
	//err_log("setpriority() failed!");
	err_log_num("setpriority() failed!",errno);
        return -1;
    }

   // Catch signals
   signal(SIGINT, uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);

    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    // Loop
    // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    int ret = ERROR_OK;
    int do_sleep = 0;
    

    for(;;)
    {
	gettimeofday(&tv_in,NULL);

	if(do_sleep)
	{
	    // avoid saturating i2c driver
	    //err_log("Will sleep to avoid saturating.");
	    fflush(stderr);
	    sleep_ms(10);
	    do_sleep = 0;
	}

	//hago algo..	
	
/*	if (ret != ERROR_OK)
	{
	    do_sleep = 1;
	    continue;
	}*/
	
	ret = uquad_read();
        if(ret == ERROR_OK)
	{
	    
	    /// send ack
	    ret = uquad_send_ack();
	    if(ret != ERROR_OK)
	    {
		err_log("Failed to send ack!");
            }
	    // continue
	}
	gettimeofday(&tv_end,NULL);

	ret = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
	/// Check if we have to wait a while
	if(ret > 0)
	{
	    if(tv_diff.tv_usec < LOOP_T_US)
	    {
		usleep(LOOP_T_US - (unsigned long)tv_diff.tv_usec);
	    }

	}
	else
	{
	    err_log("WARN: Absurd timing!");
	}
    }
 
    return 0;
}
