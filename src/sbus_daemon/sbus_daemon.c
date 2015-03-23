#include <futaba_sbus.h>
#include <serial_comm.h>
#include <custom_baud.h>
#include <uquad_aux_time.h>
#include <uquad_error_codes.h>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <errno.h>   /* Error number definitions */
#include <unistd.h>
#include <stdint.h>
#include <sys/resource.h> // for setpriority()
#include <signal.h> // for SIGINT, SIGQUIT
#include <stdlib.h> 

// kernel queues
#include <sys/ipc.h> // for IPC_NOWAIT
#include <sys/msg.h>

#define CH_COUNT		5
#define LOOP_T_US               14000UL
#define MAX_ERR_CMD             20
#define PC_TEST			1     //TODO: hacerlo generico

#define HOW_TO     "./sbus_daemon <device>"

void uquad_sig_handler(int signal_num){
    
    err_log_num("[Client] Caught signal: ",signal_num);
    fflush(stderr);
    exit(1);
}

/// Intercom via kernel msgq
typedef struct msgbuf {
    long    mtype;
    uint8_t mtext[2*CH_COUNT];
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
    if (msgsnd(msqid, &ack_msg, 4, IPC_NOWAIT) < 0)
    {
	err_log_stderr("msgsnd()");
	fflush(stderr);
	return ERROR_FAIL;
    }
    return ERROR_OK;
}

int uquad_read(int16_t *buff_tmp_16)
{
    int retval = ERROR_OK;
    
 
    /// get speed data from kernel msgq
    if ((msqid = msgget(key_s, 0666)) < 0)
	return ERROR_FAIL;
    
    /*
     * Receive an answer of message type 1.
     */
    if (msgrcv(msqid, &rbuf, 2*CH_COUNT /*2 bytes per channel*/, 1, IPC_NOWAIT) < 0)
	return ERROR_FAIL;

    /*
     * Parse message. 2 bytes per channel.
     */
    if(retval == ERROR_OK)
    {
        buff_tmp_16 = (int16_t *)rbuf.mtext;
    }

    return retval;
}


int main(int argc, char *argv[])
{  
   int ret = ERROR_OK;
   int fd;
   int err_count = 0;
   char *device;
   int16_t ch_buff[5] = {0,0,0,0,0};  //stores parsed kernel message to update servos
   if(argc<2)
   {
      err_log(HOW_TO);
      return -1;
   }
   else
      device = argv[1];

#if !PC_TEST   
   //buffer for sbus message
   uint8_t* sbusData;			
   sbusData = futaba_sbus_ptrsbusData();
#else
   char str[128];
#endif // !PC_TEST
   
   struct timeval tv_in;
   struct timeval tv_end;
   struct timeval tv_diff;

   fd = open_port(device);
   if (fd == -1)
   { 
       return -1;
   }
   configure_port(fd);

#if !PC_TEST   
   ret = custom_baud(fd);      
   if (ret < 0)
   {
       fputs("custom_baud() failed!\n", stderr);
       return ret;
   }
#endif // !PC_TEST

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
   int do_sleep = 0;

   for(;;)
   {
	gettimeofday(&tv_in,NULL);

        if(err_count > MAX_ERR_CMD)
	{
	    printf("error count exceded");
            return -1;
	}
	if(do_sleep)
	{
	    // avoid saturating i2c driver
	    //printf("Will sleep to avoid saturating.");
	    sleep_ms(1);   //?
	    do_sleep = 0;
	}

#if !PC_TEST
        futaba_sbus_servo(1, ch_buff[0]);
	futaba_sbus_servo(2, ch_buff[1]);
	futaba_sbus_servo(3, ch_buff[2]);
	futaba_sbus_servo(4, ch_buff[3]);
	futaba_sbus_servo(5, ch_buff[4]);

        futaba_sbus_updateServos();
	ret = write(fd, sbusData, 25);
#else
        sprintf(str,"ch1: %d ch2: %d ch3: %d ch4: %d ch5: %d\n", \
		ch_buff[0],ch_buff[1],ch_buff[2],ch_buff[3],ch_buff[4]);
        ret = write(fd, str, strlen(str));
#endif // !PC_TEST
        if (ret < 0)
        {
           fputs("write() failed!\n", stderr);
           do_sleep = 1;
	   err_count++;
	   continue;
	}
	else
	{
	    /// This loop was fine
	    if(err_count > 0)
		err_count--;
	}
	
        ret = uquad_read(ch_buff);
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
	    err_count++;
	}
   } //for(;;)  

   return 0; //never reaches here

}



