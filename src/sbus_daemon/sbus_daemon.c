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
#include <fcntl.h>


// kernel queues
#include <sys/ipc.h> // for IPC_NOWAIT
#include <sys/msg.h>

#define CH_COUNT		5
#define BUFF_SIZE		10
#define LOOP_T_US               14000UL
#define MAX_ERR_CMD             20

#define PC_TEST			0     //TODO: hacerlo generico
#define FILE_PATH		"/home/labcontrol2/log_sbus_daemon.txt"

#define HOW_TO    		"./sbus_daemon <device>"


#if PC_TEST
int convert_sbus_data(uint8_t* sbusData, char* buf_str)
{
   char* buf_ptr = buf_str;
   int i;
   
   for(i=0;i<SBUS_DATA_LENGTH;i++)
   {
      buf_ptr += sprintf(buf_ptr, "%02X ", sbusData[i]);
   }
   sprintf(buf_ptr,"\n");
   
   return 0; //char_count?

}
#endif //PC_TEST

#if !PC_TEST
int fd;
#else
FILE * fp = NULL;
#endif

void uquad_sig_handler(int signal_num){
    
    int ret;
    err_log_num("[Client] Caught signal: ",signal_num);
#if !PC_TEST
    ret = close(fd);
    if(ret < 0)
    {
	err_log_stderr("Failed to close serial port!");
    }
#else
    ret = fclose(fp);
    if(ret != 0)
    {
	err_log_stderr("Failed to close log file!");
    }
#endif
    fflush(stderr);
}

/// Intercom via kernel msgq
typedef struct msgbuf {
    long    mtype;
    uint8_t mtext[BUFF_SIZE];
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

int uquad_read(void)
{
    // get speed data from kernel msgq
    if ((msqid = msgget(key_s, 0666)) < 0)
	return ERROR_FAIL;
    
    /*
     * Receive an answer of message type 1.
     */
    if (msgrcv(msqid, &rbuf, BUFF_SIZE, 1, IPC_NOWAIT) < 0)
	return ERROR_FAIL;

    return ERROR_OK;
}


int main(int argc, char *argv[])
{  
   int ret = ERROR_OK;
   int err_count = 0;
   char *device;
   int16_t ch_buff_aux[CH_COUNT] = {0,0,0,0,0};  //stores parsed kernel message to update servos
   int16_t *ch_buff = ch_buff_aux;

   if(argc<2)
   {
      err_log(HOW_TO);
      return -1;
   }
   else
      device = argv[1];
  
   //buffer for sbus message
   uint8_t* sbusData;			
   sbusData = futaba_sbus_ptrsbusData();
#if PC_TEST
   char str[64];
#endif //PC_TEST

   struct timeval tv_in;
   struct timeval tv_end;
   struct timeval tv_diff;

#if !PC_TEST 
   fd = open_port(device);
   if (fd == -1)
   { 
       return -1;
   }
   configure_port(fd);
   ret = custom_baud(fd);      
   if (ret < 0)
   {
       fputs("custom_baud() failed!\n", stderr);
       return ret;
   }
#else
   fp = fopen(FILE_PATH, "w");
   if(fp == NULL)
   {
	err_log_stderr("Failed to open log file!");
	return -1;
   }

#endif // !PC_TEST

   /**
    * Inherit priority from main.c for correct IPC.
    */
   if(setpriority(PRIO_PROCESS, 0, -18) == -1)   //requires being superuser
   {
      err_log_num("setpriority() failed!",errno);
      return -1;
    }

   // Catch signals
   signal(SIGINT, uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);

   int j;
   for(j=1;j<17;j++)
     futaba_sbus_servo(j, 0);

   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   //int do_sleep = 0;

   for(;;)
   {
	gettimeofday(&tv_in,NULL);

        if(err_count > MAX_ERR_CMD)
	{
	    printf("error count exceded");
            return -1;
	}
/*	if(do_sleep)
	{
	    // avoid saturating i2c driver
	    //printf("Will sleep to avoid saturating.");
	    sleep_ms(1);   //?
	    do_sleep = 0;
	}*/

        futaba_sbus_servo(1, ch_buff[0]);
	futaba_sbus_servo(2, ch_buff[1]);
	futaba_sbus_servo(3, ch_buff[2]);
	futaba_sbus_servo(4, ch_buff[3]);
	futaba_sbus_servo(5, ch_buff[4]); 
        futaba_sbus_updateServos();

#if !PC_TEST
        ret = write(fd, sbusData, 25);
        if (ret < 0)
        {
           fputs("write() failed!\n", stderr);
           //do_sleep = 1;
	   err_count++;
	   continue;
	}
	else
	{
	    /// This loop was fine
	    if(err_count > 0)
		err_count--;
	}
#else
        convert_sbus_data(sbusData, str);
        ret = fprintf(fp, str);
        if(ret < 0)
        {
           err_log_stderr("Failed to write to log file!");
        }

#endif // !PC_TEST
	
        ret = uquad_read();
        if(ret == ERROR_OK)
	{
	  
     	   // Parse message. 2 bytes per channel.
    	   ch_buff = (int16_t *)rbuf.mtext;
            
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



