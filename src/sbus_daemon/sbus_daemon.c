#include <futaba_sbus.h>
#include <serial_comm.h>
#include <custom_baud.h>
#include <uquad_aux_time.h>
#include <uquad_error_codes.h>
#include <uquad_config.h>
#include <uquad_kernel_msgq.h>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <errno.h>   /* Error number definitions */
#include <unistd.h>
#include <stdint.h>
#include <sys/resource.h> // for setpriority()
#include <signal.h> // for SIGINT, SIGQUIT
#include <stdlib.h> 
#include <fcntl.h>
#include <sys/prctl.h>

// kernel queues
#include <sys/ipc.h> // for IPC_NOWAIT
#include <sys/msg.h>

#define CH_COUNT		5
#define LOOP_T_US               14000UL
#define MAX_ERR_SBUSD           20

#define HOW_TO    		"./sbus_daemon <device>"

// Global vars
static message_buf_t rbuf; //Stores kernel message

#if PC_TEST
struct timeval tv_start; // Guarda el tiempo de comienzo del programa

/* Si se esta realizando la prueba en un PC esta funcion se encarga de 
 * loggear en un formato legible el string sbus. */
int convert_sbus_data(uint8_t* sbusData, char* buf_str)
{
   char* buf_ptr = buf_str;
   int i,ret;
   struct timeval tv_ts; //for timestamp

   // Timestamp
   gettimeofday(&tv_ts,NULL);
   ret = uquad_timeval_substract(&tv_ts, tv_ts, tv_start);
   if(ret > 0)
   {
      buf_ptr += sprintf(buf_ptr, "%lu:%lu   \t", (unsigned long)tv_ts.tv_sec, (unsigned long)tv_ts.tv_usec);
   }
   else
   {
      err_log("WARN: Absurd timing!");
      return -1;
   }
   
   // Preparo el mensaje sbus
   for(i=0;i<SBUS_DATA_LENGTH;i++)
   {
      buf_ptr += sprintf(buf_ptr, "%02X ", sbusData[i]);
   }
  
   return 0; //char_count?

}
#endif //PC_TEST

/* Variable que almacena el file descriptor del puerto serie
 * usado para enviar el mensaje sbus. */
#if !PC_TEST
int fd; 

/* Si se realiza la prueba en un PC el mensaje se escribe 
 * en un archivo en lugar de ser enviado al puerto serie. */
#else
FILE * fp = NULL; 
#endif

void quit()
{
    int ret;
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
    exit(1);
}

void uquad_sig_handler(int signal_num){
    
    err_log_num("[Client] Caught signal: ",signal_num);
    quit();
}


int main(int argc, char *argv[])
{  

	int ret = ERROR_OK;
	int err_count = 0;
	int rcv_err_count = 0;
	int loop_count = 0;
	bool msg_received = false;
	char *device;
	int16_t *ch_buff; //stores parsed kernel message to update servos
	
    // check input arguments
    if(argc<2)
	{
		err_log(HOW_TO);
		return -1;
	}
	else
	device = argv[1];

#if SBUS_LOG_TO_FILE
   printf("Logging to %s\n", device);
#endif
  
   //buffer for sbus message
   uint8_t* sbusData = futaba_sbus_ptrsbusData();			

#if PC_TEST
   char str[128];
#endif //PC_TEST

   struct timeval tv_in;
   struct timeval tv_end;
   struct timeval tv_diff;

#if PC_TEST
   gettimeofday(&tv_start,NULL);
#endif //PC_TEST

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
       err_log_stderr("custom_baud() failed!");
       return ret;
   }
#else
   fp = fopen(device, "w");
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
	prctl(PR_SET_PDEATHSIG, SIGHUP);
	signal(SIGHUP, uquad_sig_handler);
   signal(SIGINT, uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);

   int j;
   for(j=1;j<17;j++)
     futaba_sbus_servo(j, 0);
   futaba_sbus_updateServos();

   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   
	for(;;)
	{
		gettimeofday(&tv_in,NULL);
	
	    if((err_count > MAX_ERR_SBUSD) || (rcv_err_count > MAX_ERR_SBUSD))
		{
		    err_log("error count exceded");
	        quit();
		}

		if(msg_received)
		{
        	futaba_sbus_servo(1, ch_buff[0]);
			futaba_sbus_servo(2, ch_buff[1]);
			futaba_sbus_servo(3, ch_buff[2]);
			futaba_sbus_servo(4, ch_buff[3]);
			futaba_sbus_servo(5, ch_buff[4]); 
        	futaba_sbus_updateServos();
			msg_received = false;
		}

#if !PC_TEST
        ret = write(fd, sbusData, 25);
        if (ret < 0)
        {
           fputs("write() failed!\n", stderr);
           err_count++;
		}
		else
		{
		    /// This loop was fine
		    if(err_count > 0)
			err_count--;
		}
#else
		// En modo PC test se escribe el mensaje a un archivo de texto o a stdout
	    convert_sbus_data(sbusData, str);
#if SBUS_LOG_TO_FILE
		/* Escribe en un arhivo */
	    ret = fprintf(fp, "%s", str);
	    if(ret < 0)
	    {
	        err_log("Failed to write to log file!");
	        err_count++;
	    }
#else
		/* Escribe en stdout */
		printf("%s",str);
#endif // SBUS_LOG_TO_FILE
		/// This loop was fine
		if(err_count > 0)
		   err_count--;
#endif // !PC_TEST
		
        loop_count++;

		// si pasaron mas de ~100ms es hora de leer el mensaje
		if (loop_count > 6) //14ms * 6 = 98ms
		{
    		ret = uquad_read(&rbuf);
       		if(ret == ERROR_OK)
			{
				msg_received = true;
				if(rcv_err_count > 0)
					rcv_err_count--;
				// Parse message. 2 bytes per channel.
    			ch_buff = (int16_t *)rbuf.mtext;
				/// send ack
				ret = uquad_send_ack();
				if(ret != ERROR_OK)
				{
					err_log("Failed to send ack!");
    			}
			}
			else
			{
				err_log("Failed to read msg!");
				msg_received = false;
				rcv_err_count++;
			}
			loop_count = 0;
		}


		gettimeofday(&tv_end,NULL);
		ret = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
		/// Check if we have to wait a while
		if(ret > 0)
		{
		    if(tv_diff.tv_usec < LOOP_T_US)
		    	usleep(LOOP_T_US - (unsigned long)tv_diff.tv_usec);

		}
		else
		{
            err_log("WARN: Absurd timing!");
	    	err_count++;
		}
        
	} //for(;;)  

   return 0; //never reaches here

}



