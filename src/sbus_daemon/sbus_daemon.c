#include <futaba_sbus.h>
#include <serial_comm.h>
#include <custom_baud.h>
#include <uquad_aux_time.h>
#include <uquad_error_codes.h>
#include <quadcop_config.h>
#include <uquad_kernel_msgq.h>

#include <stdio.h>   /* Standard input/output definitions */
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
#define MAX_ERR_SBUSD           50

#define HOW_TO    		"./sbus_daemon <device>"

// Global vars
static message_buf_t rbuf; //Buffer para almacenar mensajes de kernel

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
   //int loop_count = 0;
   bool msg_received = false;
   char* device;
   /* Para parsear el los mensajes se recorre el arreglo con un puntero
    *  a enteros de dos bytes.
    */
   int16_t *ch_buff;
	
   // check input arguments
   if(argc<2)
   {
      err_log(HOW_TO);
      return -1;
   }
   else device = argv[1];

#if SBUS_LOG_TO_FILE
   printf("Logging to %s\n", device);
#endif

#if PC_TEST
   char str[128];
#endif //PC_TEST

   struct timeval tv_in;
   struct timeval tv_end;
   struct timeval tv_diff;
#if DEBUG_TIMING_SBUSD
   struct timeval tv_last;
#endif //#if DEBUG_TIMING_SBUSD

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

#if PC_TEST
   futaba_sbus_begin(); //para tiempo de start
#endif //PC_TEST

   // Lleva a cero todos los canales y el mensaje sbus
   futaba_sbus_set_channel(1, 1500); //init roll en cero
   futaba_sbus_set_channel(2, 1500); //init pitch en cero
   futaba_sbus_set_channel(3, 1500); //init yaw en cero
   //futaba_sbus_set_channel(4, 1000); //init throttle en cero
   //futaba_sbus_set_channel(5, 1000); //inint flight mode
   futaba_sbus_update_msg();
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   int cont_loop_14 = 0;
   for(;;)
   {
      gettimeofday(&tv_in,NULL);

      ret = uquad_read(&rbuf);
      if(ret == ERROR_OK)
      { 
         msg_received = true;
         // Parse message. 2 bytes per channel.
         ch_buff = (int16_t *)rbuf.mtext;
         // send ack
         ret = uquad_send_ack();
         if(ret != ERROR_OK)
         {
            err_log("Failed to send ack!");
         }
      }
      else
      {
         msg_received = false;
      }

      if(msg_received)
      {
         futaba_sbus_set_channel(1, ch_buff[0]);
         futaba_sbus_set_channel(2, ch_buff[1]);
         futaba_sbus_set_channel(3, ch_buff[2]);
         //futaba_sbus_set_channel(4, ch_buff[3]);
         //futaba_sbus_set_channel(5, ch_buff[4]);
 //------------------------------------------------------------ 
         if ( (ch_buff[4] == 50) && 
            (futaba_sbus_get_failsafe() == SBUS_SIGNAL_OK) )
            futaba_sbus_set_failsafe(SBUS_SIGNAL_FAILSAFE);
 //------------------------------------------------------------ 
         if ( (ch_buff[4] == 100) && 
            (futaba_sbus_get_failsafe() == SBUS_SIGNAL_FAILSAFE) )
            futaba_sbus_set_failsafe(SBUS_SIGNAL_OK);
 //------------------------------------------------------------  
         futaba_sbus_update_msg();
         msg_received = false;
      }

#if !PC_TEST
      futaba_sbus_write_msg(fd);
#else
      // En modo PC test se escribe el mensaje a un archivo de texto o a stdout
      convert_sbus_data(str);
#if SBUS_LOG_TO_FILE
      // Escribe en un arhivo/
      ret = fprintf(fp, "%s", str);
      if(ret < 0)
      {
         err_log("Failed to write to log file!");
      }
#else
      // Escribe en stdout
      printf("%s",str);
#endif // SBUS_LOG_TO_FILE

#endif // !PC_TEST
		
      /// Control de tiempo
      gettimeofday(&tv_end,NULL);
      ret = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
      if(ret > 0)
      {
         if(tv_diff.tv_usec < LOOP_T_US)
            usleep(LOOP_T_US - (unsigned long)tv_diff.tv_usec);
#if DEBUG_TIMING_SBUSD
         gettimeofday(&tv_end,NULL);
         ret = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
         printf("duracion loop sbusd (14ms): %lu\n",(unsigned long)tv_diff.tv_usec);
#endif
      } else {
         err_log("WARN: Absurd timing!");
      }
      
   } //for(;;)  

   return 0; //never gets here

}



