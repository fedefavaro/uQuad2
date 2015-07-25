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

#define CH_COUNT		6
#define LOOP_T_US		14000UL
#define MAX_ERR_SBUSD		20

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
   bool msg_received = false;
   char* device;
   /* Para parsear el los mensajes se recorre el arreglo con un puntero
    *  a enteros de dos bytes.
    */
   int16_t *ch_buff;
   
   //char str[128]; // dbg

   // check input arguments
   if(argc<2)
   {
      err_log(HOW_TO);
      return -1;
   }
   else device = argv[1];

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
#endif

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
   futaba_sbus_set_channel(ROLL_CHANNEL, 1500); //init roll en cero
   futaba_sbus_set_channel(PITCH_CHANNEL, 1500); //init pitch en cero
   futaba_sbus_set_channel(YAW_CHANNEL, 1500); //init yaw en cero
   futaba_sbus_set_channel(THROTTLE_CHANNEL, 950); //init throttle en minimo
   futaba_sbus_set_channel(FLIGHTMODE_CHANNEL, 1500); //inint flight mode 2
   futaba_sbus_update_msg();

   sleep_ms(500); //Para ponerme a tiro con main
   
   bool main_ready = false;

   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   for(;;)
   {
	gettimeofday(&tv_in,NULL);

	//printf("errores: %d\n",err_count);//dbg

#if !PC_TEST
	if (err_count > MAX_ERR_SBUSD)
	{
	   err_log("error count exceded");
	   //err_count = 0;
	   quit();
	}
#endif

	ret = uquad_read(&rbuf);
	if(ret == ERROR_OK)
	{
	   if(!main_ready) main_ready = true;

           //err_log("read ok!");
 	   msg_received = true;
	   // Parse message. 2 bytes per channel.
	   ch_buff = (int16_t *)rbuf.mtext;
	   // send ack
	   ret = uquad_send_ack();
	   if(ret != ERROR_OK)
	   {
		err_log("Failed to send ack!");
	   }
	   if (rcv_err_count > 0)
		rcv_err_count = 0;
           
	} else {
	   //err_log("Failed to read msg!");
	   msg_received = false;
	   rcv_err_count++;
	   if (main_ready && rcv_err_count > 3) {
		err_count++;
		rcv_err_count = 0;
	   }
	}

	if(msg_received)
	{
	   futaba_sbus_set_channel(ROLL_CHANNEL, ch_buff[ROLL_CH_INDEX]);
	   futaba_sbus_set_channel(PITCH_CHANNEL, ch_buff[PITCH_CH_INDEX]);
	
	   futaba_sbus_set_channel(YAW_CHANNEL, ch_buff[YAW_CH_INDEX]);
	   futaba_sbus_set_channel(THROTTLE_CHANNEL, ch_buff[THROTTLE_CH_INDEX]);
	   //futaba_sbus_set_channel(7, ch_buff[FLIGHTMODE_CH_INDEX]); // flight mode no se modifica

	   // Comando para activar failsafe
	   if ( (ch_buff[FAILSAFE_CH_INDEX] == ACTIVATE_FAILSAFE) && 
	   (futaba_sbus_get_failsafe() == SBUS_SIGNAL_OK) )
		futaba_sbus_set_failsafe(SBUS_SIGNAL_FAILSAFE);

	   // Comando para desactivar failsafe
 	   if ( (ch_buff[FAILSAFE_CH_INDEX] == DEACTIVATE_FAILSAFE) && 
	   (futaba_sbus_get_failsafe() == SBUS_SIGNAL_FAILSAFE) )
		futaba_sbus_set_failsafe(SBUS_SIGNAL_OK);
 
	   futaba_sbus_update_msg();
	   msg_received = false;
	   //print_sbus_data();  // dbg
	}

#if !PC_TEST
        ret = futaba_sbus_write_msg(fd);
        if (ret < 0)
        {
	   err_count++;  // si fallo al enviar el mensaje se apagan los motores!!
	}
	else
	{
	   /// This loop was fine
	   if(err_count > 0)
	      err_count--;
	}
	sleep_ms(5);  //TODO revisar si hay que hacerlo siempre!!
#else
	sleep_ms(5);  //TODO revisar si hay que hacerlo siempre!!
#endif
	// Escribe el mensaje a stdout - dbg
	//convert_sbus_data(str);
	//printf("%s",str);

	/// Control de tiempo
	gettimeofday(&tv_end,NULL);
	ret = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
	if(ret > 0)
	{
	   if(tv_diff.tv_usec < LOOP_T_US)
		usleep(LOOP_T_US - (unsigned long)tv_diff.tv_usec);
#if DEBUG_TIMING_SBUSD
           gettimeofday(&tv_end,NULL);
           uquad_timeval_substract(&tv_diff, tv_end, tv_in);
           printf("duracion loop sbusd (14ms): %lu\n",(unsigned long)tv_diff.tv_usec);
#endif
	} else {
	   err_log("WARN: Absurd timing!");
	   err_count++; // si no cumplo el tiempo de loop falla la comunicacion sbus
	}

      //printf("rcv_err_count: %d\n", rcv_err_count);
      //printf("err_count: %d\n", err_count);

   } //for(;;)  

   return 0; //never gets here

}


