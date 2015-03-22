
#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <signal.h>

/// Communication with motor driver is done via kernel msgs
#define MOT_SERVER_KEY 169 // some number
#define MOT_DRIVER_KEY 170 // some other number

#define BUFF_SIZE               4
#define START_CLIENT 		"./test_msgq_client &"
#define KILL_CLIENT		"killall test_msgq_client"

#define sleep_ms(ms)    	usleep(1000*ms)

void quit()
{
    int retval;
    retval = system(KILL_CLIENT);
    exit(retval);
}    


void uquad_sig_handler(int signal_num)
{
    err_log_num("Caught signal:",signal_num);
    quit();
}

int main(int argc, char *argv[])
{  
   int retval;
   static uint8_t buff_out[BUFF_SIZE];

   // start client process
   retval = system(START_CLIENT);
   if(retval < 0)
   {
      err_log("Failed to run cmd!");
      goto cleanup;
   }

   sleep_ms(500); 

   // init kernel messeges queue
   uquad_kmsgq_t *kmsgq = uquad_kmsgq_init(MOT_SERVER_KEY, MOT_DRIVER_KEY);
   if(kmsgq == NULL)
   {
      err_log("Failed to start message queue!");
      goto cleanup;
   }

   buff_out[0] = 'H';
   buff_out[1] = 'O';
   buff_out[2] = 'L';
   buff_out[3] = 'A';


   // Catch signals
   signal(SIGINT, uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);


   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   for(;;)
   {
      sleep_ms(40);
      retval = uquad_kmsgq_send(kmsgq, buff_out, BUFF_SIZE);
      if(retval != ERROR_OK)
      {
         err_log("Failed to send message!");
         goto cleanup;
      }
   }

   cleanup:
   // deinit
   uquad_kmsgq_deinit(kmsgq);
   retval = system(KILL_CLIENT);

   return 0;

}
