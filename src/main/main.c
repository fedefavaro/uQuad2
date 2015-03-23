#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <futaba_sbus.h>
#include <signal.h>

/// Communication with motor driver is done via kernel msgs
#define MOT_SERVER_KEY 169 // some number
#define MOT_DRIVER_KEY 170 // some other number

#define DEVICE			/dev/stdout
#define BUFF_SIZE               5
#define START_SBUS 		"../sbus_daemon/sbus_daemon DEVICE &"
#define KILL_SBUS		"killall sbus_daemon"

#define sleep_ms(ms)    	usleep(1000*ms)

void quit()
{
    int retval;
    retval = system(KILL_SBUS);
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
   retval = system(START_SBUS);
   if(retval < 0)
   {
      err_log("Failed to run cmd!");
      goto cleanup;
   }

   sleep_ms(500);     /// esto?

   // init kernel messeges queue
   uquad_kmsgq_t *kmsgq = uquad_kmsgq_init(MOT_SERVER_KEY, MOT_DRIVER_KEY);
   if(kmsgq == NULL)
   {
      err_log("Failed to start message queue!");
      goto cleanup;
   }

   buff_out[0] = 'H';	// roll
   buff_out[1] = 'O';	// pitch
   buff_out[2] = 'L';	// yaw
   buff_out[3] = 'A';   // throttle
   buff_out[3] = '!';   // flight mode?


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
   retval = system(KILL_SBUS);

   return 0;

}
