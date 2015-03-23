#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <futaba_sbus.h>
#include <signal.h>

/// Communication with motor driver is done via kernel msgs
#define MOT_SERVER_KEY 169 // some number
#define MOT_DRIVER_KEY 170 // some other number

#define DEVICE			/dev/stdout
#define CH_COUNT		5
#define BUFF_SIZE		2*CH_COUNT
#define START_SBUS 		"./sbus_daemon DEVICE &"
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
   static uint8_t *buff_out;
   static uint16_t ch_buff[CH_COUNT];

   buff_out = (uint8_t *)ch_buff;

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

   ch_buff[0] = 1000;	// roll
   ch_buff[1] = 1000;	// pitch
   ch_buff[2] = 1000;	// yaw
   ch_buff[3] = 1000;   // throttle
   ch_buff[3] = 1000;   // flight mode?


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
