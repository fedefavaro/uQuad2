#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <uquad_config.h>
#include <futaba_sbus.h>
#include <signal.h>

#define CH_COUNT		5
#define BUFF_SIZE		10

#if PC_TEST
#define START_SBUS 		"./sbusd sbusd.log &" //test en un PC linux
#else
#define START_SBUS 		"./sbusd /dev/ttyO1 &" //en la beagle
#endif

#define KILL_SBUS		"killall sbusd"

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

#if PC_TEST
   printf("Starting main in PC test mode\n");
   printf("For configuration options view common/uquad_config.h\n");
#endif

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

   sleep_ms(5);     /// esto?

   // init kernel messeges queue
   uquad_kmsgq_t *kmsgq = uquad_kmsgq_init(SERVER_KEY, DRIVER_KEY);
   if(kmsgq == NULL)
   {
      err_log("Failed to start message queue!");
      goto cleanup;
   }

   ch_buff[0] = 1000;	// roll
   ch_buff[1] = 1100;	// pitch
   ch_buff[2] = 1200;	// yaw
   ch_buff[3] = 1300;   // throttle
   ch_buff[4] = 1400;   // flight mode?
   
   // Catch signals
   signal(SIGINT, uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);


   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   for(;;)
   {
      sleep_ms(15);
      retval = uquad_kmsgq_send(kmsgq, buff_out, MSGSZ);
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
