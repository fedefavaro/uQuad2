#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <uquad_config.h>
#include <futaba_sbus.h>
#include <signal.h>
#include <unistd.h>

#define CH_COUNT		5
#define BUFF_SIZE		10

#if PC_TEST
#define START_SBUS_ARG 		"sbusd.log" //test en un PC linux
#else
#define START_SBUS_ARG		"/dev/ttyO1" //en la beagle
#endif

#define KILL_SBUS		"killall sbusd"

#define sleep_ms(ms)    	usleep(1000*ms)

//Global vars
pid_t sbusd_chld = -1;

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

void uquad_sbusd_term_handler(int signal_num)
{
	pid_t p;
    int status;
    p = waitpid(-1, &status, WNOHANG);
    if(p == sbusd_chld)
    {
		err_log_num("WARN: sbusd died! sig num:", signal_num);
		//quit();
		exit(1); //TODO reemplazar por quit
    }
}

int main(int argc, char *argv[])
{  
   int retval;
   
   /* Forks main program and starts client */
   int chld_pid = fork();

   // child process
   if (chld_pid == 0)
   {
      //starts sbus daemon
      retval = execl("./sbusd", "sbusd", START_SBUS_ARG, (char *) 0);
      //only get here if execl failed 
      if(retval < 0)
      {
         err_log_stderr("Failed to run sbusd!");
         goto cleanup;
      }
      
   }
   else if (chld_pid == -1)
   {
      err_log_stderr("Failed to start child process!");
      goto cleanup;
   }
   
	// The parent process continues here...
	sbusd_chld = chld_pid;

#if PC_TEST
   printf("Starting main in PC test mode\n");
   printf("For configuration options view common/uquad_config.h\n");
#endif

   static uint8_t *buff_out;
   static uint16_t ch_buff[CH_COUNT];

   buff_out = (uint8_t *)ch_buff;

   sleep_ms(5);     /// esto?

   // init kernel messeges queue
   uquad_kmsgq_t *kmsgq = uquad_kmsgq_init(SERVER_KEY, DRIVER_KEY);
   if(kmsgq == NULL)
   {
      err_log("Failed to start message queue!");
      goto cleanup;
   }

   ch_buff[0] = 1500;	// roll
   ch_buff[1] = 1500;	// pitch
   ch_buff[2] = 1500;	// yaw
   ch_buff[3] = 1500;   // throttle
   ch_buff[4] = 1500;   // flight mode?
   
   // Catch signals
	signal(SIGINT, uquad_sig_handler);
	signal(SIGQUIT, uquad_sig_handler);
	signal(SIGCHLD, uquad_sbusd_term_handler);

   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   for(;;)
   {
      sleep_ms(105);
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
