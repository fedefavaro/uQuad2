#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <uquad_config.h>
#include <uquad_aux_time.h>
#include <uquad_aux_io.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>


#define CH_COUNT		5
#define BUFF_SIZE		10

#if PC_TEST
#define START_SBUS_ARG 		"sbusd.log" //test en un PC linux
#else
#define START_SBUS_ARG		"/dev/ttyO1" //en la beagle
#endif

#define KILL_SBUS		"killall sbusd"
#define MAIN_LOOP_T_US               110000UL


//Global vars
pid_t sbusd_chld = -1;
static io_t *io            = NULL;

void quit()
{
    int retval;
    /// IO manager
    retval = io_deinit(io);
    if(retval != ERROR_OK)
    {
	err_log("Could not close IO correctly!");
    }
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
   unsigned char tmp_buff[2] = {0,0}; // Para leer entrada de usuario

   struct timeval tv_in;
   struct timeval tv_end;
   struct timeval tv_diff;
   
   uquad_bool_t read_ok       = false;

   // Catch signals
   signal(SIGINT, uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);
   signal(SIGCHLD, uquad_sbusd_term_handler);

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
         exit(1);
      }
      
   }
   else if (chld_pid == -1) {
      err_log_stderr("Failed to start child process!");
      goto cleanup;
   }

   // The parent process continues here...
   sbusd_chld = chld_pid;

    io = io_init();
    if(io==NULL)
    {
	quit_log_if(ERROR_FAIL,"io init failed!");
    }

   retval = io_add_dev(io,STDIN_FILENO);
   quit_log_if(retval, "Failed to add stdin to io list"); 

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
   
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   for(;;)
   {
      gettimeofday(&tv_in,NULL);
      
      retval = uquad_kmsgq_send(kmsgq, buff_out, MSGSZ);
      if(retval != ERROR_OK)
      {
         fputs("Failed to send message!\n",stderr);
         goto cleanup;
      }

      /// -- -- -- -- -- -- -- --
      /// Check stdin
      /// -- -- -- -- -- -- -- --
      retval = io_poll(io);
      quit_log_if(retval,"io_poll() error");
      retval = io_dev_ready(io,STDIN_FILENO,&read_ok,NULL);
      log_n_continue(retval, "Failed to check stdin for input!");
      if(!read_ok)
 	 goto end_stdin;

      retval = fread(tmp_buff,sizeof(unsigned char),1,stdin);
      if(retval <= 0)
      {
	 log_n_jump(ERROR_READ, end_stdin,"No user input detected!");
      }
      else if(tmp_buff[0] == 'a') //ch0 up
      {
         if(ch_buff[0]<=1990)
            ch_buff[0] = ch_buff[0]+10;
      }
      else if(tmp_buff[0] == 'd') //ch0 down
      {
         if(ch_buff[0]>=1010)
            ch_buff[0] = ch_buff[0]-10;
      }
      else if(tmp_buff[0] == 'r') //ch0 reset
      {
         ch_buff[0] = 1500;
      }

      end_stdin: //vengo aca si algo sale mal con leer stdin
     
      gettimeofday(&tv_end,NULL);
      retval = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
      
      /// Check if we have to wait a while
      if(retval > 0)
      {
         if(tv_diff.tv_usec < MAIN_LOOP_T_US)
            usleep(MAIN_LOOP_T_US - (unsigned long)tv_diff.tv_usec);
         
      }
      else
      {
         fputs("WARN: Absurd timing!\n",stderr);
      }

      //DEBUG
      /*gettimeofday(&tv_end,NULL);
      retval = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
      printf("loop main: %lu\n",(unsigned long)tv_diff.tv_usec);*/

   }

   cleanup:
   // deinit
   uquad_kmsgq_deinit(kmsgq);
   retval = system(KILL_SBUS);

   return 0;

}
