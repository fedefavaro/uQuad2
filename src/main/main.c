/**
 ******************************************************************************
 *
 * @file       main.c
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      QuadCop autopilot software
 * @see        src/main/README for information regarding how to run, configure, etc.
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/> or write to the 
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <uquad_config.h>
#include <uquad_aux_time.h>
#include <uquad_aux_io.h>
#include <futaba_sbus.h>
#include <gps_comm.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#define CH_COUNT		5
#define BUFF_SIZE		10

#define KILL_SBUS		"killall sbusd"
#define MAIN_LOOP_T_US		105000UL

//Global vars
pid_t sbusd_child_pid = -1;
pid_t gpsd_child_pid = -1;

      sigset_t mask;
      sigset_t orig_mask;

static io_t *io  	= NULL;
uquad_bool_t read_ok	= false;

uquad_kmsgq_t *kmsgq 	= NULL;

/**
 * Interrumpe ejecucion del programa. Dependiendo del valor del parametro
 * que se le pase interrumpe mas o menos cosas.
 * Q == 2 : interrupcion manual (ctrl-c), cierra todo y avisa que fue manual
 * Q == 1 : interrupcion por muerte del sbusd, cierra todo menos sbusd
 * Q == 0 : interrupcion estandar, cierra todo
 * Q cualquier otro : igual que Q == 0.
 *
 * TODO QUE HACER CUANDO ALGO FALLA Y NECESITAMOS APAGAR LOS MOTORES!
 */
void quit(int Q)
{
   int retval;
   char str[30];
   switch(Q)
   {
      case 2:
         err_log("interrumpido manualmente (ctrl-c)");
      default:
      case 0:
         /// Demonio S-BUS 
         retval = system(KILL_SBUS);
         //sprintf(str, "kill -SIGTERM %d", sbusd_child_pid);
         //retval = system(str);         
         if (retval < 0)
         {
            err_log("Could not terminate sbusd!");
         }
      case 1:
         /// IO manager
         retval = io_deinit(io);
         if(retval != ERROR_OK)
         {
            err_log("Could not close IO correctly!");
         }
         /// Kernel Messeges Queue
         uquad_kmsgq_deinit(kmsgq);

#if !DISABLE_GPS
         /// GPS
         retval = deinit_gps();
         if(retval != ERROR_OK)
         {
            err_log("Could not close gps correctly!");
         }
#endif //DISABLE_GPS

         retval = system("killall gpsd");
         //sprintf(str, "kill -SIGTERM %d", &gpsd_child_pid);
         //retval = system(str);
         if(retval != ERROR_OK)
         {
            err_log("Could not close gps correctly!");
         }

   } //switch(Q)
   
   exit(0);
}    


void uquad_sig_handler(int signal_num)
{

   err_log_num("Caught signal:",signal_num);
   // Si se murio el demonio sbus termino el programa
   if (signal_num == SIGCHLD)
   {
      pid_t p;
      int status;
      p = waitpid(-1, &status, WNOHANG);
      if(p == sbusd_child_pid)
      {
         err_log_num("WARN: sbusd died! sig num:", signal_num);
         quit(1); //exit sin cerrar sbusd
      } else if(p == gpsd_child_pid) {
         err_log_num("WARN: gpsd died! sig num:", signal_num);
         quit(0);
      } else {
         err_log_num("Return:", signal_num);
         return;
      }
   }
   
   // bloqueo SIGCHLD si entre por SIGINT
   if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0) {
      perror ("sigprocmask");
   }
   // Si se capturo SIGINT o SIGQUIT termino el programa
   quit(2);
}


int main(int argc, char *argv[])
{  
   sigemptyset (&mask);
   sigaddset (&mask, SIGCHLD);
   //sigaddset (&mask, SIGTERM);

   int retval;
   
   // Para leer entrada de usuario
   unsigned char tmp_buff[2] = {0,0}; 

   // Control de tiempos
   struct timeval tv_in, tv_end, tv_diff;

   // Catch signals
   signal(SIGINT,  uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);
   signal(SIGCHLD, uquad_sig_handler);

   // -- -- -- -- -- -- -- -- -- 
   // Inicializacion
   // -- -- -- -- -- -- -- -- -- 

   /// Demonio S-BUS 
   sbusd_child_pid = futaba_sbus_start_daemon();
   if(sbusd_child_pid == -1)
   {
      err_log_stderr("Failed to start child process (sbusd)!");
      exit(1);
   }

#if !DISABLE_GPS
   /// GPS
   gpsd_child_pid = init_gps();
   if(gpsd_child_pid == -1)
   {
      quit_log_if(ERROR_FAIL,"Failed to init gps!");
   }
#endif

   //Doy tiempo a que inicien bien los programitas...
   sleep_ms(500);   

   /// IO manager
   io = io_init();
   if(io==NULL)
   {
      quit_log_if(ERROR_FAIL,"io init failed!");
   }
   retval = io_add_dev(io,STDIN_FILENO);  // Se agrega stdin al io manager
   quit_log_if(retval, "Failed to add stdin to io list"); 

   /// Kernel Messeges Queue
   kmsgq = uquad_kmsgq_init(SERVER_KEY, DRIVER_KEY);
   if(kmsgq == NULL)
   {
      quit_log_if(ERROR_FAIL,"Failed to start message queue!");
   }

#if PC_TEST
   printf("Starting main in PC test mode\n");
   printf("For configuration options view common/uquad_config.h\n");
#endif

   static uint8_t *buff_out;            // buffer para enviar mensajes de kernel
   static uint16_t ch_buff[CH_COUNT];   // arreglo para almacenar el valor de los canales a enviar
   buff_out = (uint8_t *)ch_buff;

   /// Valores iniciales de los canales de sbus
   ch_buff[0] = 1500;      // roll
   ch_buff[1] = 1500;      // pitch
   //ch_buff[2] = 1500; // yaw
   //ch_buff[3] = 1500; // throttle
   //ch_buff[4] = 1500; // flight mode?
   
    
   // -- -- -- -- -- -- -- -- -- 
   // Loop
   // -- -- -- -- -- -- -- -- -- 
   for(;;)
   {
      gettimeofday(&tv_in,NULL);
      
      /// -- -- -- -- -- -- -- --
      /// Check stdin
      /// -- -- -- -- -- -- -- --
//--------------------------------------------------------------------------
      retval = io_poll(io);
      quit_log_if(retval,"io_poll() error");
      retval = io_dev_ready(io,STDIN_FILENO,&read_ok,NULL);
      log_n_continue(retval, "Failed to check stdin for input!");
      //if(!read_ok)
 	// goto end_stdin;
      if(read_ok)
      {
         retval = fread(tmp_buff,sizeof(unsigned char),1,stdin); //TODO corregir que queda algo por leer en el buffer?
         if(retval <= 0)
         {
	    //log_n_jump(ERROR_READ, end_stdin,"No user input detected!");
            err_log_num("No user input detected!",ERROR_READ);
         }
         switch(tmp_buff[0])
         {
         case '0':
            ch_buff[0] = 1500;
            break;
         case '1':
            ch_buff[0] = 1550;
            break;
         case '2':
            ch_buff[0] = 1600;
            break;
         case '3':
            ch_buff[0] = 1650;
            break;
         case 'F':
            err_log("Failsafe set");
            ch_buff[4] = 50;
            break;
         case 'f':
            err_log("Failsafe clear");
            ch_buff[4] = 100;
            break;
         default:
            err_log("Velocidad invalida. Ingrese 0,1,2,3 para 0%,10%,20%,30%");
            break;
         } //switch(tmp_buff[0])
      }
      //end_stdin: //vengo aca si algo sale mal con leer stdin
 //--------------------------------------------------------------------------
#if !DISABLE_GPS
      /// if GPS
      retval = get_gps_data();
      if (retval < 0 )
      {
         err_log("No hay datos de gps");
         //que hago si no hay datos!?
      }    
#endif
      // envia mensaje de kernel para ser leidos por el demonio sbus
      retval = uquad_kmsgq_send(kmsgq, buff_out, MSGSZ);
      if(retval != ERROR_OK)
      {
         quit_log_if(ERROR_FAIL,"Failed to send message!");
      }

      /// Control de tiempo del loop
      gettimeofday(&tv_end,NULL);
      retval = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
      if(retval > 0)
      {
         if(tv_diff.tv_usec < MAIN_LOOP_T_US)
            // Sobro tiempo, voy a dormir
            usleep(MAIN_LOOP_T_US - (unsigned long)tv_diff.tv_usec);
      }
      else
         fputs("WARN: Absurd timing!\n",stderr);
      
#if DEBUG_TIMING_MAIN
      gettimeofday(&tv_end,NULL);
      retval = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
      printf("duracion loop main: %lu\n",(unsigned long)tv_diff.tv_usec);
#endif

   } // for(;;)

   return 0; //never reaches here

}
