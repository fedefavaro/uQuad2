/**
 * main: QuadCop autopilot software
 * Copyright (C) 2015 Federico Favaro <ffavaro gmail.com>, Joaquin Berrutti <jberruti gmail.com>, Lucas Falkenstein <lfalkenstein gmail.com>
 *
 * This file is part of QuadCop.
 *
 * QuadCop is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with QuadCop.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @file   main.c
 * @author Federico Favaro <ffavaro gmail.com>, Joaquin Berrutti <jberruti gmail.com>, Lucas Falkenstein <lfalkenstein gmail.com>
 * @date   Mon May 15 10:24:44 2015
 *
 * @brief  QuadCop autopilot software
 *
 * TODO See src/main/README for information regarding how to run, configure, etc.
 *
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


#define CH_COUNT		5
#define BUFF_SIZE		10

#define KILL_SBUS		"killall sbusd"
#define MAIN_LOOP_T_US		110000UL

//Global vars
pid_t child_pid = -1;

static io_t *io  	= NULL;
uquad_bool_t read_ok	= false;

uquad_kmsgq_t *kmsgq 	= NULL;

//TODO QUE HACER CUANDO ALGO FALLA Y NECESITAMOS APAGAR LOS MOTORES!
void quit()
{
   int retval;

   /// IO manager
   retval = io_deinit(io);
   if(retval != ERROR_OK)
   {
      err_log("Could not close IO correctly!");
   }
   /// Kernel Messeges Queue
   uquad_kmsgq_deinit(kmsgq);
   
   /// GPS
   retval = deinit_gps();
   if(retval != ERROR_OK)
   {
      err_log("Could not close gps correctly!");
   }
   
   /// Demonio S-BUS 
   retval = system(KILL_SBUS);
   //if (retval ... TODO
   
   exit(1);
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
      if(p == child_pid)
      {
         err_log_num("WARN: sbusd died! sig num:", signal_num);
         //quit();
         exit(1); //TODO reemplazar por quit
      } else return;
   }
   // Si se capturo SIGINT o SIGQUIT termino el programa
   quit();
}


int main(int argc, char *argv[])
{  
   int retval;
   unsigned char tmp_buff[2] = {0,0}; // Para leer entrada de usuario

   struct timeval tv_in, tv_end, tv_diff;

   // Catch signals
   signal(SIGINT,  uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);
   signal(SIGCHLD, uquad_sig_handler);

 
   // -- -- -- -- -- -- -- -- -- 
   // Inicializacion
   // -- -- -- -- -- -- -- -- -- 

   /// GPS
//--------------------------------------------------------------------------
   retval = init_gps();
   if(retval < 0)
   {
      quit_log_if(ERROR_FAIL,"Failed to init gps!");
   }
//--------------------------------------------------------------------------

   /// Demonio S-BUS 
   child_pid = futaba_sbus_start_daemon();
   if(child_pid == -1)
   {
      err_log_stderr("Failed to start child process!");
      exit(1);
   }
   
   /// IO manager
   io = io_init();
   if(io==NULL)
   {
      quit_log_if(ERROR_FAIL,"io init failed!");
   }
   retval = io_add_dev(io,STDIN_FILENO);
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
   ch_buff[0] = 0;      // roll
   ch_buff[1] = 0;      // pitch
   //ch_buff[2] = 1500; // yaw
   //ch_buff[3] = 1500; // throttle
   //ch_buff[4] = 1500; // flight mode?
   
    
   // -- -- -- -- -- -- -- -- -- 
   // Loop
   // -- -- -- -- -- -- -- -- -- 
   for(;;)
   {
      gettimeofday(&tv_in,NULL);
      
      // envia mensaje de kernel para ser leidos por el demonio sbus
      retval = uquad_kmsgq_send(kmsgq, buff_out, MSGSZ);
      if(retval != ERROR_OK)
      {
         quit_log_if(ERROR_FAIL,"Failed to send message!");
      }

      /// if GPS
//--------------------------------------------------------------------------
      retval = get_gps_data();
      //if (ret ... TODO



//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
      /// -- -- -- -- -- -- -- --
      /// Check stdin
      /// -- -- -- -- -- -- -- --
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
         default:
            err_log("Velocidad invalida. Ingrese 0,1,2,3 para 0%,10%,20%,30%");
            break;
         } //switch(tmp_buff[0])
      }
      //end_stdin: //vengo aca si algo sale mal con leer stdin
 //--------------------------------------------------------------------------
    
      /// Control de tiempo del loop
      gettimeofday(&tv_end,NULL);
      retval = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
      if(retval > 0)
      {
         if(tv_diff.tv_usec < MAIN_LOOP_T_US)
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
