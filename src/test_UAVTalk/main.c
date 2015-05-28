/**
Copyright (c) 2011.  All rights reserved.
An Open Source Arduino based OSD and Camera Control project.

Program  : ArduCAM-OSD (Supports the variant: minimOSD)
Version  : V1.9, 14 February 2012
Author(s): Sandro Benigno
Coauthor(s):
Jani Hirvinen   (All the EEPROM routines)
Michael Oborne  (OSD Configutator)
Mike Smith      (BetterStream and Fast Serial libraries)
Special Contribuitor:
Andrew Tridgell by all the support on MAVLink
Doug Weibel by his great orientation since the start of this project
Contributors: James Goppert, Max Levine
and all other members of DIY Drones Dev team
Thanks to: Chris Anderson, Jordi Munoz

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <stdlib.h>
#include <signal.h>

#include "uquad_error_codes.h"
#include "serial_comm.h"
#include "uquad_aux_time.h"
#include "UAVTalk.h"


#define CH_COUNT		5
#define BUFF_SIZE		10


/// Global vars
// CC3D
int fd_CC3D;
actitud_t act_last;
double yaw_rate;

// LOG
int log_fd;

/** 
 * Valores iniciales de los canales a enviar al proceso sbusd
 *
 * ch_buff[0]  roll
 * ch_buff[1]  pitch
 * ch_buff[2]  yaw
 * ch_buff[3]  throttle
 * ch_buff[4]  flight mode
 * ch_buff[5]  activar/desactivar failsafe
 *
 * init todo en cero y flight mode en 2 
 */
uint16_t ch_buff[CH_COUNT]={1500,1500,1500,950,2000};

/** 
 * Buffer para enviar mensajes de kernel
 * El casteo es necesario para enviar los mensajes de kernel
 * (de a 1 byte en lugar de 2)
 */
uint8_t *buff_out=(uint8_t *)ch_buff; 

void quit(void)
{
   uav_talk_deinit(fd_CC3D);
   exit(0);
}    


void uquad_sig_handler(int signal_num)
{
   printf("Caught signal: %d\n", signal_num);
   quit();
}


int main(int argc, char *argv[])
{    

  int retval;
  char* log_name;

  if(argc<2)
  {
	err_log("USAGE: ./test_UAVTalk log_name");
	exit(1);
  }
  else
  {
	log_name = argv[1];
  }

  // Catch signals
  signal(SIGINT,  uquad_sig_handler);
  signal(SIGQUIT, uquad_sig_handler);
  
  // Control de tiempos                                                         
  struct timeval tv_in_loop,
                 tv_start_main,
                 tv_diff,
                 dt;
			   
  // -- -- -- -- -- -- -- -- -- 
  // Inicializacion
  // -- -- -- -- -- -- -- -- -- 
  
  /// Tiempo
   set_main_start_time();
   tv_start_main = get_main_start_time();

   /// Log
   log_fd = open(log_name, O_RDWR | O_CREAT | O_NONBLOCK );
   if(log_fd < 0)
   {
      err_log_stderr("Failed to open log file!");
      quit();
   }

   /// UAVTALK
   fd_CC3D = uav_talk_init();
   bool CC3D_readOK = false;
   bool log_writeOK = false;

   actitud_t act = {0,0,0,{0,0}}; //almacena variables de actitud leidas de la cc3d y timestamp
   act_last = act;
   
   char buff_act[512];
   char buf_pwm[512];
   int buff_len;

  // -- -- -- -- -- -- -- -- -- 
  // Loop
  // -- -- -- -- -- -- -- -- --
  for(;;)
  {
	  //Para tener tiempo de entrada en cada loop
          gettimeofday(&tv_in_loop,NULL);
	  
	  //Pido el objeto
	  uavtalk_request_object(fd_CC3D, ATTITUDESTATE_OBJID);
	  
	  //Vacio buffer Rx
	  serial_flush(fd_CC3D); 
	  
	  //Espero a recibir objeto
	  retval = 0;
	  while(retval <= 0) {  
         /// Leo datos de CC3D
         CC3D_readOK = check_read_locks(fd_CC3D);
         if (CC3D_readOK) {
         
            retval = uavtalk_read(fd_CC3D, &act);
            if (retval < 0)
            {
               err_log("uavtalk_read failed");
               continue;
            } else if (retval == 0) {
               err_log("objeto no era actitud");                             
               continue;
            }
         } else {
            err_log("UAVTalk: read NOT ok");
            continue;
            //quit(0);
         }
	  }
	  
	  // velocidad
      retval = uquad_timeval_substract(&dt, act.ts, act_last.ts);
      if(retval > 0) {
         if(dt.tv_usec > 60000) err_log("WARN: se perdieron muestras");
         yaw_rate = 1000000*(act.yaw - act_last.yaw) / (long)(dt.tv_usec);
         act_last = act;
      } else {
         err_log("WARN: Absurd timing!");
         yaw_rate = sqrt (-1); //NaN
         //serial_flush(fd_CC3D);
      }
	  
	  
	  // Log - T_s_act T_us_act roll pitch yaw yaw_dot C_roll C_pitch C_yaw T_s_main T_us_main
      //Timestamp main
      uquad_timeval_substract(&tv_diff, tv_in_loop, tv_start_main);
      buff_len = uavtalk_to_str(buff_act, act);
      
      buff_len += sprintf(buf_pwm, "%lf %u %u %u %u %lu %lu\n",
                       yaw_rate,
                       ch_buff[0],
                       ch_buff[1],
                       ch_buff[2],
                       ch_buff[3],
		               tv_diff.tv_sec,
                       tv_diff.tv_usec);

      strcat(buff_act, buf_pwm);
      log_writeOK = check_write_locks(log_fd);
      if (log_writeOK) {
         retval = write(log_fd, buff_act, buff_len);
         if(retval < 0)
            err_log_stderr("Failed to write to log file!");   
      }
  
      /// Control de tiempos del loop
      wait_loop_T_US(MAIN_LOOP_50_MS,tv_in_loop);
	  
  } //fin loop

}
