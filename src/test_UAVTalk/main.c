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

#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

// Configurations
#include "OSD_Config.h"
#include "OSD_Vars.h"

#include "serial_comm.h"

// OpenPilot UAVTalk:
#include "UAVTalk.h"

/// Defs
#define TELEMETRY_SPEED	57600  // How fast our MAVLink telemetry is coming to Serial port
#define DEVICE  	"/dev/ttyUSB0"

/// Macros
#define sleep_ms(ms)    usleep(1000*ms)

/// Global vars
int fd;


void quit(void)
{
   int ret = close(fd);
   if(ret < 0)
      printf("No se pudo cerrar puerto. Ya esta cerrado?\n");
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

  fd_set rfds;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  
  // Catch signals
  signal(SIGINT,  uquad_sig_handler);
  signal(SIGQUIT, uquad_sig_handler);
  
  // -- -- -- -- -- -- -- -- -- 
  // Inicializacion
  // -- -- -- -- -- -- -- -- -- 
  
  /// Puerto Serie Beagle-CC3D
  fd = open_port(DEVICE);
  if (fd < 0) quit();
  printf("CC3D conectada\n");
  
  retval = configure_port(fd, B57600);
  if (retval < 0) quit();

  // -- -- -- -- -- -- -- -- -- 
  // Loop
  // -- -- -- -- -- -- -- -- --
  for(;;)
  {
     
     FD_ZERO(&rfds);
     FD_SET(fd, &rfds);
     retval = select(fd+1, &rfds, NULL, NULL, &tv);
     if(retval < 0)
        printf("select() failed!\n");
     
     if(FD_ISSET(fd,&rfds))
     {
        if (uavtalk_read(fd)) {
           // imprimo lo que leo?
        } else {
           //sleep_ms(10);
        }
     }
  
  } //fin loop

}



