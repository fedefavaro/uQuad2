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

// Get the common arduino functions
#include "Arduino.h"

// Configurations
#include "OSD_Config.h"
#include "OSD_Vars.h"
//#include "OSD_Func.h"

// OpenPilot UAVTalk:
#include "UAVTalk.h"


/* *************************************************/
/* ***************** DEFINITIONS *******************/

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port


/* **********************************************/
/* ***************** SETUP() *******************/

void setup() {
  
  Serial.begin(115200);
  
  SERIAL_PORT.begin(TELEMETRY_SPEED);  //definido en UAVTalk.h

}

void loop() {
  
    if (uavtalk_read()) {
        // imprimo lo que leo?
    } else {
        //espero? delay?
        delay(50);
    }
    
}


/* *********************************************** */
/* ******** functions used in main loop() ******** */

