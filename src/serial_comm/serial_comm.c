/**
 ******************************************************************************
 *
 * @file       serial_comm.c
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      Permite manipular manualmente un puerto serie.
 * @see        ??
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

#include "serial_comm.h"
#include <uquad_error_codes.h>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
//#include <termios.h> /* POSIX terminal control definitions */

#include <sys/time.h>
#include <unistd.h>

#define LOOP_T_US               14000UL
#define MAX_ERR_CMD             20

#define sleep_ms(ms)    usleep(1000*ms)

/*
 * open_port(device) - Open serial port on device
 *
 * Returns the file descriptor on success or -1 on error.
 */
int open_port(char *device)
{
  int fd; /* File descriptor for the port */

  fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  //{
    //Could not open the port.
    err_log_str("open_port: Unable to open: ", device);
  //}
  //else
    //fcntl(fd, F_SETFL, 0);

  return fd;
}

int configure_port(int fd)
{
  struct termios options;
  
  // get the current options for the port...
  tcgetattr(fd, &options);				

  //set options
  cfsetispeed(&options, B115200); 			// set the in baud rate...
  cfsetospeed(&options, B115200);			// set the out baud rate...
  options.c_cflag |= (CLOCAL | CREAD);			// enable the receiver and set local mode...
  options.c_cflag |= PARENB;				// enable parity...
  options.c_cflag &= ~PARODD;				// set even parity...
  options.c_cflag |= CSTOPB;				// two stop bits...
  options.c_cflag &= ~CSIZE;				// mask the character size bits...
  options.c_cflag |= CS8;    				// select 8 data bits...
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 	// choosing raw input...
  options.c_iflag &= ~(IXON | IXOFF | IXANY); 		// disable software flow control...
  
  // set the new options for the port...
  tcsetattr(fd, TCSANOW, &options);

  return 0;

}

int configure_port_gps(int fd, speed_t baudrate)
{
  struct termios options;
  int rc;
 
  // get the current options for the port...
  if((rc = tcgetattr(fd, &options)) < 0){
     err_log_stderr("Error al obtener atributos: "); //logea strerror(errno)
     return -1;
  }

  //set options
  cfsetispeed(&options, baudrate); 			// set the in baud rate...
  cfsetospeed(&options, baudrate);			// set the out baud rate...
  options.c_cflag |= (CLOCAL | CREAD);			// enable the receiver and set local mode...
  options.c_cflag &= ~PARENB;				// No parity bit...
  options.c_cflag &= ~CSTOPB;				// 1 stop bits...
  options.c_cflag &= ~CSIZE;				// mask the character size bits...
  options.c_cflag |= CS8;    				// select 8 data bits...
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 	// choosing raw input...
  options.c_iflag &= ~(IXON | IXOFF | IXANY); 		// disable software flow control...
  
  // set the new options for the port...
  if((rc = tcsetattr(fd, TCSANOW, &options)) < 0){
     err_log_stderr("Error al aplicar nuevos atributos: "); //logea strerror(errno)
     return -1;
  }
  
  return 0;

}






