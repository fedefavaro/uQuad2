#include "serial_comm.h"
#include <uquad_error_codes.h>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

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
  {
   /*
    * Could not open the port.
    */
    err_log_str("open_port: Unable to open: ", device);
  }
  else
    fcntl(fd, F_SETFL, 0);

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






