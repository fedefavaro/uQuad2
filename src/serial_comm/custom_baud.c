#include "custom_baud.h"

/*
* Allows to set arbitrary speed for the serial device on Linux.
* stty allows to set only predefined values: 9600, 19200, 38400, 57600, 115200, 230400, 460800.
*/
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <asm/termios.h>

#define BAUDRATE		115200//100000

int custom_baud(int fd)
{
  struct termios2 tio; 
  
  ioctl(fd, TCGETS2, &tio);
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = BAUDRATE;
  tio.c_ospeed = BAUDRATE;
  int r = ioctl(fd, TCSETS2, &tio);
 
  if (r == 0)
     printf("Changed successfully.\n");
  else 
     perror("ioctl");
   
  return 0;

}
