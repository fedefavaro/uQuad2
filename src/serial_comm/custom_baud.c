/**
 ******************************************************************************
 *
 * @file       custom_baud.c
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      Modifica arbitrariamente el baudrate de un puerto serie.
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

#include "custom_baud.h"
#include <quadcop_config.h>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <asm/termios.h>
#include <stropts.h>

#define BAUDRATE		100000

/**
 * Permite establecer cualquier velocidad de transmicion en linux.
 * stty solo permite valores predefinidos : 9600, 19200, 38400, 57600,...
 */
int custom_baud(int fd)
{
  struct termios2 tio; 
  
  ioctl(fd, TCGETS2, &tio);
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = BAUDRATE;
  tio.c_ospeed = BAUDRATE;
  int r = ioctl(fd, TCSETS2, &tio);

#if DEBUG
  if (r == 0)
     printf("Baudrate seteado a 100000.\n");
  else 
     perror("ioctl");
#endif

  return 0;

}
