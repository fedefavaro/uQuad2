/**
 ******************************************************************************
 *
 * @file       serial_comm.h
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

#ifndef SERIAL_PORT_h
#define SERIAL_PORT_h


#include <termios.h> 

/**
 * Abre el puerto serie especificado por device.
 *
 * @return file descriptor o codigo de error
 */
int open_port(char *device);

/**
 * Configura manualmente el puerto serie (modificando registros).
 *
 * @param fd file descriptor de puerto a configurar
 * @return 
 */
int configure_port(int fd);

int configure_port_gps(int fd, speed_t baudrate);

void serial_flush(int fd);

#endif


