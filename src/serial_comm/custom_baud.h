/**
 ******************************************************************************
 *
 * @file       custom_baud.h
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      Modifica arbitrariamente el baudrate de un puerto serie.
 * @see        serial_comm/serial_comm.h
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
 /*
  * Lo tuve que hacer en una libreria separada de serial_comm porque
  * los encabezados <termios.h> y <asm/termios.h> son incompatibles.
  */

#ifndef CUSTOM_BAUD_h
#define CUSTOM_BAUD_h

int custom_baud(int fd);

#endif


