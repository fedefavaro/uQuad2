/**
 ******************************************************************************
 *
 * @file       socket_comm.h
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      Implementa cmunicacion con otra pc que tenga matlab para graficar trayectoria en tiempo real
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
 *
 */


#ifndef SOCKET_COMM_H
#define SOCKET_COMM_H

#include <stdlib.h>
#include <quadcop_types.h>
#include <path_planning.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define MAXPENDING 	5    /* Max connection requests */
#define BUFFSIZE 	32
#define PORT_DEF 	12345

int socket_comm_init(void);

int socket_comm_wait_client(void);

int socket_comm_send_path(Lista_path *lista);

int socket_comm_update_position(position_t position);

#endif // SOCKET_COMM_H



