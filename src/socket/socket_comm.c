/**
 ******************************************************************************
 *
 * @file       socket_comm.c
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

#include "socket_comm.h"
#include <uquad_error_codes.h>

#define SOCKET_BUFF_SIZE	2

int serversock, clientsock;
struct sockaddr_in echoserver, echoclient;

int socket_comm_init(void)
{
  /* Create the TCP socket */
  if ((serversock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
     err_log("Failed to create socket");
     return -1;
  }

  int verdadero = 1;
  if (setsockopt(serversock,SOL_SOCKET,SO_REUSEADDR,&verdadero,sizeof(int)) == -1) {
     err_log("Failed to set socket options");
     return -1;
  }

  /* Construct the server sockaddr_in structure */
  memset(&echoserver, 0, sizeof(echoserver));       /* Clear struct */
  echoserver.sin_family = AF_INET;                  /* Internet/IP */
  echoserver.sin_addr.s_addr = htonl(INADDR_ANY);   /* Incoming addr */
  echoserver.sin_port = htons(PORT_DEF);       /* server port */
  
  /* Bind the server socket */
  if (bind(serversock, (struct sockaddr *) &echoserver,
           sizeof(echoserver)) < 0) {
      err_log("Failed to bind the server socket");
      return -1;
  }
  /* Listen on the server socket */
  if (listen(serversock, MAXPENDING) < 0) {
      err_log("Failed to listen on server socket");
      return -1;
  }

  return 0;
}



int socket_comm_wait_client(void)
{
  unsigned int clientlen = sizeof(echoclient);
  
  printf("----------------------\n  Esperando Cliente  \n----------------------\n");
  /* Wait for client connection */
  if ((clientsock =
       accept(serversock, (struct sockaddr *) &echoclient,
       &clientlen)) < 0) {
          err_log("Failed to accept client connection");
	  return -1;
  }
  fprintf(stdout, "Client connected: %s\n",
                              inet_ntoa(echoclient.sin_addr));

  return 0;
}


int socket_comm_send_path(Lista_path *lista)
{
   Elemento_path *elemento = lista->inicio;

   int lista_tamano = lista->tamano;

   double buffer_sock_double[19*lista_tamano+1];
   int buffer_sock_double_len = sizeof(buffer_sock_double);

   buffer_sock_double[0] = (double)lista->tamano;
 
   int i = 0;
   while (elemento != NULL) {
	buffer_sock_double[1+i*19] = RADIO;
	buffer_sock_double[2+i*19] = elemento->dato->xi;
	buffer_sock_double[3+i*19] = elemento->dato->yi;
	buffer_sock_double[4+i*19] = elemento->dato->anguloi;
	buffer_sock_double[5+i*19] = elemento->dato->xf;
	buffer_sock_double[6+i*19] = elemento->dato->yf;
	buffer_sock_double[7+i*19] = elemento->dato->angulof;
	buffer_sock_double[8+i*19] = (double)elemento->dato->tipo;
	buffer_sock_double[9+i*19] = elemento->dato->xci;
	buffer_sock_double[10+i*19] = elemento->dato->yci;
	buffer_sock_double[11+i*19] = elemento->dato->xri;
	buffer_sock_double[12+i*19] = elemento->dato->yri;
	buffer_sock_double[13+i*19] = elemento->dato->xrf;
	buffer_sock_double[14+i*19] = elemento->dato->yrf;
	buffer_sock_double[15+i*19] = elemento->dato->xcf;
	buffer_sock_double[16+i*19] = elemento->dato->ycf;
	buffer_sock_double[17+i*19] = elemento->dato->Ci;
	buffer_sock_double[18+i*19] = elemento->dato->S;
	buffer_sock_double[19+i*19] = elemento->dato->Cf;
	elemento = elemento->siguiente;
        i++;
   }

//int j = 0;
//for(j=0;j<19*lista_tamano+1;j++) {
//   printf("%lf\n",buffer_sock_double[j]);
//}

   /* Send back received data */
   if (send(clientsock, buffer_sock_double, buffer_sock_double_len, 0) != buffer_sock_double_len) {
        err_log("Failed to send bytes to client");
	return -1;
   }

   return 0;
}

int socket_comm_update_position(position_t position)
{
   double buffer_sock[SOCKET_BUFF_SIZE] = {0, 0};
   int buffer_len = sizeof(buffer_sock);
   buffer_sock[0] = position.x;
   buffer_sock[1] = position.y;
   if (send(clientsock, &buffer_sock, buffer_len, 0) != buffer_len) {
      err_log("Failed to send bytes to client");
      return -1;
   }
   return 0;
}

