/*
 * Sacado de: http://gnosis.cx/publish/programming/sockets.html
 */

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>

#include <time.h>
#include <stdint.h>

#define MAXPENDING 5    /* Max connection requests */
#define BUFFSIZE 32

void Die(char *mess) { perror(mess); exit(1); }

void HandleClient(int sock) {
            char buffer[BUFFSIZE];
            int received = -1;
            /* Receive message */
            if ((received = recv(sock, buffer, BUFFSIZE, 0)) < 0) {
              Die("Failed to receive initial bytes from client");
            }
            /* Send bytes and check for more incoming data in loop */
            while (received > 0) {
              /* Send back received data */
              if (send(sock, buffer, received, 0) != received) {
                Die("Failed to send bytes to client");
              }
              /* Check for more data */
              if ((received = recv(sock, buffer, BUFFSIZE, 0)) < 0) {
                Die("Failed to receive additional bytes from client");
              }
            }
            close(sock);
}


int main(int argc, char *argv[])
{    

  // -- -- -- -- -- -- -- -- -- 
  // Inicializacion
  // -- -- -- -- -- -- -- -- -- 
  int serversock, clientsock;
  struct sockaddr_in echoserver, echoclient;

  if (argc != 2) {
      fprintf(stderr, "USAGE: echoserver <port>\n");
      exit(1);
  }
  /* Create the TCP socket */
  if ((serversock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
     Die("Failed to create socket");
  }
  /* Construct the server sockaddr_in structure */
  memset(&echoserver, 0, sizeof(echoserver));       /* Clear struct */
  echoserver.sin_family = AF_INET;                  /* Internet/IP */
  echoserver.sin_addr.s_addr = htonl(INADDR_ANY);   /* Incoming addr */
  echoserver.sin_port = htons(atoi(argv[1]));       /* server port */
  
  /* Bind the server socket */
  if (bind(serversock, (struct sockaddr *) &echoserver,
           sizeof(echoserver)) < 0) {
      Die("Failed to bind the server socket");
  }
  /* Listen on the server socket */
  if (listen(serversock, MAXPENDING) < 0) {
      Die("Failed to listen on server socket");
  }
  
  int cont = 0;
  // -- -- -- -- -- -- -- -- -- 
  // Loop
  // -- -- -- -- -- -- -- -- --
  #define BUFF_SIZE	2
  double buffer[BUFF_SIZE] = {0, 0};
  int buffer_len = sizeof(buffer);

/* //Imprime bytes, swapea e imprime denuevo
   double buffer = 0.1;
   int buffer_len = sizeof(buffer);
   uint8_t *p = (uint8_t *)&buffer;
   uint8_t* i = p;
   while ( (int)(p-i) < buffer_len )
   {
      printf("%u\n",*p++);
   }

   double buffer2;
   uint8_t *p2 = (uint8_t *)&buffer2;
   int j = 0;
   while ( j++ < buffer_len )
   {
      *(p2++) = *(--p);
   }

   p = (uint8_t *)&buffer2;
   uint8_t* k = p;
   while ( (int)(p-k) < buffer_len )
   {
      printf("%u\n",*p++);
   }*/

  //exit(0);

  /* Run until cancelled */
            //while (1) {
              unsigned int clientlen = sizeof(echoclient);
              /* Wait for client connection */
              if ((clientsock =
                   accept(serversock, (struct sockaddr *) &echoclient,
                          &clientlen)) < 0) {
                Die("Failed to accept client connection");
              }
              fprintf(stdout, "Client connected: %s\n",
                              inet_ntoa(echoclient.sin_addr));
	      while (1) {

                 /* Send back received data */
                 if (send(clientsock, &buffer, buffer_len, 0) != buffer_len)
                    Die("Failed to send bytes to client");
		 buffer[0] += 0.5;
                 if (cont >= 100) {
		    buffer[1] += 50;
                    fprintf(stdout, "Escalon: %d\n", (int)buffer[1]);
		    cont = 0;
		 }
		 cont += 1;
                 usleep(1000*50);
                 
              //HandleClient(clientsock);
            }
}



