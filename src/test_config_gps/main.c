#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define BAUDRATE_9600		9600
#define BAUDRATE_57600		57600
#define DEVICE				"/dev/ttyUSB0"

#define BUFF_LENGTH			8

#define GPS_UPDATE_10HZ 	"$PMTK220,100*2F\r\n"
#define GPS_BAUD_57600 		"$PMTK251,57600*2C\r\n"


#define sleep_ms(ms)    	usleep(1000*ms)


typedef struct gps {
   int fd;
} gps_t;

// global vars
gps_t *gps;

// forward defs
int deinit_gps(gps_t *gps);

void quit()
{
    int retval = 0;
    deinit_gps(gps);
    exit(retval);
}   


void uquad_sig_handler(int signal_num)
{
    printf("Caught signal: %d ",signal_num);
    quit();
} 


int connect_gps(gps_t *gps, const char *device, int baud)
{
    char str[128];
    int retval;

    gps->fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if(gps->fd < 0)
    {
	fputs("open()\n", stderr);
        return -1;
    }
    
    retval = sprintf(str,"stty -F %s %d -echo raw", device, baud);
    if(retval < 0)
    {
	fputs("sprintf()\n", stderr);
        return -1;
    }
    
    retval = system(str);
    if(retval != 0)
    {
	fputs("system()\n", stderr);
        return -1;
    }
    
    printf("GPS conectado en: %s a %d baudios\n", device, baud);
    return 0;

}


int disconnect_gps(int fd)
{
    int retval = 0;

    if(fd > 0)
    {
	retval = close(fd);
        if(retval < 0) {
           fputs("Failed to close device!", stderr);
           return -1;
        }
    } else {
        fputs("fd < 0", stderr);
        return -1;
    }
 
    printf("GPS desconectado\n");
    return retval;
}


gps_t *init_gps(const char *device, int baud)
{
    gps_t *gps;
    gps = (gps_t *)malloc(sizeof(gps_t));
    
    int retval = connect_gps(gps,device,baud);    
    if(retval < 0)
    {
       fputs("Could not connect gps.\n", stderr);
       quit(); 
    }
    
    return gps;
}


int deinit_gps(gps_t *gps)
{
    disconnect_gps(gps->fd);
    if(gps == NULL)
    {
	fputs("WARN: Nothing to free.\n", stderr);
        return 0;
    }
    free(gps);
    
    return 0;
}

int send_command_gps(gps_t *gps, const char *command)
{
   int retval;
   int length = 0;
   length = strlen(command);
   
   if(length < 0)
   {
		fputs("strlen()\n", stderr);
		return -1;
   }
   retval = write(gps->fd, command, length);
   if(retval < 0)
   {
      fputs("Write error: no data!\n", stderr);
      return -1;
   }

   return 0;
}


int main(int argc, char *argv[])
{  
   int retval;
   gps = init_gps(DEVICE,BAUDRATE_9600);
   
   sleep_ms(5);
   retval = send_command_gps(gps, GPS_BAUD_57600);
   sleep_ms(5); 
   //retval = send_command_gps(gps, GPS_BAUD_57600);
   //sleep_ms(5);

   disconnect_gps(gps->fd);
   connect_gps(gps, DEVICE, BAUDRATE_57600);
   sleep_ms(5); 
   retval = send_command_gps(gps, GPS_UPDATE_10HZ);
   sleep_ms(5);
   //retval = send_command_gps(gps, GPS_UPDATE_10HZ);
   //sleep_ms(5);

   uint8_t buff_gps[BUFF_LENGTH];
   uint8_t inByte=0;
   uint8_t index = 0;
   bool received = false;
   // Catch signals
   signal(SIGINT, uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);

   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   for(;;)
   {
      //sleep_ms(1);
      retval = read(gps->fd,&inByte,1);
      if(retval < 0)
      {
         //fputs("Read error: no data!\n", stderr);
      } else {
   			
			buff_gps[index++] = inByte;
			if(inByte == '\n')
			{
				buff_gps[index] = 0;
				index = 0;
				received = true;
			}
         
			if(received)
			{	
				received = false;
				printf("%s",buff_gps);
      			//sleep_ms(10);
			}
	  }
      
   }

   return 0; //never reaches here

}






