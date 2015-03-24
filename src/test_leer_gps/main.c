#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>

#define BAUDRATE		115200 //57600
#define DEVICE			"/dev/ttyUSB0"
#define BUFF_LENGTH		8

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
    printf("Caught signal: %d",signal_num);
    quit();
} 


int connect_gps(gps_t *gps, const char *device)
{
    char str[128];
    int retval;

    gps->fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if(gps->fd < 0)
    {
	fputs("open()\n", stderr);
        return -1;
    }
    
    retval = sprintf(str,"stty -F %s %d -echo raw", device, BAUDRATE);
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
    
    printf("GPS conectado en: %s\n", DEVICE);
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

    return retval;
}


gps_t *init_gps(const char *device)
{
    gps_t *gps;
    gps = (gps_t *)malloc(sizeof(gps_t));
    
    int retval = connect_gps(gps,device);    
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


int main(int argc, char *argv[])
{  
   int retval;
   uint8_t buff_gps[BUFF_LENGTH];

   gps = init_gps(DEVICE);
   
   // Catch signals
   signal(SIGINT, uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);

   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   for(;;)
   {
      
      retval = read(gps->fd,buff_gps,BUFF_LENGTH-1);
      if(retval < 0)
      {
         //fputs("Read error: no data!\n", stderr);
      } else {
         buff_gps[BUFF_LENGTH-1] = '\0';
         printf("%s",buff_gps);
      }
   
      sleep_ms(50);
      
   }

   return 0; //never reaches here

}






