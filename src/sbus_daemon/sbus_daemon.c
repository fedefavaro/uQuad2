#include <futaba_sbus.h>
#include <serial_comm.h>
#include <custom_baud.h>
#include <uquad_aux_time.h>
#include <uquad_error_codes.h>

#include <stdio.h>   /* Standard input/output definitions */
//#include <string.h>  /* String function definitions */
#include <errno.h>   /* Error number definitions */
#include <unistd.h>
#include <stdint.h>

f#define LOOP_T_US               14000UL
#define MAX_ERR_CMD             20

#define HOW_TO     "./sbus_daemon <device>"

int main(int argc, char *argv[])
{  
   
   int fd;
   int ret = ERROR_OK;
   int do_sleep = 0;
   int err_count = 0;
   char *device;

   if(argc<2)
   {
      err_log(HOW_TO);
      return -1;
   }
   else
      device = argv[1];
   
   //buffer for sbus message
   uint8_t* sbusData;			
   sbusData = futaba_sbus_ptrsbusData();

   struct timeval tv_in;
   struct timeval tv_end;
   struct timeval tv_diff;

   fd = open_port(device);
   if (fd == -1)
   { 
       return -1;
   }
   configure_port(fd);
   
   ret = custom_baud(fd);      
   if (ret < 0)
   {
       fputs("custom_baud() failed!\n", stderr);
       return ret;
   }
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   for(;;)
   {
	gettimeofday(&tv_in,NULL);
        if(err_count > MAX_ERR_CMD)
	{
	    printf("error count exceded");
            return -1;
	}
	if(do_sleep)
	{
	    // avoid saturating i2c driver
	    //printf("Will sleep to avoid saturating.");
	    sleep_ms(10);
	    do_sleep = 0;
	}
	
        futaba_sbus_updateServos();
	ret = write(fd, sbusData, 25);
        if (ret < 0)
        {
           fputs("write() failed!\n", stderr);
           do_sleep = 1;
	   continue;
	}
	else
	{
	    /// This loop was fine
	    if(err_count > 0)
		err_count--;
	}
	
	gettimeofday(&tv_end,NULL);

	ret = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
	/// Check if we have to wait a while
	if(ret > 0)
	{
	    if(tv_diff.tv_usec < LOOP_T_US)
	    {
		usleep(LOOP_T_US - (unsigned long)tv_diff.tv_usec);
	    }

	}
	else
	{
	    err_count++;
	}
   } //for(;;)  

   return 0; //never reaches here

}



