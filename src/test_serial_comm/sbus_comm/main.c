#include "FutabaSBUS.h"
#include "SerialPort.h"
#include "CustomBaud.h"

#include <stdio.h>   /* Standard input/output definitions */
//#include <string.h>  /* String function definitions */
#include <errno.h>   /* Error number definitions */
#include <sys/time.h>
#include <unistd.h>
#include <stdint.h>

#define ERROR_OK		0
#define ERROR_FAIL		-1
#define LOOP_T_US               14000UL
#define MAX_ERR_CMD             20

#define sleep_ms(ms)    usleep(1000*ms)

int uquad_timeval_substract (struct timeval * result, struct timeval x, struct timeval y){
    /* Perform the carry for the later subtraction by updating y. */
    if (x.tv_usec < y.tv_usec) {
	int nsec = (y.tv_usec - x.tv_usec) / 1000000 + 1;
	y.tv_usec -= 1000000 * nsec;
	y.tv_sec += nsec;
    }
    if (x.tv_usec - y.tv_usec > 1000000) {
	int nsec = (y.tv_usec - x.tv_usec) / 1000000;
	y.tv_usec += 1000000 * nsec;
	y.tv_sec -= nsec;
    }
    
    /* Compute the time remaining to wait.
       tv_usec is certainly positive. */
    result->tv_sec = x.tv_sec - y.tv_sec;
    result->tv_usec = x.tv_usec - y.tv_usec;
    
    if(x.tv_sec < y.tv_sec)
	// -1 if diff is negative
	return -1;
    if(x.tv_sec > y.tv_sec)
	// 1 if diff is positive
	return 1;
    // second match, check usec
    if(x.tv_usec < y.tv_usec)
	// -1 if diff is negative
	return -1;
    if(x.tv_usec > y.tv_usec)
	// 1 if diff is positive
	return 1;

    // 0 if equal
    return 0;
}


int main(int argc, char *argv[])
{  
   
   int fd;
   int ret = ERROR_OK;
   int do_sleep = 0;
   int err_count = 0;
   
   //buffer for sbus message
   uint8_t* sbusData;			
   sbusData = FutabaSBUS_ptrsbusData();

   struct timeval tv_in;
   struct timeval tv_end;
   struct timeval tv_diff;

   fd = open_port();
   if (fd == -1)
   { 
       return ERROR_FAIL;  
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
	
        FutabaSBUS_updateServos();
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



