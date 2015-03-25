#include <gps.h>
#include <stdbool.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

struct gps_data_t my_gps_data;

/*struct GPSdata {
    double latitude;
    double longitude;
    double altitude;
} myGPSdata;
*/

// ran for first time: initialization
char* hostName = "localhost";
char* hostPort = "2947";     // default port

void quit()
{
    int retval = 0;
    /* When you are done... */
	(void) gps_stream(&my_gps_data, WATCH_DISABLE, NULL);
	(void) gps_close (&my_gps_data);
    exit(retval);
}   

void uquad_sig_handler(int signal_num)
{
    printf("Caught signal: %d",signal_num);
    quit();
} 

int main(int argc, char *argv[])
{ 
    int ret;
	ret = gps_open(hostName, hostPort, &my_gps_data);
    if(ret<0);

	(void) gps_stream(&my_gps_data, WATCH_ENABLE | WATCH_JSON, NULL);


   // Catch signals
   signal(SIGINT, uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);

   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   // Loop
   // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
	for(;;)
	{

		/* Put this in a loop with a call to a high resolution sleep () in it. */
		if (gps_waiting (&my_gps_data, 500)) {
			errno = 0;
			if (gps_read (&my_gps_data) == -1) {
			//que hago si falla...
			} else {
				/* Display data from the GPS receiver. */
				//if (gps_data.set & ...
				printf("latitude: %lf\n", my_gps_data.fix.latitude);
				printf("longitude: %lf\n", my_gps_data.fix.longitude);
				printf("altitude: %lf\n", my_gps_data.fix.altitude);
    		}
		}
  	


	}
	
	return 0;
}



