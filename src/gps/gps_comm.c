#include <gps_comm.h>
#include <uquad_config.h>

char* hostName = "localhost";
char* hostPort = "1234";     // default port

struct gps_data_t my_gps_data;

int init_gps(void)
{
   int ret;

   //retval = start_gpsd();
   //sleep(5) //?? o mejor lo hago en start_gpsd() ?

   ret = gps_open(hostName, hostPort, &my_gps_data);
   if(ret < 0)
   {
      err_log("No se pudo abrir el puerto");
      return -1;
   }
   (void) gps_stream(&my_gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

   return 0;
}


int deinit_gps(void)
{
   int ret = 0;

   /* When you are done... */
   (void) gps_stream(&my_gps_data, WATCH_DISABLE, NULL);
   (void) gps_close (&my_gps_data);

   ret = system(KILL_GPSD);
   //if (ret ... TODO

   return 0;
}


int start_gpsd(void)
{
   //ret = system(START_GPSD);
   //if (ret ... TODO

   //sleep(5); //aca? 

   return 0;
}

int get_gps_data(void)
{
   /* Put this in a loop with a call to a high resolution sleep () in it. */
   if (gps_waiting(&my_gps_data, 500)) {
      errno = 0;
      if (gps_read(&my_gps_data) == -1)
      {
         //que hago si falla...
         err_log("No se pudo leer datos del gps");
      } else {
         /* Display data from the GPS receiver. */
         //if (gps_data.set & ...
            printf("latitude: %lf\n", my_gps_data.fix.latitude);
            printf("longitude: %lf\n", my_gps_data.fix.longitude);
            printf("altitude: %lf\n", my_gps_data.fix.altitude);
      }
   } 
#if DEBUG
   else err_log("No entro a gps_waiting() ... ???");
#endif
   return 0;
}







