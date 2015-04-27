#include <gps_comm.h>
#include <uquad_config.h>

char* hostName = "localhost";
char* hostPort = "1234";     // default port

struct gps_data_t my_gps_data;

int init_gps(void)
{
   int ret;

   ret = start_gpsd();
   if (ret < 0)
      return -1;
   
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
   (void)gps_stream(&my_gps_data, WATCH_DISABLE, NULL);
   (void)gps_close(&my_gps_data);

   ret = system(KILL_GPSD);
   if (ret < 0)
   {
      err_log("Failed to kill gpsd!");
      return -1;
   }

   return 0;
}


int start_gpsd(void)
{
   int ret = system(START_GPSD);
   if (ret < 0)
   {
      err_log("Failed to run gpsd!");
      return -1;
   }

   sleep(10); //espero que arranque el programa 

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
         return -1;

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







