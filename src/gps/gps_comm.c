/**
 ******************************************************************************
 *
 * @file       gps_comm.c
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      Implementa la comunicacion con el gps a traves de gpsd.
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
 */

#include <gps_comm.h>
#include <uquad_config.h>
#include <uquad_aux_time.h>

char* hostName = "localhost";
char* hostPort = "1234";     // default port

struct gps_data_t my_gps_data;

int init_gps(void)
{
   int ret,child_pid;
   child_pid = start_gpsd();
   if (child_pid < 0)
      return -1;
   
   sleep_ms(100);

   ret = gps_open(hostName, hostPort, &my_gps_data);
   if(ret < 0)
   {
      err_log("No se pudo abrir el puerto");
      return -1;
   }
   //sleep(100); //espero que arranque el programa 

   (void) gps_stream(&my_gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

   return child_pid;
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
   //Forks main program and starts client
   int child_pid = fork();  

   //-- -- -- El child ejecuta el siguiente codigo -- -- --
   if (child_pid == 0)
   {
      int retval;
      //starts sbus daemon
      retval = execl(START_GPSD_PATH, "gpsd",START_GPSD_DEV,"-S",START_GPSD_PORT, "-N",
//en debug level me tira mucha cosa
//                     "-D9",
                     (char*) 0);
      //only get here if execl failed 
      if(retval < 0)
      {
         err_log_stderr("Failed to run gpsd (execl)!");
         return -1;
      }
   }

   //-- -- -- El parent (main) ejecuta el siguiente codigo -- -- --
   return child_pid;

}


int get_gps_data(void)
{
   printf("entre 0\n");
   /* Put this in a loop with a call to a high resolution sleep () in it. */
   if (gps_waiting(&my_gps_data, 500)) {
      errno = 0;
      if (gps_read(&my_gps_data) == -1)
      {
         printf("entre 2\n");
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



