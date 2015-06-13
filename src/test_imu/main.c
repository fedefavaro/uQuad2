/**
 ******************************************************************************
 *
 * @file       main.c
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      test imu
 * @see        src/main/README for information regarding how to run, configure, etc.
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

#include <stdlib.h>

#include <imu_comm.h>
#include <uquad_aux_math.h>
#include <uquad_aux_time.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

/// Global vars
int fd;

/// Forward decs
int configure_port(int fd, speed_t baudrate);

int main(int argc, char *argv[])
{
   int retval;

   set_signals();

   /// Abrir puerto de la imu
   fd = open_port("/dev/ttyUSB0");
   if (fd < 0) {
      puts("No se pudo abrir el puerto");
      return -1;
   }
   printf("IMU conectada - fd: %d\n",fd); //dbg


   retval = configure_port(fd, B115200); //TODO cambiar nombre
   if (retval < 0) {
      puts("No se pudo configurar el puerto");
      return -1;
   }

   // Clean serial buffer
   tcflush(fd,TCIOFLUSH);

   // Change IMU to binary mode.
   retval = write(fd,"!",1);
   if (retval < 1) {
      puts("No se pudo pasar la IMU a binario");
      quit();
   }

   /// Inicializo variables IMU
   imu_raw_t imu_raw;
   imu_data_t *imu_data;
   imu_data_alloc(imu_data);
    

   /// Calibraciones IMU
   //acc_calib_init();
   //gyro_calib_init();
   //magn_calib_init();
   //pres_calib_init();

   // -- -- -- -- -- -- -- -- -- 
   // Loop
   // -- -- -- -- -- -- -- -- -- 
   for(;;)
   {
	//para tener tiempo de entrada en cada loop
	gettimeofday(&tv_in_loop,NULL);

	IMU_readOK = check_read_locks(fd);
	if (IMU_readOK) {

            
            // Paso los datos del buffer RX a imu_raw.
            imu_comm_parse_frame_binary(&imu_raw);
            //imu_raw2data(&imu_raw, imu_data);
            
            print_imu_raw(&imu_raw);
            
	    // Reseteo imu_ready.
            reset_imu_ready();
        }

	/// Control de tiempos del loop
	wait_loop_T_US(MAIN_LOOP_50_MS,tv_in_loop);

   } // for(;;)

   return 0; //nunca llego aca

} //FIN MAIN



// -- -- -- -- -- -- -- -- -- 
// Funciones auxiliares
// -- -- -- -- -- -- -- -- --

/*********************************************/
/**** Para terminar la ejecucion limpiamente */
/*********************************************/
void quit(void)
{
   int retval;
   retval = close(fd);
   if(retval < 0)
      puts("No se pudo cerrar puerto. Ya esta cerrado?");
   exit(0);
}


/*********************************************/
/*********** Manejo de senales ***************/
/*********************************************/
void uquad_sig_handler(int signal_num)
{

   // Si se capturo SIGINT o SIGQUIT termino el programa
   err_log_num("Caught signal:",signal_num);
   quit();
}
void set_signals(void)
{
  
   // Catch signals
   signal(SIGINT,  uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);
}


int configure_port(int fd, speed_t baudrate)
{
  struct termios options;
  int rc;
 
  // get the current options for the port...
  if((rc = tcgetattr(fd, &options)) < 0){
     err_log_stderr("Error al obtener atributos: "); //logea strerror(errno)
     return -1;
  }

  //set options
  cfsetispeed(&options, baudrate); 			// set the in baud rate...
  cfsetospeed(&options, baudrate);			// set the out baud rate...
  cfmakeraw(&options);
  options.c_cflag |= (CLOCAL | CREAD);			// enable the receiver and set local mode...
  //options.c_cflag &= ~PARENB;				// No parity bit...
  //options.c_cflag &= ~CSTOPB;				// 1 stop bits...
  //options.c_cflag &= ~CSIZE;				// mask the character size bits...
  //options.c_cflag |= CS8;    				// select 8 data bits...
  //options.c_lflag &= ~(ICANON | ECHO | ECHOE | /*ISIG*/ ECHOK); 	// choosing raw input...
  //options.c_iflag &= ~(IXON | IXOFF | IXANY); 		// disable software flow control...
  
  // set the new options for the port...
  if((rc = tcsetattr(fd, TCSANOW, &options)) < 0){
     err_log_stderr("Error al aplicar nuevos atributos: "); //logea strerror(errno)
     return -1;
  }
  
  return 0;

}


/**
 * calcula diferencia entre tiempo de entrada al loop ('tv_in')y tiempo
 * actual ('tv_end') y manda a dormir la cantidad de tiempo que falte 
 * para completar 'loop_duration_usec'.
 */
int wait_loop_T_US(unsigned long loop_duration_usec, struct timeval tv_in)
{
   struct timeval tv_end, tv_diff;
   gettimeofday(&tv_end,NULL);
   int retval = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
   if(retval > 0)
   {
      if(tv_diff.tv_usec < loop_duration_usec)
         usleep(loop_duration_usec - (unsigned long)tv_diff.tv_usec); // Sobro tiempo, voy a dormir
   } else
         err_log("WARN: Main Absurd timing!");
   
#if DEBUG_TIMING_MAIN
      gettimeofday(&tv_end,NULL);
      retval = uquad_timeval_substract(&tv_diff, tv_end, tv_in);
      printf("duracion loop main: %lu\n",(unsigned long)tv_diff.tv_usec);
#endif

   return retval;
}


