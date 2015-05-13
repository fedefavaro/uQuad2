/**
 ******************************************************************************
 *
 * @file       main.c
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      QuadCop autopilot software
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

#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <quadcop_config.h>
#include <uquad_aux_time.h>
#include <uquad_aux_io.h>
#include <futaba_sbus.h>
#include <gps_comm.h>
#include <UAVTalk.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#define CH_COUNT		5
#define BUFF_SIZE		10

#define KILL_SBUS		"killall sbusd"

/// Global vars

// Almacena pids de hijos
pid_t sbusd_child_pid = 0;
pid_t gpsd_child_pid = 0;

// GPS
gps_t gps;

// IO
static io_t *io  	= NULL;
uquad_bool_t read_ok	= false; //flag para determinar si se puede leer de un dispositivo
unsigned char tmp_buff[2] = {0,0};  // Para leer entrada de usuario

// KMQ
uquad_kmsgq_t *kmsgq 	= NULL;
// Valores iniciales de los canales a enviar a sbusd
//ch_buff[0]  roll
//ch_buff[1]  pitch
//ch_buff[2]  yaw
//ch_buff[3]  throttle
//ch_buff[4]  flight mode?
uint16_t ch_buff[CH_COUNT]={1500,1500,1500,1500,1500}; 
uint8_t *buff_out=(uint8_t *)ch_buff; // buffer para enviar mensajes de kernel
                                      // El casteo es necesario para enviar los mensajes de kernel (de a 1 byte en lugar de 2)

// UAVTalk
int fd_CC3D;

/// Declaracion de funciones auxiliares
void quit(int Q);
void uquad_sig_handler(int signal_num);
void set_signals(void);
void read_from_stdin(void);


/*********************************************/
/**************** Main ***********************/
/*********************************************/
int main(int argc, char *argv[])
{  
   int retval;

   //setea senales y mascara 
   set_signals(); 

   // Control de tiempos
   struct timeval tv_in;
   
   // -- -- -- -- -- -- -- -- -- 
   // Inicializacion
   // -- -- -- -- -- -- -- -- -- 

   ///GPS config - Envia comandos al gps a traves del puerto serie - //

#if !DISABLE_GPS
   retval = preconfigure_gps();
   if(retval < 0)                                                 
   {                                                                        
      err_log("Failed to preconfigure gps!");
      quit(0);                                                              
   } 
   sleep_ms(1000);  //necesario? TODO verificar
  

   /// Ejecuta GPS daemon - proceso independiente
   gpsd_child_pid = init_gps();
   if(gpsd_child_pid == -1)
   {
      err_log_stderr("Failed to init gps!");
      quit(0);
   }
#endif //!DISABLE_GPS


   /// Ejecuta Demonio S-BUS - proceso independiente
   sbusd_child_pid = futaba_sbus_start_daemon();                            
   if(sbusd_child_pid == -1)                                                
   {                                                                        
      err_log_stderr("Failed to start child process (sbusd)!");             
      quit(1);                                                              
   }


   /// inicializa kernel messages queues - para comunicacion con sbusd
   kmsgq = uquad_kmsgq_init(SERVER_KEY, DRIVER_KEY);
   if(kmsgq == NULL)
   {
      quit_log_if(ERROR_FAIL,"Failed to start message queue!");
   }

   //Doy tiempo a que inicien bien los procesos...
   sleep_ms(500);   


   /// inicializa IO manager
   io = io_init();
   if(io==NULL)
   {
      quit_log_if(ERROR_FAIL,"io init failed!");
   }
   retval = io_add_dev(io,STDIN_FILENO);  // Se agrega stdin al io manager
   quit_log_if(retval, "Failed to add stdin to io list"); 


   /// inicializa UAVTalk
//------------------------------------------------------------------------
   fd_CC3D = uav_talk_init();
   bool CC3D_readOK;
//------------------------------------------------------------------------

   /// mensajitos al usuario...
#if PC_TEST
   err_log("WARNING: Comenzando en modo 'PC test' - Ver common/quadcop_config.h");
#endif

#if DISABLE_GPS
   err_log("WARNING: GPS disabled!");
#endif


   // -- -- -- -- -- -- -- -- -- 
   // Loop
   // -- -- -- -- -- -- -- -- -- 
   for(;;)
   {
      gettimeofday(&tv_in,NULL); //para tener tiempo de entrada en cada loop

      /// Polling de dispositivos IO
//      retval = io_poll(io);
//      quit_log_if(retval,"io_poll() error");      
      
      /// Check stdin
//      retval = io_dev_ready(io,STDIN_FILENO,&read_ok,NULL);
//      log_n_continue(retval, "Failed to check stdin for input!");
//      if(read_ok)
//      {
//         read_from_stdin();
//      }

#if !DISABLE_GPS
      /// Obener datos del GPS
      retval = get_gps_data(&gps);
      if (retval < 0 )
      {  
         //que hago si NO hay datos!?
         err_log("No hay datos de gps");
      } else {
         //que hago si SI hay datos!?
         printf("%lf\t%lf\t%lf\t%lf\t%lf\n",   \
                gps.latitude,gps.longitude,gps.altitude,gps.speed,gps.track);
      }
#endif

//UAVTalk TODO esto deberia estar en un loop mucho mas rapido
//------------------------------------------------------------------------
     CC3D_readOK = check_read_locks(fd_CC3D);
     if (CC3D_readOK) {
		
        if (uavtalk_read(fd_CC3D)) {
           // imprimo lo que leo?
        } else {
           err_log("uavtalk_read failed");
        }
     } else err_log("UAVTalk: read NOT ok");
//------------------------------------------------------------------------

      // Envia actitud y throttle deseados a sbusd (a traves de mensajes de kernel)
      retval = uquad_kmsgq_send(kmsgq, buff_out, MSGSZ);
      if(retval != ERROR_OK)
      {
         quit_log_if(ERROR_FAIL,"Failed to send message!");
      }

      /// Control de tiempos del loop
      wait_loop_T_US(MAIN_LOOP_T_US,tv_in);

     
   } // for(;;)

   return 0; //nunca deberia llegar aca

} //FIN MAIN



// -- -- -- -- -- -- -- -- -- 
// Funciones auxiliares
// -- -- -- -- -- -- -- -- --

/*********************************************/
/**** Para terminar la ejecucion limpiamente */
/*********************************************/
/**
 * Interrumpe ejecucion del programa. Dependiendo del valor del parametro
 * que se le pase interrumpe mas o menos cosas.
 * Q == 2 : interrupcion manual (ctrl-c), cierra todo y avisa que fue manual
 * Q == 1 : interrupcion por muerte del sbusd, cierra todo menos sbusd
 * Q == 0 : interrupcion estandar, cierra todo
 * Q cualquier otro : igual que Q == 0.
 *
 * TODO QUE HACER CUANDO ALGO FALLA Y NECESITAMOS APAGAR LOS MOTORES!
 * TODO SI SE CIERRA GPSD TRATAR DE ABRIRLO DENUEVO?
 */
void quit(int Q)
{
   int retval;
   
   if(Q == 2) {
      err_log("interrumpido manualmente (ctrl-c)");  //TODO no funco bien en la beagle
   } else
      err_log("algo salio mal, cerrando...");
   
   if(Q != 1) {
      /// Terminar Demonio S-BUS 
      retval = system(KILL_SBUS);
      //sprintf(str, "kill -SIGTERM %d", sbusd_child_pid);
      //retval = system(str);         
      if (retval < 0)
         err_log("Could not terminate sbusd!");
   }
   
   /// cerrar IO manager
   retval = io_deinit(io);
   if(retval != ERROR_OK)
      err_log("Could not close IO correctly!");

   /// Kernel Messeges Queue
   uquad_kmsgq_deinit(kmsgq);
   
#if !DISABLE_GPS
   if(Q != 2) {
      /// cerrar conexiones con GPSD y terminarlo
      retval = deinit_gps();
      if(retval != ERROR_OK)
         err_log("Could not close gps correctly!");
   }
#endif //DISABLE_GPS

   /// cerrar UAVTalk
   retval = uav_talk_deinit(fd_CC3D);
   if(retval != ERROR_OK)
      err_log("Could not close UAVTalk correctly!");
      
   exit(0);

}


/*********************************************/
/*********** Manejo de senales ***************/
/*********************************************/

sigset_t mask;
sigset_t orig_mask;

void uquad_sig_handler(int signal_num)
{

   // Si se murio el demonio sbus termino el programa
   if (signal_num == SIGCHLD)
   {
      pid_t p;
      int status;
      p = waitpid(-1, &status, WNOHANG);
      if(p == sbusd_child_pid)
      {
         err_log_num("WARN: sbusd died! sig num:", signal_num);
         quit(1); //exit sin cerrar sbusd
      } else if(p == gpsd_child_pid) {
         err_log_num("WARN: gpsd died! sig num:", signal_num);
         quit(0); //exit sin cerrar gpsd
      } else {
         err_log_num("SIGCHLD desconocido, return:", signal_num);
         //quit(0);
         return;
      }
   }
   
   // bloqueo SIGCHLD si entre por SIGINT para no entrar 2 veces
   if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0) {
      perror ("sigprocmask");
   }

   // Si se capturo SIGINT o SIGQUIT termino el programa
   err_log_num("Caught signal:",signal_num);
   quit(2);
}

void set_signals(void)
{
   //mascaras para bloquear SIGCHLD
   sigemptyset (&mask);
   sigaddset (&mask, SIGCHLD);
   //sigaddset (&mask, SIGTERM);
  
   // Catch signals
   signal(SIGINT,  uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);
   signal(SIGCHLD, uquad_sig_handler);

}


/*********************************************/
/************* Leer de stdin *****************/
/*********************************************/

void read_from_stdin(void)
{
         int retval = fread(tmp_buff,sizeof(unsigned char),1,stdin); //TODO corregir que queda algo por leer en el buffer?
         if(retval <= 0)
         {
	    //log_n_jump(ERROR_READ, end_stdin,"No user input detected!");
            err_log_num("No user input detected!",ERROR_READ);
            return;
         }
         switch(tmp_buff[0])
         {
         case '0':
            ch_buff[0] = 1500;
            break;
         case '1':
            ch_buff[0] = 1550;
            break;
         case '2':
            ch_buff[0] = 1600;
            break;
         case '3':
            ch_buff[0] = 1650;
            break;
         case 'F':
            err_log("Failsafe set");
            ch_buff[4] = 50;
            break;
         case 'f':
            err_log("Failsafe clear");
            ch_buff[4] = 100;
            break;
         default:
            err_log("Velocidad invalida. Ingrese 0,1,2,3 para 0%,10%,20%,30%");
            break;
         } //switch(tmp_buff[0])
}






