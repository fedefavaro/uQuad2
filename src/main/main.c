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
#include <path_planning.h>
//#include <path_following.h>
#include <futaba_sbus.h>
#include <serial_comm.h>
#include <gps_comm.h>
#include <UAVTalk.h>
#include <control_yaw.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <fcntl.h>
#include <stdlib.h>

#define CH_COUNT		6
#define BUFF_SIZE		12

#define KILL_SBUS		"killall sbusd"

/// Global vars

// Almacena pids de hijos
pid_t sbusd_child_pid = 0;
pid_t gpsd_child_pid = 0;

// Para prueba yaw
uint16_t throttle_inicial = 0;

// Path planning
Lista_path* lista_path;
Lista_wp* lista_way_point;
way_point_t wp = {0,0,0,0};

// GPS
gps_t gps;
bool gps_updated = false;

// LOG
int log_fd;

// IO
static io_t *io  	= NULL;
uquad_bool_t read_ok	= false; //flag para determinar si se puede leer de un dispositivo
unsigned char tmp_buff[2] = {0,0};  // Para leer entrada de usuario

// KMQ
uquad_kmsgq_t *kmsgq 	= NULL;

/** 
 * Valores iniciales de los canales a enviar al proceso sbusd
 *
 * ch_buff[0]  roll
 * ch_buff[1]  pitch
 * ch_buff[2]  yaw
 * ch_buff[3]  throttle
 * ch_buff[4]  flight mode
 * ch_buff[5]  activar/desactivar failsafe
 *
 * init todo en cero y flight mode en 2 
 */
uint16_t ch_buff[CH_COUNT]={1500,1500,1500,950,2000,0};

/** 
 * Buffer para enviar mensajes de kernel
 * El casteo es necesario para enviar los mensajes de kernel
 * (de a 1 byte en lugar de 2)
 */
uint8_t *buff_out=(uint8_t *)ch_buff;

// UAVTalk
int fd_CC3D;
actitud_t act = {0,0,0,{0,0}}; //almacena variables de actitud leidas de la cc3d y timestamp
actitud_t act_last;
double yaw_rate;
double yaw_zero = 0;
bool uavtalk_updated = false;

// Control de yaw
double u = 0; //senal de control (setpoint de velocidad angular)
double yaw_d = 0;

// Control activado/desactivado // TODO Mover a contorl_yaw
typedef enum {
	STOPPED = 0,
	STARTED,
        FINISHED
} estado_control_t;
estado_control_t control_status = STOPPED;

// Almacena posicion actual del quad // TODO sacar aca
typedef struct posicion {
   double x;
   double y;
   double z;
   struct timeval ts;
} posicion_t;
posicion_t posicion = {0,0,0,{0,0}};

#if !SIMULATE_GPS
// Almacena velovidad actual del quad // TODO sacar aca
typedef struct velocidad {
   double module;
   double angle;
   struct timeval ts;
} velocidad_t;
velocidad_t velocidad = {0,0,{0,0}};
#else
// Almacena velovidad actual del quad // TODO sacar aca
typedef struct velocidad {
   double x;
   double y;
   double z;
   struct timeval ts;
} velocidad_t;
velocidad_t velocidad = {0,0,0,{0,0}};
#endif

// Almacena masa del quad // TODO sacar aca
double masa = 1.85; // kg
double g = 9.81; // m/s*s
double B = 1; // coef friccion
#define PITCH_DESIRED		9 //grados
#include <math.h>
double pitch = PITCH_DESIRED*M_PI/180; // angulo de pitch en radianes
double last_yaw_measured = M_PI/6;

/// Declaracion de funciones auxiliares
void quit(int Q);
void uquad_sig_handler(int signal_num);
void set_signals(void);
void read_from_stdin(void);
// Convierte angulo de yaw a senal de pwm para enviar a la cc3d // TODO scar de aca
uint16_t convert_yaw2pwm(double yaw);
// SIMULACION GPS TODO SACAR DE ACA
void simulate_gps(posicion_t* pos, velocidad_t* vel, double yaw_measured);

//-----------------------------------------------------------
#if SOCKET_TEST
   #include <sys/socket.h>
   #include <arpa/inet.h>
   #include <netinet/in.h>

   #define MAXPENDING 	5    /* Max connection requests */
   #define BUFFSIZE 	32
   #define PORT_DEF 	12345

   void Die(char *mess) { perror(mess); quit(1); }
#endif
//-----------------------------------------------------------

/*********************************************/
/**************** Main ***********************/
/*********************************************/
int main(int argc, char *argv[])
{

//-----------------------------------------------------------
#if SOCKET_TEST
  int serversock, clientsock;
  struct sockaddr_in echoserver, echoclient;

  /* Create the TCP socket */
  if ((serversock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
     Die("Failed to create socket");

  int verdadero = 1;
  if (setsockopt(serversock,SOL_SOCKET,SO_REUSEADDR,&verdadero,sizeof(int)) == -1)
     Die("Failed to set socket options");
  
  /* Construct the server sockaddr_in structure */
  memset(&echoserver, 0, sizeof(echoserver));       /* Clear struct */
  echoserver.sin_family = AF_INET;                  /* Internet/IP */
  echoserver.sin_addr.s_addr = htonl(INADDR_ANY);   /* Incoming addr */
  echoserver.sin_port = htons(PORT_DEF);       /* server port */
  
  /* Bind the server socket */
  if (bind(serversock, (struct sockaddr *) &echoserver,
           sizeof(echoserver)) < 0) {
      Die("Failed to bind the server socket");
  }
  /* Listen on the server socket */
  if (listen(serversock, MAXPENDING) < 0) {
      Die("Failed to listen on server socket");
  }




#endif
//-----------------------------------------------------------

   int retval;
   char* log_name;
   int err_count_no_data = 0; //si no tengo datos nuevos varias veces es peligroso

   printf("%lf\n",M_PI);

   if(argc<3)
   {
	err_log("USAGE: ./auto_pilot log_name throttle_inicial");
	exit(1);
   }
   else
   {
	log_name = argv[1];
	throttle_inicial = atoi(argv[2]);
	printf("Throttle inicial: %u\n", throttle_inicial);
    }

    // Configurar pin de uart1 tx
#if !PC_TEST
   retval = system("echo 2 > /sys/kernel/debug/omap_mux/dss_data6");
   if (retval < 0)
   {
      err_log("system() failed - UART1 mux config");
      exit(0);
   }
#endif

   //setea senales y mascara
   set_signals();

   // Control de tiempos
   struct timeval tv_in_loop,
                  tv_start_main,
                  tv_diff,
                  dt;

   // -- -- -- -- -- -- -- -- --
   // Inicializacion
   // -- -- -- -- -- -- -- -- --

   /// Tiempo
   set_main_start_time();
   tv_start_main = get_main_start_time(); //uquad_aux_time.h


   /// Path planning & Following
   // init lista path
   lista_path = (Lista_path *)malloc(sizeof(struct ListaIdentificar_path));
   inicializacion_path(lista_path);

   // init lista wp
   lista_way_point = (Lista_wp *)malloc(sizeof(struct ListaIdentificar_wp));
   inicializacion_wp(lista_way_point);
   retval = way_points_input(lista_way_point); //carga waypoints en la lista desde un archivo de texto
   if (retval < 0) exit(0);

   // Generacion de trayectoria
   path_planning(lista_way_point, lista_path);

   log_trayectoria(lista_path); //dbg
   //visualizacion_path(lista_path); // dbg

#if SOCKET_TEST
  #define SOCKET_BUFF_SIZE	2
  double buffer[SOCKET_BUFF_SIZE] = {0, 0};
  int buffer_len = sizeof(buffer);
  unsigned int clientlen = sizeof(echoclient);
  
  printf("----------------------\n  Esperando Cliente  \n----------------------\n");
  /* Wait for client connection */
  if ((clientsock =
       accept(serversock, (struct sockaddr *) &echoclient,
       &clientlen)) < 0) {
          Die("Failed to accept client connection");
  }
  fprintf(stdout, "Client connected: %s\n",
                              inet_ntoa(echoclient.sin_addr));  
#endif

   /// Control yaw
   control_yaw_init_error_buff();

   /// Control velocidad
   // TODO

   /// Control altura
   // TODO

   /// Log
   log_fd = open(log_name, O_RDWR | O_CREAT | O_NONBLOCK );
   if(log_fd < 0)
   {
      err_log_stderr("Failed to open log file!");
      quit(1);
   }
   bool log_writeOK;

   ///GPS config - Envia comandos al gps a traves del puerto serie - //
#if !SIMULATE_GPS
   retval = preconfigure_gps();
   if(retval < 0) 
   {
      err_log("Failed to preconfigure gps!");
      quit(0);  
   } 
   //sleep_ms(1000);  //TODO verificar si es necesario y cuanto
  
   /// Ejecuta GPS daemon - proceso independiente
   gpsd_child_pid = init_gps();
   if(gpsd_child_pid == -1)
   {
      err_log_stderr("Failed to init gps!");
      quit(0);
   }
#else
   // TODO init fake gps
#endif //!SIMULATE_GPS

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
   sleep_ms(500); //TODO verificar si es necesario y cuanto

   /// inicializa IO manager
   io = io_init();
   if(io==NULL)
   {
      quit_log_if(ERROR_FAIL,"io init failed!");
   }
   retval = io_add_dev(io,STDIN_FILENO);  // Se agrega stdin al io manager
   quit_log_if(retval, "Failed to add stdin to io list"); 


   /// inicializa UAVTalk
#if !DISABLE_UAVTALK
   fd_CC3D = uav_talk_init();
   bool CC3D_readOK;
#endif
   
   /// mensajitos al usuario...
#if PC_TEST
   err_log("WARNING: Comenzando en modo 'PC test' - Ver common/quadcop_config.h");
#endif

#if SIMULATE_GPS
   err_log("WARNING: GPS simulated");
#endif
#if DISABLE_UAVTALK
   err_log("WARNING: UAVTALK disabled!");
#endif

int8_t count_50 = 1; // controla tiempo de loop 100ms
act_last = act;

char buff_act[512]; //TODO determinar valor
char buf_pwm[512]; //TODO determinar valor
int buff_len;

// TODO Espero a tener comunicacion estable con cc3d
#if !DISABLE_UAVTALK  
   err_log("Clearing CC3D input buffer...");
   //while(read(fd_CC3D,tmp_buff,1) > 0);
   serial_flush(fd_CC3D);
#endif 

#if DISABLE_UAVTALK 
uavtalk_updated = true;
#else
uavtalk_updated = false;
#endif
gps_updated = true;

bool first_time = true;

   printf("----------------------\n  Entrando al loop  \n----------------------\n");
   // -- -- -- -- -- -- -- -- -- 
   // Loop
   // -- -- -- -- -- -- -- -- -- 
   for(;;)
   {
	//para tener tiempo de entrada en cada loop
	gettimeofday(&tv_in_loop,NULL); //para tener tiempo de entrada en cada loop
	
	//trayectoria finalizada
	if (control_status == FINISHED) quit(0);

	//TODO Mejorar control de errores
	if(err_count_no_data > 10)
	{
	   err_log("mas de 10 errores en recepcion de datos!");
	   // TODO que hago? me quedo en hovering hasta obtener datos?
	   err_count_no_data = 0; //por ahora...
	}

	/** loop 50 ms **/
#if !DISABLE_UAVTALK
	/// Leo datos de CC3D
	CC3D_readOK = check_read_locks(fd_CC3D);
	if (CC3D_readOK) {
	   retval = uavtalk_read(fd_CC3D, &act);
	   if (retval < 0)
	   {
		err_log("uavtalk_read failed");
		continue;
	   } else if (retval == 0) {
		err_log("objeto no era actitud");  
		continue;
	   }
	   uavtalk_updated = true; //readOK (retval > 0)
	} else {
	   err_log("UAVTalk: read NOT ok");
	   continue;
	   //quit(0);
	}
#else
	sleep_ms(15); //simulo demora en lectura TODO determinar cuanto
#endif

	++count_50; // ??

	/// Polling de dispositivos IO
	retval = io_poll(io);
	quit_log_if(retval,"io_poll() error");

	/// Check stdin
	retval = io_dev_ready(io,STDIN_FILENO,&read_ok,NULL);
	if(retval < 0)
	{
	   err_log("Failed to check stdin for input!");
	}
	if(read_ok)
	{
	   read_from_stdin();
	}

	/** loop 100 ms **/
	if(count_50 > 1)
	{ 

	   //sleep_ms(5); //era 105000 us ??

#if !SIMULATE_GPS
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
#endif //!SIMULATE_GPS

	   count_50 = 0;
	} // if(count_50 > 1)

#if SIMULATE_GPS
	// TODO Datos del gps simulado
	//gps_updated = true;
	/*
	 * La posicion entra la actual y sale la siguiente (simulada)
	 * act_last guarda el angulo medido en el loop anterior
	 * velocidad esta seteada por el usuario
	 */
	 if(control_status == STARTED && !first_time)
	   simulate_gps(&posicion, &velocidad, act_last.yaw);
#endif //SIMULATE_GPS
	   

	if(control_status == STARTED)
        {
	   if (first_time)	
		first_time = false;
	   // Calcula diferencia respecto a cero
#if !FAKE_YAW
	   act.yaw = act.yaw - yaw_zero;
#else
	   last_yaw_measured = last_yaw_measured - yaw_zero;
#endif
           /// Path Follower & yaw control
	   // si hay datos de gps y de yaw hago carrot chase //TODO cuando tenga gps esto tiene que cambiar
	   if(gps_updated && uavtalk_updated)
	   {
#if !SIMULATE_GPS
	     /* guardo en waypoint p los valores de x, y hallados mediante el gps.
	      * primero deberia hacer convert_gps2utm(&utm, gps) y restarle la utm inicial
	      */
	      convert_gps2waypoint(&wp, gps);
	      wp.angulo = act.yaw;
#else
              wp.x = posicion.x; //TODO definir si la variable posicion la voy a usar siempre o solo simulando gps
	      wp.y = posicion.y;
	      wp.z = posicion.z;
	#if FAKE_YAW
	      wp.angulo = last_yaw_measured;
	#else
	      wp.angulo = act.yaw; 
	#endif // FAKE_YAW
#endif //!SIMULATE_GPS
	      
	      //carrot chase
	      retval = path_following(wp, lista_path, &yaw_d);
	      if (retval == -1) {
		  control_status = FINISHED;
		  puts("trayectoria finalizada"); // dbg
	      }
	      
	      if (err_count_no_data > 0)
	         err_count_no_data--;

	      /// Control TODO mejorar esto
	      u = control_yaw_calc_error(yaw_d, act.yaw);  // TODO PASAR A RADIANES
	      //printf("senal de control: %lf\n", u); // dbg

	      //Convertir velocidad en comando
	      ch_buff[2] = (uint16_t) (u*25/11 + 1500);
	      //printf("comando a enviar: %u\n", ch_buff[2]); // dbg
         
      	      //gps_updated = false;
#if !DISABLE_UAVTALK
	      uavtalk_updated = false;
#endif
	   } else {
	      err_log("No tengo datos para seguimiento de trayectorias");
	      err_count_no_data++;
	   }
	} 

	// Envia actitud y throttle deseados a sbusd (a traves de mensajes de kernel)
	retval = uquad_kmsgq_send(kmsgq, buff_out, MSGSZ);
	if(retval != ERROR_OK)
	{
	   quit_log_if(ERROR_FAIL,"Failed to send message!");
	}

#if DEBUG 
	#if !DISABLE_UAVTALK       
	// checkeo de tiempos al muestrear - debuggin
	retval = uquad_timeval_substract(&dt, act.ts, act_last.ts);
	if(retval > 0) {
	   if(dt.tv_usec > 60000) {
		err_log("WARN: se perdieron muestras");
           }
	act_last = act;
	} else {
	   err_log("WARN: Absurd timing!");
	   //serial_flush(fd_CC3D);
	}
	#endif // !DISABLE_UAVTALK
#endif // DEBUG

#if SOCKET_TEST
       buffer[0] = posicion.x;
       buffer[1] = posicion.y;
       /* Send back received data */
       if (send(clientsock, &buffer, buffer_len, 0) != buffer_len)
           Die("Failed to send bytes to client");
#endif

	// Log - T_s_act T_us_act roll pitch yaw C_roll C_pitch C_yaw T_s_main T_us_main
	//Timestamp main
	uquad_timeval_substract(&tv_diff, tv_in_loop, tv_start_main);
	buff_len = uavtalk_to_str(buff_act, act);

	buff_len += sprintf(buf_pwm, "%lf %u %u %u %u %lu %lu %lf %lf %lf %lf %lf\n",
				yaw_rate,
				ch_buff[0],
				ch_buff[1],
				ch_buff[2],
				ch_buff[3],
				tv_diff.tv_sec,
				tv_diff.tv_usec,
				posicion.x,
				posicion.y,
				posicion.z,
				yaw_d,
				last_yaw_measured);

	strcat(buff_act, buf_pwm);
	log_writeOK = check_write_locks(log_fd);
	if (log_writeOK) {
	   retval = write(log_fd, buff_act, buff_len);
	   if(retval < 0)
		err_log_stderr("Failed to write to log file!");
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

   /// Log
   close(log_fd);
   
#if !SIMULATE_GPS
   if(Q != 2) {
      /// cerrar conexiones con GPSD y terminarlo
      retval = deinit_gps();
      if(retval != ERROR_OK)
         err_log("Could not close gps correctly!");
   }
#endif // !SIMULATE_GPS

#if !DISABLE_UAVTALK
   /// cerrar UAVTalk
   retval = uav_talk_deinit(fd_CC3D);
   if(retval != ERROR_OK)
      err_log("Could not close UAVTalk correctly!");
#endif // !DISABLE_UAVTALK
     
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
#if !SIMULATE_GPS
      } else if(p == gpsd_child_pid) {
         err_log_num("WARN: gpsd died! sig num:", signal_num);
         quit(0); //exit sin cerrar gpsd
#endif //!SIMULATE_GPS
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
         int retval = fread(tmp_buff,sizeof(unsigned char),2,stdin); //TODO corregir que queda algo por leer en el buffer?
         if(retval <= 0)
         {
	    //log_n_jump(ERROR_READ, end_stdin,"No user input detected!");
            err_log_num("No user input detected!",ERROR_READ);
            return;
         }
         
	 //retval = 0;
         switch(tmp_buff[0])
         {
         case 'S':
            ch_buff[3] = throttle_inicial; //valor pasado como parametro
#if !SIMULATE_GPS
	    velocidad.module = 4; //TODO esto??
#endif //!SIMULATE_GPS
            puts("Comenzando lazo cerrado");
            control_status = STARTED;
            break;
         case 'P':
            ch_buff[3] = 1000;
            puts("Deteniendo");
            control_status = STOPPED;
            break;
         case 'F':
            puts("WARN: Failsafe set");
            ch_buff[5] = 50;
            break;
         case 'f':
            puts("WARN: Failsafe clear");
            ch_buff[5] = 100;
            break;
         case 'b':
            ch_buff[0] = 1500;
            ch_buff[1] = 1500;
            ch_buff[2] = 1500;
            ch_buff[3] = 1000;  // neutral throttle
            ch_buff[4] = 1500;
            puts("Seteando valor neutro");
            break;
         case 'A':
#if FAKE_YAW
	    yaw_zero = last_yaw_measured;
#else
	    yaw_zero = act.yaw;
#endif
            ch_buff[0] = 1500; //roll
            ch_buff[1] = 1500; //pitch
            ch_buff[2] = 1000; //yaw
            ch_buff[3] = 950;  //throttle  
            puts("Armando..."); 
            break;
         case 'D':
            ch_buff[0] = 1500; //roll
            ch_buff[1] = 1500; //pitch
            ch_buff[2] = 2000; //yaw
            ch_buff[3] = 950;  //throttle
            puts("Desarmando...");
            break;
#ifdef SETANDO_CC3D
	// Para setear maximos y minimos en CC3D
         case 'M':
            ch_buff[0] = 2000; //roll
            ch_buff[1] = 2000; //pitch
            ch_buff[2] = 2000; //yaw
            ch_buff[3] = 2000; //throttle
            ch_buff[4] = 2000; //flight mode
            puts("Seteando maximo valor"); 
            break;
         case 'm':
            ch_buff[0] = 1000;
            ch_buff[1] = 1000;
            ch_buff[2] = 1000;
            ch_buff[3] = 950; //min throttle
            ch_buff[4] = 1000;
            puts("Seteando minimo valor");
            break;
#endif
         default:
            puts("comando invalido");
            //retval = -1;
            break;
         } //switch(tmp_buff[0])

         return;
}

#define VELOCITY		3.0 // m/s
// SIMULACION GPS TODO SACAR DE ACA
void simulate_gps(posicion_t* pos, velocidad_t* vel, double yaw_measured)
{
#if FAKE_YAW
   /*********   FAKE YAW   **********/
   //el yaw_d se actualiza despues de sumulate_gps, por lo tanto este yaw_d es el del loop anterior
   yaw_measured = last_yaw_measured + 0.06*(yaw_d - last_yaw_measured);
   last_yaw_measured = yaw_measured + yaw_zero;
   /*********************************/
#endif

   double F_x = masa*g*sin(pitch)*cos(yaw_measured); // proyeccion en x fuerza motores
   double F_y = masa*g*sin(pitch)*sin(yaw_measured); // proyeccion en y fuerza motores

   int i = 0;
   for(i=0;i<5;i++) {
  
	// Fuerza de rozamiento
	double r_x = -B*vel->x;
	double r_y = -B*vel->y;

	// Aceleracion
	double a_x = (r_x + F_x)/masa;
	double a_y = (r_y + F_y)/masa;

	// Velocidad
	vel->x = vel->x + a_x*YAW_SAMPLE_TIME/5;
	vel->y = vel->y + a_y*YAW_SAMPLE_TIME/5;

	// Posicion
	pos->x = pos->x + vel->x*YAW_SAMPLE_TIME/5;
	pos->y = pos->y + vel->y*YAW_SAMPLE_TIME/5;
   }


/*   //vel->x = vel->x + YAW_SAMPLE_TIME*B*(pow(VELOCITY,2)-pow(vel->x,2)-tan(yaw_measured)*vel->x*sqrt(pow(VELOCITY,2)-pow(vel->x,2)))/(tan(yaw_measured)*sqrt(pow(VELOCITY,2)-pow(vel->x,2))+vel->x)/masa;
   vel->x = vel->x + YAW_SAMPLE_TIME*B/masa*sqrt(pow(VELOCITY,2)-pow(vel->x,2))*(sqrt(pow(VELOCITY,2)-pow(vel->x,2))-vel->x*tan(yaw_measured))/(tan(yaw_measured)*sqrt(pow(VELOCITY,2)-pow(vel->x,2))+vel->x);
   vel->y = sqrt(pow(VELOCITY,2) - pow(vel->x,2));

	printf("velocidad x:  %lf  ",vel->x);
	printf("velocidad y:  %lf  ",vel->y);
	printf("Modulo:  %lf\n", sqrt( pow(vel->x,2) + pow(vel->y,2) ));

   pos->x = pos->x + vel->x*YAW_SAMPLE_TIME;
   pos->y = pos->y + vel->y*YAW_SAMPLE_TIME;
*/
   return;
}





