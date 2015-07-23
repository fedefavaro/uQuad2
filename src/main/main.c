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

#include <quadcop_types.h>
#include <uquad_kernel_msgq.h>
#include <uquad_error_codes.h>
#include <quadcop_config.h>
#include <uquad_aux_time.h>
#include <uquad_aux_io.h>
#include <socket_comm.h>
//#include <path_planning.h>
//#include <path_following.h>
#include <futaba_sbus.h>
#include <serial_comm.h>
#include <gps_comm.h>
#include <uavtalk_parser.h>
#include <imu_comm.h>
#include <control_yaw.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>

// shm - shared memory.
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define CH_COUNT		6	//cantidad de canales de comunicacion con sbusd
#define BUFF_SIZE		12	//tamano del buffer de comunicacion (tamano de cada canal = BUFF_SIZE/CH_COUNT)

#define KILL_SBUS		"killall sbusd"

#define SHMSZ     27
#include <semaphore.h>

//#define SETANDO_CC3D

/*********************************************
 ************* Global vars *******************
 *********************************************/
//Semaphore
sem_t * sem_id;

// Almacena pids de hijos
pid_t sbusd_child_pid = 0;
pid_t gpsd_child_pid = 0;
pid_t uavtalk_child_pid = 0;

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

// STDIN
bool stdin_readOK = false;		//puedo leer de la entrada estandar sin bloquear
unsigned char tmp_buff[2] = {0,0};	//almacena commnado enviado por el usuario

// KMQ
uquad_kmsgq_t *kmsgq 	= NULL;

/** 
 * Valores iniciales de los canales a enviar al proceso sbusd
 *
 * ch_buff[ROLL_CH_INDEX]	roll
 * ch_buff[PITCH_CH_INDEX]	pitch
 * ch_buff[YAW_CH_INDEX]	yaw
 * ch_buff[THROTTLE_CH_INDEX]	throttle
 * ch_buff[FLIGHTMODE_CH_INDEX]	flight mode
 * ch_buff[FAILSAFE_CH_INDEX]	activar/desactivar failsafe
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
#if !FAKE_YAW
actitud_t act = {0,0,0,{0,0}}; 			//almacena variables de actitud leidas de la cc3d y timestamp
#else
actitud_t act = {0,0,INITIAL_YAW,{0,0}};	//si estoy en modo fake yaw inicializo el valor de yaw
#endif
bool uavtalk_updated = false;

// IMU
imu_raw_t imu_raw; 		//datos crudos
imu_data_t imu_data; 		//datos utiles
int fd_IMU; 			//file descriptor de la IMU
bool IMU_readOK = false; 	//puedo leer sin bloquear
bool imu_updated = false; 	//existen datos nuevos de la IMU
bool baro_calibrated = false;	//barometro calibrado
double po = 0; 			//variable para relevar presion ambiente (calib del baro)
int baro_calib_cont = 0;	//contador para determinar cuantas muestras tomar para la calib del baro

// Control de yaw
double u_yaw = 0; //senal de control (setpoint de velocidad angular)
double yaw_d = 0;

// Control activado/desactivado // TODO Mover
typedef enum {
	STOPPED = 0,
	STARTED,
        FINISHED
} estado_control_t;
estado_control_t control_status = STOPPED;

position_t position = {0,0,0,{0,0}};

#if !SIMULATE_GPS
velocity_t velocity = {0,0,{0,0}};
#else
velocity_t velocity = {0,0,0,{0,0}};
#endif

double pitch = 0; // angulo de pitch en radianes

/// Declaracion de funciones auxiliares
void quit(int Q);
void uquad_sig_handler(int signal_num);
void set_signals(void);
void read_from_stdin(void);

uint16_t convert_yaw2pwm(double yaw); // Convierte angulo de yaw a senal de pwm para enviar a la cc3d // TODO no implementado

/*********************************************/
/**************** Main ***********************/
/*********************************************/
int main(int argc, char *argv[])
{
   /// mensajes al usuario
#if PC_TEST
   printf("----------------------\n  WARN: Modo 'PC test' - Ver common/quadcop_config.h  \n----------------------\n");
   sleep_ms(500);
#endif
#if SIMULATE_GPS
   printf("----------------------\n  WARN: GPS simulado - Ver common/quadcop_config.h  \n----------------------\n");
   sleep_ms(500);
#endif
#if DISABLE_UAVTALK
   printf("----------------------\n  WARN: UAVTalk desabilitado - Ver common/quadcop_config.h  \n----------------------\n");
   sleep_ms(500);
#endif

   /// Init socket comm
#if SOCKET_TEST
   socket_comm_init();
#endif

   int retval;
   char* log_name;
   int err_act = 0;		//Aumenta si no hay datos nuevos de actitud
   int err_count_no_data = 0; 	//si no tengo datos nuevos varias veces es peligroso
   int no_gps_data = 0;

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
		  tv_out_loop, tv_out_last_loop,
                  tv_start_main,
                  tv_diff;
#if DEBUG
   struct timeval dt;
#endif

   // -- -- -- -- -- -- -- -- --
   // Inicializacion
   // -- -- -- -- -- -- -- -- --

   /// Tiempo
   set_main_start_time();
   tv_start_main = get_main_start_time(); //uquad_aux_time.h

#if SOCKET_TEST
   if (socket_comm_wait_client() == -1)
	quit(1);
#endif

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

   log_trayectoria(lista_path);    //dbg
   //visualizacion_path(lista_path); // dbg

#if SOCKET_TEST
   if (socket_comm_send_path(lista_path) == -1)
	quit(1);
#endif

   /// Control yaw
#if CONTROL_YAW_ADD_DERIVATIVE
   control_yaw_init_error_buff();
#endif

   /// Control velocidad
   // TODO

   // Se calcula el valor de pitch necesario para alcanzar la velocidad deseada.
   // El signo negativo es para que sea coherente con el sentido de giro de la cc3d (angulo positivo = giro horario)
   pitch = -atan(B_roz*VEL_DESIRED/MASA/G);
   //printf("pitch: %lf\n", pitch*180/M_PI); //dbg

   /// Control altura
   control_alt_init_error_buff();

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
     
   /// Ejecuta GPS daemon - proceso independiente
   gpsd_child_pid = init_gps();
   if(gpsd_child_pid == -1)
   {
      err_log_stderr("Failed to init gps!");
      quit(0);
   }
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

   //Doy tiempo a que inicien bien los procesos secundarios
   sleep_ms(500); //TODO verificar cuanto es necesario

/*   /// inicializa IO manager
   io = io_init();
   if(io==NULL)
   {
      quit_log_if(ERROR_FAIL,"io init failed!");
   }
   retval = io_add_dev(io,STDIN_FILENO);  // Se agrega stdin al io manager
   quit_log_if(retval, "Failed to add stdin to io list"); */


   /// inicializa UAVTalk
#if !DISABLE_UAVTALK
   uavtalk_child_pid = uavtalk_parser_start(tv_start_main);
   if(uavtalk_child_pid == -1)
   {
      err_log_stderr("Failed to start child process (uavtalk)!"); 
      quit(1);  
   }
#endif

   /// init IMU
#if !DISABLE_IMU
   fd_IMU = imu_comm_init(IMU_DEVICE);
   if(fd_IMU < 0) 
   {
      err_log("Failed to init IMU!");
      quit(0);  
   } 
   imu_data_alloc(&imu_data);
   magn_calib_init(); //Carga parametros de calibracion del magn, baro se calibra en el loop
#endif
   
   int8_t count_50 = 1; // controla tiempo de loop 100ms
   //act_last = act;

   char buff_log[512]; //TODO determinar valor
   char buff_log_aux[512]; //TODO determinar valor
   int buff_log_len;

#if !DISABLE_UAVTALK  
   //err_log("Clearing CC3D input buffer...");
   //serial_flush(fd_CC3D);
#endif 

#if !DISABLE_IMU
   // Clean IMU serial buffer
   err_log("Clearing IMU input buffer...");
   serial_flush(fd_IMU);
#endif

   int bytes_avail = 0;
   bool first_time = true;

#if !DISABLE_UAVTALK 
//--------------------------------------------------------------
    typedef struct act_sdata {
	actitud_t act;
	int flag;
    } act_sdata_t;


    int shmid;
    key_t key = 5678;
    act_sdata_t *shm;

   /**
    * Semaphore open
    */
    sem_id=sem_open("/mysem", O_CREAT, S_IRUSR | S_IWUSR, 1);

    /*
     * Locate the segment.
     */
    if ((shmid = shmget(key, SHMSZ, 0666)) < 0) {
        perror("shmget");
        exit(1);
    }

    /*
     * Now we attach the segment to our data space.
     */
    if ((shm = shmat(shmid, NULL, 0)) == (void *) -1) {
        perror("shmat");
        exit(1);
    }
//--------------------------------------------------------------
#endif //!DISABLE_UAVTALK 

   printf("----------------------\n  Entrando al loop  \n----------------------\n");
   // -- -- -- -- -- -- -- -- -- 
   // Loop
   // -- -- -- -- -- -- -- -- -- 
   for(;;)
   {
	//para tener tiempo de entrada en cada loop
	gettimeofday(&tv_in_loop,NULL);
	
	//TODO Mejorar control de errores
	if(err_count_no_data > 10)
	{
	   err_log("mas de 10 errores en recepcion de datos!");
	   // TODO que hago? me quedo en hovering hasta obtener datos?
	   err_count_no_data = 0; //por ahora...
	}

#if !DISABLE_IMU
	/// Lectura de datos de la IMU
//--------------------------------------------------------------------------------------------------------
	// Vuelvo a leer si tengo una trama de datos completa luego de finalizada la lectura
	leer_IMU_denuevo:

	// Reviso si tengo una trama de datos completa antes de leer
	ioctl(fd_IMU, FIONREAD, &bytes_avail);
	if(bytes_avail < 34) {
		//puts("datos no completos IMU!"); //dbg
		sleep_ms(5);
		goto leer_IMU_denuevo;
	}


	IMU_readOK = check_read_locks(fd_IMU);
	if (IMU_readOK) {

            // Leo datos
	    retval = imu_comm_read(fd_IMU);            
 	    if (retval < 0 ) {
		puts("unable to read IMU data");
	    }
            // Paso los datos del buffer RX a imu_raw.
            imu_comm_parse_frame_binary(&imu_raw);
	    //print_imu_raw(&imu_raw); // dbg

	    // Si no estoy calibrando convierto datos para usarlos
	    if (baro_calibrated) {
		imu_raw2data(&imu_raw, &imu_data);
		//print_imu_data(&imu_data); //dbg
	    }

	    imu_updated = true;
            IMU_readOK = false;

        } else {
	   err_log("imu: read NOT ok");
	}

/*	/// Reviso si quedan datos para no atrasarme
	IMU_readOK = check_read_locks(fd_IMU);
	if (IMU_readOK) {
		printf("todavia quedan datos IMU!\n");
		IMU_readOK = false;
		ioctl(fd_IMU, FIONREAD, &bytes_avail);
		//printf("%d\n ",bytes_avail);
		if(bytes_avail > 33) goto leer_denuevo; //continue;
	}*/
	// Reviso si tengo una trama de datos completa atrasada
	ioctl(fd_IMU, FIONREAD, &bytes_avail);
	if(bytes_avail > 33) {
		//puts("todavia quedan datos IMU!"); //dbg
		goto leer_IMU_denuevo;
	}

	if (!baro_calibrated) {
		if(imu_updated) {		
			po += imu_raw.pres;
			baro_calib_cont++;
			if (baro_calib_cont == BARO_CALIB_SAMPLES) {
			   pres_calib_init(po/BARO_CALIB_SAMPLES);
			   baro_calibrated = true;
			   puts("Barometro calibrado!");
		}	}
		goto end_loop; //si estoy calibrando no hago nada mas!
	}
//--------------------------------------------------------------------------------------------------------
#endif


	/** loop 50 ms **/
#if !DISABLE_UAVTALK
	/// Leo datos de CC3D
//--------------------------------------------------------------------------------------------------------
	leer_cc3d:
	// control de errores - si intento leer 3 veces (15ms) y no hay datos nuevos, cierro
	if(err_act > 3) {
		err_log("No hay datos nuevos de actitud, cerrando.");
		quit(1);
	} 
	
	sem_wait(sem_id); //activo semaforo para evitar que el parser modifique los datos mientras los leo
	// El parser sobreescribio un dato - //dbg
	if(shm->flag == 2)
	   //puts("perdi un dato actitud!"); //dbg

	// si llego al dato antes de que este pronto espero un poco y vuelvo a consultar	
	if(shm->flag == 0) { 
	   //puts("llegue muy temprano!"); //dbg
	   sem_post(sem_id); //desactivo semaforo - no pude leer dato
	   sleep_ms(4); //doy tiempo a que este pronto el dato //TODO determinar cuanto
	   err_act++;
	   goto leer_cc3d;
	}
	act = shm->act; // copio de la memoria compartida a la memoria del main
	shm->flag = 0;  // marco el flag acorde (dato leido)
	sem_post(sem_id); //desactivo semaforo - (dato leido)
	err_act = 0;
	//uav_talk_print_attitude(act); //dbg
//--------------------------------------------------------------------------------------------------------	
#else
   #if FAKE_YAW
	if(control_status == STARTED && !first_time)
	   act.yaw = simulate_yaw(yaw_d);  //yaw_d es el del loop anterior
	uavtalk_updated = true;
   #endif	
#endif

#if !FAKE_YAW
	// Calcula diferencia respecto a cero
	act.yaw = act.yaw - get_yaw_zero();
#endif

	++count_50; // control de loop 100ms

	/// Check stdin
	stdin_readOK = check_read_locks(STDIN_FILENO);
	if (stdin_readOK) {
	   read_from_stdin();
	}

	/** loop 100 ms **/
	if(count_50 > 1)
	{ 
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
		gps_updated = true;
	   }
#else //GPS SIMULADO
	   if(control_status == STARTED && !first_time)
	      gps_simulate_position(&position, &velocity, act.yaw, pitch);
	   gps_updated = true;
#endif //!SIMULATE_GPS

	   count_50 = 0;
	} /** end loop 100 ms **/


	if(control_status == STARTED)
        {
	   if (first_time)	
		first_time = false;

           /// Con datos de actitud hago control de yaw y path following(si tengo gps)
	   if(uavtalk_updated)
	   {
	      /// Path Follower - si hay datos de gps y de yaw hago carrot chase	      
	      if(gps_updated) {
#if !SIMULATE_GPS
	        /* guardo en waypoint p los valores de (x,y) hallados mediante el gps.
	         * primero deberia hacer convert_gps2utm(&utm, gps) y restarle la utm inicial
	         */
	         //convert_gps2waypoint(&wp, gps); // TODO no implementado!!
	         wp.angulo = act.yaw;
#else
                 wp.x = position.x;
	         wp.y = position.y;
	         wp.z = position.z;
	         wp.angulo = act.yaw;
#endif //!SIMULATE_GPS
	      
	         //carrot chase
	         retval = path_following(wp, lista_path, &yaw_d);
	         if (retval == -1) {
		     control_status = FINISHED;
		     puts("¡¡ Trayectoria finalizada !!");
		     ch_buff[THROTTLE_CH_INDEX] = THROTTLE_NEUTRAL; // detengo los motores
	         }
		 
		 no_gps_data = 0;

	      } else {
		 no_gps_data++;
		 if (no_gps_data > 1) { // Debo tener datos del gps cada 2 loops de 50 ms
		    err_count_no_data++;
		    no_gps_data = 0;
		    err_log("No tengo datos de gps para seguimiento de trayectorias");
		 }
	      }

	   /// Control Yaw - necesito solo medida de yaw
	   u_yaw = control_yaw_calc_input(yaw_d, act.yaw);  // TODO PASAR A RADIANES
	   //printf("senal de control: %lf\n", u); // dbg

	   //Convertir velocidad en comando
	   ch_buff[YAW_CH_INDEX] = (uint16_t) (u_yaw*25/11 + 1500);
	   //printf("  %u\n", ch_buff[YAW_CH_INDEX]); // dbg

           if (err_count_no_data > 0)
	         err_count_no_data--;

	   } else {
	      err_log("No tengo datos para control de yaw");
	      err_count_no_data++;
	   }
	
	
	   /// Control de Velocidad
	   if(gps_updated) {
	      //TODO...

	   }

#if !DISABLE_IMU
	   /// Control de Altura
	   if(imu_updated) {
	      //TODO...

	   }
#endif

	   // Luego de finalizados los controles reseteo flags
	   uavtalk_updated = false;
	   gps_updated = false;
	   imu_updated = false;
	
	} // end if(control_started)


	// Envia actitud y throttle deseados a sbusd (a traves de mensajes de kernel)
	retval = uquad_kmsgq_send(kmsgq, buff_out, MSGSZ);
	if(retval != ERROR_OK)
	{
	   quit_log_if(ERROR_FAIL,"Failed to send message!");
	}

#if SOCKET_TEST
       if (socket_comm_update_position(position) == -1)
	  quit(1);
#endif

	/** Log - T_s_act/T_us_act/roll/pitch/yaw/C_roll/C_pitch/C_yaw/C_throt/T_s_main/T_us_main/pos.x/pos.y/pos.z/yaw_d **/
	
	//Timestamp main
	uquad_timeval_substract(&tv_diff, tv_in_loop, tv_start_main);

	// Duracion del main
	gettimeofday(&tv_out_loop,NULL);
	uquad_timeval_substract(&dt, tv_out_loop, tv_out_last_loop);

	//datos de CC3D para log
	buff_log_len = uavtalk_to_str(buff_log, act);

	//otros logs
	buff_log_len += sprintf(buff_log_aux, "%u %u %u %u %lu %lu %lf %lf %lf %lf %lf",
				ch_buff[ROLL_CH_INDEX],
				ch_buff[PITCH_CH_INDEX],
				ch_buff[YAW_CH_INDEX],
				ch_buff[THROTTLE_CH_INDEX],
				tv_diff.tv_sec,
				tv_diff.tv_usec,
				position.x,
				position.y,
				position.z,
				yaw_d,
				u_yaw);

	strcat(buff_log, buff_log_aux);
	
	//datos de IMU para log
	buff_log_len += imu_to_str(buff_log_aux, imu_data);

	strcat(buff_log, buff_log_aux);

	log_writeOK = check_write_locks(log_fd);
	if (log_writeOK) {
	   retval = write(log_fd, buff_log, buff_log_len);
	   if(retval < 0)
		err_log_stderr("Failed to write to log file!");
	}

	end_loop:

/*#if DEBUG 
	// checkeo de tiempos del main
	gettimeofday(&tv_out_loop,NULL);
	retval = uquad_timeval_substract(&dt, tv_out_loop, tv_in_loop);
	if(retval > 0) {
	   if(dt.tv_usec > 50000) {
		err_log("WARN: se atraso main");
           }
	} else {
	   err_log("WARN: Main absurd timing!");
	}
#endif // DEBUG*/

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
 * Q == 3 : interrupcion por muerte del parser, cierra todo menos parser
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
   
/*   /// cerrar IO manager
   retval = io_deinit(io);
   if(retval != ERROR_OK)
      err_log("Could not close IO correctly!");*/

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
   if(Q != 3) {
      /// cerrar UAVTalk
      retval = kill(uavtalk_child_pid, SIGKILL);
      if(retval != ERROR_OK)
         err_log("Could not close Parser correctly!");
   }
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
#if !DISABLE_UAVTALK
      } else if(p == uavtalk_child_pid) {
         err_log_num("WARN: uavtalk_parser died! sig num:", signal_num);
         quit(3); //exit sin cerrar gpsd
#endif //!DISABLE_UAVTALK
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
         if(retval < 0)
         {
	    //log_n_jump(ERROR_READ, end_stdin,"No user input detected!");
            err_log_num("No user input detected!",ERROR_READ);
            return;
         }
         
	 //retval = 0;
         switch(tmp_buff[0])
         {
         case 'S':
            ch_buff[THROTTLE_CH_INDEX] = throttle_inicial; //valor pasado como parametro
#if !SIMULATE_GPS
	    velocity.module = 4; //TODO esto??
#endif //!SIMULATE_GPS
            puts("Comenzando lazo cerrado");
            control_status = STARTED;
            break;
         case 'P':
            ch_buff[THROTTLE_CH_INDEX] = THROTTLE_NEUTRAL;
            puts("Deteniendo");
            control_status = STOPPED;
            break;
         case 'F':
            puts("WARN: Failsafe set");
            ch_buff[FAILSAFE_CH_INDEX] = ACTIVATE_FAILSAFE;
            break;
         case 'f':
            puts("WARN: Failsafe clear");
            ch_buff[FAILSAFE_CH_INDEX] = DEACTIVATE_FAILSAFE;
            break;
         case 'b':
            ch_buff[ROLL_CH_INDEX] = ROLL_NEUTRAL;
            ch_buff[PITCH_CH_INDEX] = PITCH_NEUTRAL;
            ch_buff[YAW_CH_INDEX] = YAW_NEUTRAL;
            ch_buff[THROTTLE_CH_INDEX] = THROTTLE_NEUTRAL;
            ch_buff[FLIGHTMODE_CH_INDEX] = FLIGHT_MODE_2;  //neutral value
            puts("Seteando valor neutro");
            break;
         case 'A':
#if !FAKE_YAW
	    //yaw_zero = act.yaw;
	    set_yaw_zero(act.yaw);
#endif
            ch_buff[ROLL_CH_INDEX] = ROLL_NEUTRAL;
            ch_buff[PITCH_CH_INDEX] = PITCH_NEUTRAL;
            ch_buff[YAW_CH_INDEX] = YAW_ARM;
            ch_buff[THROTTLE_CH_INDEX] = THROTTLE_ARM; 
            puts("Armando...");
            break;
         case 'D':
            ch_buff[ROLL_CH_INDEX] = ROLL_NEUTRAL;
            ch_buff[PITCH_CH_INDEX] = PITCH_NEUTRAL;
            ch_buff[YAW_CH_INDEX] = YAW_DISARM;
            ch_buff[THROTTLE_CH_INDEX] = THROTTLE_DISARM;
            puts("Desarmando...");
            break;
#ifdef SETANDO_CC3D
	// Para setear maximos y minimos en CC3D
         case 'M':
            ch_buff[ROLL_CH_INDEX] = MAX_COMMAND;
            ch_buff[PITCH_CH_INDEX] = MAX_COMMAND;
            ch_buff[YAW_CH_INDEX] = MAX_COMMAND;
            ch_buff[THROTTLE_CH_INDEX] = MAX_COMMAND;
            ch_buff[FLIGHTMODE_CH_INDEX] = MAX_COMMAND;
            puts("Seteando maximo valor"); 
            break;
         case 'm':
            ch_buff[ROLL_CH_INDEX] = MIN_COMMAND;
            ch_buff[PITCH_CH_INDEX] = MIN_COMMAND;
            ch_buff[YAW_CH_INDEX] = MIN_COMMAND;
            ch_buff[THROTTLE_CH_INDEX] = MIN_THROTTLE; // Se necesecita que sea distino por el armado/desarmado
            ch_buff[FLIGHTMODE_CH_INDEX] = MIN_COMMAND;
            puts("Seteando minimo valor");
            break;
#endif
         default:
#if DEBUG
            puts("comando invalido");
#endif
            break;
         } //switch(tmp_buff[0])

	 fflush( stdin );

         return;
}



