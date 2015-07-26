

#include "imu_comm.h"
#include <math.h>
#include <uquad_error_codes.h>
#include <serial_comm.h>

#include <quadcop_types.h>

uint16_t tiempo;
int16_t accx;
int16_t accy;
int16_t accz;
int16_t gyrox;
int16_t gyroy;
int16_t gyroz;
int16_t magnx;
int16_t magny;
int16_t magnz;
uint16_t temp;
uint32_t pres;
int16_t us_obstacle;
        
// Matrices calibracion IMU
// Magn
//static uquad_mat_t *magn_K;
//static uquad_mat_t *magn_b;
// Baro
//static double K;
//static double *pres_K = &K;
static double po = 0;
//static double *pres_po = &po;
//static double expo;
//static double *pres_exponente = & expo;   
    
// Buffer de recepcion.
static unsigned char RX_imu_buffer[RX_IMU_BUFFER_SIZE];

//*****************************************************************************
//
// Inicializacion imu
//
//*****************************************************************************
int imu_comm_init(char *device)
{
   int retval; 
   int fd = open_port(device);
   if (fd < 0) {
      err_log("No se pudo abrir el puerto de la IMU");
      return -1;
   }
   printf("IMU conectada - fd: %d\n",fd); //dbg

   retval = configure_port_gps(fd, B115200); //TODO cambiar nombre
   if (retval < 0) {
      puts("No se pudo configurar el puerto de la IMU");
      return -1;
   }

   // Change IMU to binary mode.
   retval = write(fd,"!",1);
   if (retval < 1) {
      puts("No se pudo pasar la IMU a binario");
      return -1;
   }

   return fd;

}


//*****************************************************************************
//
// Inicializacion imu_data
//
//*****************************************************************************
/*void imu_data_alloc(imu_data_t *imu_data)
{
   // imu_data->magn = uquad_mat_alloc(3,1);
    // initialize data to zeros
    //uquad_mat_zeros(imu_data->magn);
}*/


//*****************************************************************************
//
// Leo datos de la imu
//
//*****************************************************************************
int imu_comm_read(int fd)
{
   int ret;
   int imu_data_ready = 0;
   uint8_t c;
   int index = 0;
   bool in_sync = false;

   while (!imu_data_ready)// && check_read_locks(fd))
   {				
	ret = read(fd,&c,1);
	if (ret <= 0) {
	   puts("read failed");
	   return -1;
	}

	// Inicio del mensaje..
        if ( (in_sync == false) && ((c == 'A') || (c == 'C')) ) {
            index = 0;
	    RX_imu_buffer[index] = c;
	    in_sync = true;
        } else if ((index < (RX_IMU_BUFFER_SIZE-2)) && (in_sync == true)) {
	       RX_imu_buffer[++index] = c;
	} else if ( (c == 'Z') && (in_sync == true) ) {
               RX_imu_buffer[++index] = c;
	       imu_data_ready = 1;
	} else return -1;
            
   } // while...
	
   return imu_data_ready;
}

//*****************************************************************************
//
// Separa los datos del buffer RX y guarda la info en imu_raw
//
//*****************************************************************************
void imu_comm_parse_frame_binary(imu_raw_t *frame)//, unsigned char *data)
{
    unsigned char *data;
    data = RX_imu_buffer+1;
    int i = 0;
    int16_t* buffParse_16;
    memcpy(&frame->T_us, data, IMU_BYTES_T_US);
    data += IMU_BYTES_T_US;
    buffParse_16 = (int16_t *)data;
    //acc
    for(;i<3;++i)
        frame->acc[i%3] = buffParse_16[i];
    //gyro
    for(;i<6;++i)
        frame->gyro[i%3] = buffParse_16[i]; // 4 mod 3 = 1. Then gyro[4%3] = gyro[1].
    //magn
    for(;i<9;++i)
        frame->magn[i%3] = buffParse_16[i];
    //temp
    frame->temp = (uint16_t)buffParse_16[i++];
    //press
    frame->pres = *((uint32_t*)(buffParse_16 + i));
    //us - obstaculo
    i = i+2;
    frame->us_obstacle = buffParse_16[i];
    //us - altura
    frame->us_altitude = buffParse_16[++i];
}



//*****************************************************************************
//
// Imprime los datos de imu_raw
//
//*****************************************************************************
void print_imu_raw(imu_raw_t *frame)
{
    printf("%c", RX_imu_buffer[0]);
    printf("  %lu", frame->T_us);
    printf("  %i", frame->acc[0]);
    printf("  %i", frame->acc[1]);
    printf("  %i", frame->acc[2]);
    printf("  %i", frame->gyro[0]);
    printf("  %i", frame->gyro[1]);
    printf("  %i", frame->gyro[2]);
    printf("  %i", frame->magn[0]);
    printf("  %i", frame->magn[1]);
    printf("  %i", frame->magn[2]);
    printf("  %i", frame->temp);
    printf("  %lu", frame->pres);
    printf("  %i", frame->us_obstacle);
    printf("  %i", frame->us_altitude);
    printf("  %c\n", RX_imu_buffer[RX_IMU_BUFFER_SIZE-1]);
}

//*****************************************************************************
//
// Imprime los datos de imu_data
//
//*****************************************************************************
void print_imu_data(imu_data_t *data)
{
    printf("%lf", data->T_us);
    //printf("\t%lf", data->magn->m_full[0]);
    //printf("\t%lf", data->magn->m_full[1]);
    //printf("\t%lf", data->magn->m_full[2]);
    printf("\t%lf", data->alt);
    printf("\t%lf", data->us_obstacle);
    printf("\t%lf\n", data->us_altitude);
}


//*****************************************************************************
//
// Calibraciones IMU
//
//*****************************************************************************

#if 0
void magn_calib_init(void)
{
    // K
    magn_K = uquad_mat_alloc(3, 3);
    magn_K->m_full[0] = 0.00402824066832922;
    magn_K->m_full[1] = -8.96774717665988e-06;
    magn_K->m_full[2] = 0.000363980178696652;
    magn_K->m_full[3] = 0.0;
    magn_K->m_full[4] = 0.00405222522881617;
    magn_K->m_full[5] = -0.000155928970260749;
    magn_K->m_full[6] = 0.0;
    magn_K->m_full[7] = 0.0;
    magn_K->m_full[8] = 0.00460429864076721;
	
    // b
    magn_b = uquad_mat_alloc(3, 1);
    magn_b->m_full[0] = -63.9019715992965;
    magn_b->m_full[1] = -38.911556235825;
    magn_b->m_full[2] = -75.7517190381074;
}
#endif

/*
 * Recibe como parametro un promedio de la presion ambiente
 * en el momento de encendido
 */
#define PRESS_EXP                  0.191387559808612
#define PRESS_K                    44330.0
void pres_calib_init(double po_mean)
{
    // cargo presion ambiente medida
    po = po_mean;

}


//*****************************************************************************
//
// Conversion datos crudos a datos utiles.
//
//*****************************************************************************

#if 0
void temp_raw2data(imu_raw_t *raw, imu_data_t *data)
{
	data->temp = ((double)(raw->temp))/10;
}


void magn_raw2data(imu_raw_t *raw, imu_data_t *data)
{
	// Modelo: C = K(C_raw - b)
  
	// Paso los datos crudos del Magn a una matriz magn_raw
	uquad_mat_t *magn_raw = uquad_mat_alloc(3, 1);
	magn_raw->m_full[0] = raw->magn[0];
	magn_raw->m_full[1] = raw->magn[1];
	magn_raw->m_full[2] = raw->magn[2];
	
	// Resta: C_raw - b
	uquad_mat_t *C_b = uquad_mat_alloc(3, 1);
	uquad_mat_sub(C_b, magn_raw, magn_b);
	
	// Multiplicacion: K * (C_raw - b)
	uquad_mat_t *magn_data = uquad_mat_alloc(3, 1);
	uquad_mat_prod(magn_data, magn_K, C_b);
	
	// Copio resultado en matriz de salida
	data->magn->m_full[0] = magn_data->m_full[0];
	data->magn->m_full[1] = magn_data->m_full[1];
	data->magn->m_full[2] = magn_data->m_full[2];
	
	// Liberacion de memoria
	uquad_mat_free(magn_raw);
	uquad_mat_free(C_b);	
	uquad_mat_free(magn_data);
}
#endif

void pres_raw2data(imu_raw_t *raw, imu_data_t * data)
{
	data->alt = PRESS_K*(1.0 - pow((raw->pres / po), PRESS_EXP));
}


double us_alt_coef = 0.2;
double us_alt_umbral = 0.1;
double imu_filter_us_alt(double us_alt)
{
	static double y_k_1,x_k_1 = 0; //salida anterior,medida anterior
	double y_k;

	if (( us_alt > (1+us_alt_umbral)*x_k_1) || (us_alt < (1-us_alt_umbral)*x_k_1 )) {
	   y_k = y_k_1;
	} else { 
	   y_k = us_alt*us_alt_coef + (1-us_alt_coef)*y_k_1;
	   x_k_1 = us_alt;
	}
	y_k_1 = y_k;

	return y_k;
}


void imu_raw2data(imu_raw_t *raw, imu_data_t *data)
{
	struct timeval tv_aux;

	data->T_us = raw->T_us;
	//temp_raw2data(raw, data);
	//magn_raw2data(raw, data);
	pres_raw2data(raw, data);

	data->us_obstacle = (raw->us_obstacle*1.695)/100;//*0.99226 + 3.51228;
	data->us_altitude = imu_filter_us_alt( (raw->us_altitude*1.695/100) );

	// Timestamp
	gettimeofday(&tv_aux,NULL);
	uquad_timeval_substract(&data->ts, tv_aux, get_main_start_time());

}

/*
 *
 * T_s_imu T_us_imu alt us_obstacle us_altitude
*/
int imu_to_str(char* buf_str, imu_data_t imu_data)
{
   char* buf_ptr = buf_str;
   //int ret;
   
   // Timestamp
   buf_ptr += sprintf(buf_ptr, " %04lu %06lu", (unsigned long)imu_data.ts.tv_sec, (unsigned long)imu_data.ts.tv_usec);
  
   buf_ptr += sprintf(buf_ptr, " %lf", imu_data.alt);
   buf_ptr += sprintf(buf_ptr, " %lf", imu_data.us_obstacle);
   buf_ptr += sprintf(buf_ptr, " %lf\n", imu_data.us_altitude);
   //buf_ptr += sprintf(buf_ptr,"\t");

   return (buf_ptr - buf_str); //char_count

}


/**
 * Simula movimiento del quad en base a modelo fisico TODO roll pitch
 *
 * m*(d^2)z/dt^2 = Th - B*dz/dt - P
 */
#define IMU_SAMPLE_TIME 	0.05
void imu_simulate_altitude(double *h, double u_h, double pitch, double roll)
{
   double accel, froz;
   static double vel = 0;   

   // Aceleracion	
   accel = u_h/MASA - G;

   int i = 0;
   for(i=0;i<5;i++) {
	
	// Velocidad
	vel = vel + accel*IMU_SAMPLE_TIME/5;
	
	// Altura
	*h = *h + vel*IMU_SAMPLE_TIME/5;
  }

   return;
}


