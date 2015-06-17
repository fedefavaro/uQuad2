

#include "imu_comm.h"
#include <math.h>
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
// Acc
static uquad_mat_t *acc_TK_inv;
static uquad_mat_t *acc_b;
static double *acc_temp_z;		
static double *acc_temp;
// Gyro
static uquad_mat_t *gyro_TK_inv;
static uquad_mat_t *gyro_b;
static uquad_mat_t *gyro_temp_dep;
static uquad_mat_t *gyro_temp_indep;
static double *gyro_temp;
// Magn
static uquad_mat_t *magn_K;
static uquad_mat_t *magn_b;
// Baro
static double K;
static double *pres_K = &K;
static double po = 0;
static double *pres_po = &po;
static double expo;
static double *pres_exponente = & expo;   
    
// Buffer de recepcion.
static unsigned char RX_imu_buffer[31];
//static int RX_imu_buffer_inicio;
//static int RX_imu_buffer_ultimo;
//static int largo_mensaje;

// flag que identifica mensaje completo
static bool imu_ready;

//int cont = 0;


//*****************************************************************************
//
// True si mensaje completo
//
//*****************************************************************************
//bool check_imu_ready(void)
//{
//    return imu_ready;
//}



//*****************************************************************************
//
// Reseteo imu_ready
//
//*****************************************************************************
//void reset_imu_ready(void)
//{
//    imu_ready = false;
//}


/* devuelve true si puedo leer, false si no puedo */
bool check_read_locks(int fd) {

  fd_set rfds;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
  int retval = select(fd+1, &rfds, NULL, NULL, &tv);
  if(retval < 0)
      printf("select() failed!\n");
     
  return FD_ISSET(fd,&rfds);
}


//*****************************************************************************
//
// Leo datos de la imu
//
//*****************************************************************************
int imu_comm_read(int fd)
{
   int ret;
   uint8_t imu_data_ready = 0;
   uint8_t c;
   int index = 0;
   bool in_sync = false;

   while (!imu_data_ready && check_read_locks(fd))
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
        } else {
            if (index < 30) {
	       RX_imu_buffer[index++] = c;
	    } else { // index == 30 
               if ((c == 'Z')) {
                   RX_imu_buffer[index++] = c;
		   imu_data_ready = 1;
	       } else
		   return -1;
            }
	}

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
    buffParse_16 = (int16_t *) data;
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
    i++;
    frame->us_obstacle = *((int16_t*)(buffParse_16 + i++));
}



//*****************************************************************************
//
// Imprime los datos de imu_raw
//
//*****************************************************************************
void print_imu_raw(imu_raw_t *frame)
{
    printf("%u", frame->T_us);
    printf("  %i", frame->acc[0]);
    printf("  %i", frame->acc[1]);
    printf("  %i", frame->acc[2]);
    printf("  %i", frame->gyro[0]);
    printf("  %i", frame->gyro[1]);
    printf("  %i", frame->gyro[2]);
    printf("  %i", frame->magn[0]);
    printf("  %i", frame->magn[1]);
    printf("  %i", frame->magn[2]);
    printf("  %u", frame->temp);
    printf("  %lu", frame->pres);
    printf("  %i\n", frame->us_obstacle);
}



//*****************************************************************************
//
// Calibraciones IMU
//
//*****************************************************************************
#if 0
void acc_calib_init(void)
{
    // Cargo K
    uquad_mat_t *K = uquad_mat_alloc(3, 3);
    K->m_full[0] = 26.7572;
    K->m_full[1] = 0.0;
    K->m_full[2] = 0.0;
    K->m_full[3] = 0.0;
    K->m_full[4] = 27.0599;
    K->m_full[5] = 0.0;
    K->m_full[6] = 0.0;
    K->m_full[7] = 0.0;
    K->m_full[8] = 25.9838;
	
    // Cargo T
    uquad_mat_t *T = uquad_mat_alloc(3, 3);
    T->m_full[0] = 1.0;
    T->m_full[1] = -0.0005;
    T->m_full[2] = 0.0025;
    T->m_full[3] = 0.0149;
    T->m_full[4] = 1.0;
    T->m_full[5] = -0.0023;
    T->m_full[6] = 0.0;
    T->m_full[7] = -0.0077;
    T->m_full[8] = 1;
	
    // Cargo b
    acc_b = uquad_mat_alloc(3, 1);
    acc_b->m_full[0] = 16.9374;
    acc_b->m_full[1] = -0.2446;
    acc_b->m_full[2] = -47.7863;

    // Cargo coef de calibracion por temperatura: acc_temp_z
    *acc_temp_z = -0.604990434419997;
	
    // Cargo temperatura con la que se calibro: temp_calib
    *acc_temp = 23.3093987230917;
	
    // Invierto Matriz K: K_inv
    uquad_mat_t *K_inv = uquad_mat_alloc(3, 3);
    uquad_mat_inv(K_inv, K, NULL, NULL);
	
    // Multiplicacion: TK_inv = T*K_inv
    acc_TK_inv = uquad_mat_alloc(3, 3);
    uquad_mat_prod(acc_TK_inv, T, K_inv);

    // Libero memoria
    uquad_mat_free(K);
    uquad_mat_free(K_inv);
    uquad_mat_free(T);
}


void gyro_calib_init(void)
{
    // Cargo K
    uquad_mat_t *K = uquad_mat_alloc(3, 3);
    K->m_full[0] = 796.6646;
    K->m_full[1] = 0.0;
    K->m_full[2] = 0.0;
    K->m_full[3] = 0.0;
    K->m_full[4] = 808.2419;
    K->m_full[5] = 0.0;
    K->m_full[6] = 0.0;
    K->m_full[7] = 0.0;
    K->m_full[8] = 803.4482;
	
    // Cargo T
    uquad_mat_t *T = uquad_mat_alloc(3, 3);
    T->m_full[0] = 1.0;
    T->m_full[1] = -0.0093;
    T->m_full[2] = 0.0659;
    T->m_full[3] = 0.0093;
    T->m_full[4] = 1.0;
    T->m_full[5] = -0.0128;
    T->m_full[6] = 0.025;
    T->m_full[7] = 0.0552;
    T->m_full[8] = 1.0;
	
    // Cargo b
    gyro_b = uquad_mat_alloc(3, 1);
    gyro_b->m_full[0] = -25.3819;
    gyro_b->m_full[1] = -15.721;
    gyro_b->m_full[2] = -1.8871;
	
    // Cargo matriz de calibracion por temperatura dependiente
    gyro_temp_dep = uquad_mat_alloc(3, 1);
    gyro_temp_dep->m_full[0] = -0.361070248326612;
    gyro_temp_dep->m_full[1] = 2.79836504468675;
    gyro_temp_dep->m_full[2] = -1.94461170726143;
	
    // Cargo matriz de calibracion por temperatura independiente
    gyro_temp_indep = uquad_mat_alloc(3, 1);
    gyro_temp_indep->m_full[0] = -0.361070248326612;
    gyro_temp_indep->m_full[1] = 2.79836504468675;
    gyro_temp_indep->m_full[2] = -1.94461170726143;
	
    // Invierto Matriz K: K_inv
    uquad_mat_t *K_inv = uquad_mat_alloc(3, 3);
    uquad_mat_inv(K_inv, K, NULL, NULL);
	
    // Multiplicacion: TK_inv = T*K_inv
    gyro_TK_inv = uquad_mat_alloc(3, 3);
    uquad_mat_prod(gyro_TK_inv, T, K_inv);
    
    *gyro_temp = 27.9869547673702;
    
    // Libero memoria
    uquad_mat_free(K);
    uquad_mat_free(K_inv);
    uquad_mat_free(T);
}
#endif //if 0


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


void pres_calib_init(void)
{
    int i;

    // Cargo K
    *pres_K = 44330;
    
    // Cargo exp
    *pres_exponente = 0.191387559808612;
	
    // Cargo po promediando varios resultados con el quad en el piso, previo al despegue
    imu_raw_t raw;
    imu_ready = false;
    for (i=0; i < 500; i++) {
        while(!imu_ready);
        imu_comm_parse_frame_binary(&raw);
        *pres_po = *pres_po + raw.pres;
    }
    *pres_po = *pres_po/500;
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


void acc_raw2data(imu_raw_t *raw, imu_data_t *data)
{
	// Modelo Acc: acc_raw = K*T_inv*acc_data + b
	//			==> acc_data = T*K_inv*(acc_raw - b) 
	
	// Paso los datos crudos del Acc a una matriz acc_raw
	uquad_mat_t *acc_raw = uquad_mat_alloc(3, 1);
	acc_raw->m_full[0] = raw->acc[0];
	acc_raw->m_full[1] = raw->acc[1];
	acc_raw->m_full[2] = raw->acc[2];

	// Sumo: b = b + b_temp (solo eje Z)
	acc_b->m_full[2] = acc_b->m_full[2] + (*acc_temp_z*(data->temp - *acc_temp));

	// Resto: A_b = A - b (donde A son los datos crudos del Acc)
	uquad_mat_t *A_b = uquad_mat_alloc(3, 1);
	uquad_mat_sub(A_b, acc_raw, acc_b);

	// Multiplicacion: acc_data = TK_inv*A_b
	uquad_mat_t *acc_data = uquad_mat_alloc(3, 1);
	uquad_mat_prod(acc_data, acc_TK_inv, A_b);
	
	// Copio resultado en matriz de salida
	data->acc->m_full[0] = acc_data->m_full[0];
	data->acc->m_full[1] = acc_data->m_full[1];
	data->acc->m_full[2] = acc_data->m_full[2];
	
	// Libero memoria
	uquad_mat_free(A_b);
	uquad_mat_free(acc_raw);
	uquad_mat_free(acc_data);
}

void gyro_raw2data(imu_raw_t *raw, imu_data_t *data)
{
	// Modelo Gyro: G = T*K_inv*(G_raw - b -Kt*(temp - temp0) - bt) 
		
	// Paso los datos crudos del Gyro a una matriz G_raw
	uquad_mat_t *gyro_raw = uquad_mat_alloc(3, 1);
	gyro_raw->m_full[0] = raw->gyro[0];
	gyro_raw->m_full[1] = raw->gyro[1];
	gyro_raw->m_full[2] = raw->gyro[2];
	
	// Multiplicacion: Kt * (temp - temp0)
	uquad_mat_t *Kt_t_to = uquad_mat_alloc(3, 1);
	Kt_t_to->m_full[0] = gyro_temp_dep->m_full[0]*(data->temp - *gyro_temp);
	Kt_t_to->m_full[1] = gyro_temp_dep->m_full[1]*(data->temp - *gyro_temp);
	Kt_t_to->m_full[2] = gyro_temp_dep->m_full[2]*(data->temp - *gyro_temp);
	
	// Resto: G_raw - b
	uquad_mat_t *G_b = uquad_mat_alloc(3, 1);
	uquad_mat_sub(G_b, gyro_raw, gyro_b);
	
	// Resto (G_raw - b) - (Kt*(temp - temp0))
	uquad_mat_t *G_b_Kt = uquad_mat_alloc(3, 1);
	uquad_mat_sub(G_b_Kt, G_b, Kt_t_to);
	
	// Resto (G_raw - b - Kt*(temp - temp0)) - bt
	uquad_mat_t *G_b_kt_bt = uquad_mat_alloc(3, 1);
	uquad_mat_sub(G_b_kt_bt, G_b_Kt, gyro_temp_indep);
	
	// Multiplicacion: TK_inv * (G_raw - b - Kt*(temp - temp0) - bt)
	uquad_mat_t *gyro_data = uquad_mat_alloc(3, 1);
	uquad_mat_prod(gyro_data, gyro_TK_inv, G_b_kt_bt);
	
	// Copio resultado en matriz de salida
	data->gyro->m_full[0] = gyro_data->m_full[0];
	data->gyro->m_full[1] = gyro_data->m_full[1];
	data->gyro->m_full[2] = gyro_data->m_full[2];
	
	// Libero memoria
	uquad_mat_free(gyro_raw);
	uquad_mat_free(Kt_t_to);
	uquad_mat_free(G_b);
	uquad_mat_free(G_b_Kt);
	uquad_mat_free(G_b_kt_bt);
	uquad_mat_free(gyro_data);
}
#endif //if 0


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


void pres_raw2data(imu_raw_t *raw, imu_data_t * data)
{
	data->alt = *pres_K*(1 - pow((raw->pres / *pres_po), *pres_exponente));
}


void imu_raw2data(imu_raw_t *raw, imu_data_t *data)
{
	data->T_us = raw->T_us;
	//temp_raw2data(raw, data);
	//acc_raw2data(raw, data);
	//gyro_raw2data(raw, data);
	magn_raw2data(raw, data);
	pres_raw2data(raw, data);
}



//*****************************************************************************
//
// Inicializacion imu_data
//
//*****************************************************************************
void imu_data_alloc(imu_data_t *imu_data)
{
    imu_data->acc = uquad_mat_alloc(3,1);
    imu_data->gyro = uquad_mat_alloc(3,1);
    imu_data->magn = uquad_mat_alloc(3,1);

    // initialize data to zeros
    uquad_mat_zeros(imu_data->acc);
    uquad_mat_zeros(imu_data->gyro);
    uquad_mat_zeros(imu_data->magn);
    //imu_data->acc_ok = false;
    //imu_data->magn_ok = false;
}



