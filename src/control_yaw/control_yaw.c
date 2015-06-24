/******************************************************************************
 *
 * @file       control_yaw.h
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      ??
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
 *
 */

#include "control_yaw.h"
#include <stdlib.h>
#include <math.h>
#include <quadcop_config.h>

double Kp = 2.7;//3.5;//2.7;     
double Td = 0.4;
double alpha = 0.25;
double a = 0.5;
double b = 0.5;

/**
 * Buffer para almacenar senales de error
 * 
 * Elemento mas nuevo se almacena el lugar correspondiente
 *  al indice cero
 */
error_yaw_t error_yaw_buff[CONTROL_YAW_BUFF_SIZE];


/*
 * Inicializa el buffer
 */
void control_yaw_init_error_buff(void)
{
   error_yaw_t new_err;
   new_err.error = 0;
   int i;
   // Inicializo en cero
   for(i=0; i < CONTROL_YAW_BUFF_SIZE; i++)
   {
      error_yaw_buff[i] = new_err;
   }
      
   return;
}


/*
 * Agrega un elemento nuevo al buffer y elimina el ultimo.
 */
int control_yaw_add_error_buff(error_yaw_t new_err)
{
   int i;
   // Desplazo un lugar todos los elementos
   for(i=0; i < CONTROL_YAW_BUFF_SIZE-1; i++)
   {
      error_yaw_buff[CONTROL_YAW_BUFF_SIZE-1-i] = error_yaw_buff[CONTROL_YAW_BUFF_SIZE-2-i];
   }
   // Agrego elemento nuevo en el primer lugar
   error_yaw_buff[0] = new_err;
   
   return 0;
}

/*
 * Calcula la derivada discreta del error.
 *
 * Puede hacer promedio (MEAN_ENABLE == 1) o saltear muestras (MEAN_ENABLE == 0)
 */
double control_yaw_derivate_error(void)
{
   double err_mean_sup = 0;
   double err_mean_inf = 0;
   double mean_time = YAW_SAMPLE_TIME; //TODO calcularlo usando tiempo del timestamp de las muestras (act.ts)
   
   double err_dot;

#if CONTROL_YAW_MEAN_ENABLE
   // Promedio de muestras superiores e inferiores
   int i;
   for(i=0; i < CONTROL_YAW_BUFF_SIZE/2; i++)
   {
	err_mean_sup += error_yaw_buff[CONTROL_YAW_BUFF_SIZE-1-i].error;
	err_mean_inf += error_yaw_buff[i].error; 
   }
   // calculo la derivada como diferencia de promedios. Los elementos mas nuevos estan en
   // los indices mas chicos.
   // err_dot = ( (err_inf - err_sup)*(CONTROL_YAW_BUFF_SIZE/2) )/( mean_time*(CONTROL_YAW_BUFF_SIZE/2) );
   err_dot = ( err_mean_inf - err_mean_sup )/ mean_time;
#else
   err_dot = ( error_yaw_buff[0].error - error_yaw_buff[CONTROL_YAW_BUFF_SIZE-1].error )/( mean_time*(CONTROL_YAW_BUFF_SIZE-1) );
#endif

   return err_dot;
}


void control_yaw_filter_input(double *u)
{
   //datos anteriores
   static double y_k_1 = 0,
		 x_k_1 = 0;
   
   double x_k = *u; 

#if !CONTROL_YAW_ADD_ZERO
   //y_{k} = alpha*x_{k} + (1-alpha)*y_{k-1}
   double y_k = alpha*x_k + (1-alpha)*y_k_1;
#else
   //y_{k} = x_{k} + a*x_{k} - b*y_{k-1}
   double y_k = x_k + a*x_k_1 - b*y_k_1;
   x_k_1 = x_k;
#endif

   *u = y_k;
   y_k_1 = y_k;

   return;
}



/*
 * Agrega un elemento nuevo al buffer y elimina el ultimo.
 */
int control_yaw_print_error_buff(void)
{
   int i;
   // Desplazo un lugar todos los elementos
   for(i=0; i < CONTROL_YAW_BUFF_SIZE; i++)
   {
      printf("%lf  ",error_yaw_buff[i].error);
   }
   
   //printf("\n");
   
   return 0;
}



/*
 * Los parametros de entrada son en radianes pero el control es en grados
 */
double control_yaw_calc_input(double yaw_d, double yaw_measured) 
{
   double u = 180/M_PI*Kp*(yaw_d - yaw_measured); //El control se hace en grados y los datos enstan en radianes

#if CONTROL_YAW_ADD_DERIVATIVE
   error_yaw_t new_err;
   new_err.error = 180/M_PI*(yaw_d - yaw_measured);
   control_yaw_add_error_buff(new_err);

   //printf("%lf\n",control_yaw_derivate_error());   //dbg

   u += Kp*Td*control_yaw_derivate_error();
   printf("%lf  ",u);
   //Filtro la entrada a la planta para suavizar los picos
   control_yaw_filter_input(&u);
#endif

   control_yaw_print_error_buff(); //dbg
   printf("%lf",u);

   return u;
}


#if FAKE_YAW
/**
 * Simula cambio de yaw en base a yaw deseado
 *
 * devuelve yaw simulado
 */
double simulate_yaw(double yaw_d)
{
   static double last_yaw_simulated = INITIAL_YAW;

   double yaw_simulated = last_yaw_simulated + PORCENTAGE_UPDATE_YAW*(yaw_d - last_yaw_simulated);
   last_yaw_simulated = yaw_simulated;

   return yaw_simulated;
}
#endif


static double yaw_zero = 0;
void set_yaw_zero(double yaw_measured)
{
   yaw_zero = yaw_measured;
   return;
}


double get_yaw_zero(void)
{
   return yaw_zero;
}






