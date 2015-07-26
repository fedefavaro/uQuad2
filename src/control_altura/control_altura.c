/******************************************************************************
 *
 * @file       control_altura.h
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

#include "control_altura.h"
#include <stdlib.h>
#include <math.h>
#include <quadcop_config.h>

double Kp_alt = 1.85;     
double Td_alt = 2.0;
double alpha_alt = 0.3;
double a_alt = 0.5;
double b_alt = 0.5;

/**
 * Buffer para almacenar senales de error
 * 
 * Elemento mas nuevo se almacena el lugar correspondiente
 *  al indice cero
 */
error_alt_t error_alt_buff[CONTROL_ALT_BUFF_SIZE];


/*
 * Inicializa el buffer
 */
void control_alt_init_error_buff(void)
{
   error_alt_t new_err;
   new_err.error = 0;
   int i;
   // Inicializo en cero
   for(i=0; i < CONTROL_ALT_BUFF_SIZE; i++)
   {
      error_alt_buff[i] = new_err;
   }
      
   return;
}


/*
 * Agrega un elemento nuevo al buffer y elimina el ultimo.
 */
int control_alt_add_error_buff(error_alt_t new_err)
{
   int i;
   // Desplazo un lugar todos los elementos
   for(i=0; i < CONTROL_ALT_BUFF_SIZE-1; i++)
   {
      error_alt_buff[CONTROL_ALT_BUFF_SIZE-1-i] = error_alt_buff[CONTROL_ALT_BUFF_SIZE-2-i];
   }
   // Agrego elemento nuevo en el primer lugar
   error_alt_buff[0] = new_err;
   
   return 0;
}

/*
 * Calcula la derivada discreta del error.
 *
 * Puede hacer promedio (MEAN_ENABLE == 1) o saltear muestras (MEAN_ENABLE == 0)
 */
double control_alt_derivate_error(void)
{
   double err_mean_sup = 0;
   double err_mean_inf = 0;
   double mean_time = ALT_SAMPLE_TIME; //TODO calcularlo usando tiempo del timestamp de las muestras (act.ts)
   
   double err_dot;

#if CONTROL_ALT_MEAN_ENABLE
   // Promedio de muestras superiores e inferiores
   int i;
   for(i=0; i < CONTROL_ALT_BUFF_SIZE/2; i++)
   {
	err_mean_sup += error_alt_buff[CONTROL_ALT_BUFF_SIZE-1-i].error;
	err_mean_inf += error_alt_buff[i].error; 
   }
   err_mean_sup = err_mean_sup/(CONTROL_ALT_BUFF_SIZE/2);
   err_mean_inf = err_mean_inf/(CONTROL_ALT_BUFF_SIZE/2);
   // calculo la derivada como diferencia de promedios. Los elementos mas nuevos estan en los indices mas chicos.
   // e_dot = e_mean_inf - e_mean_sup / (n/2)*T
   err_dot = ( err_mean_inf - err_mean_sup )/(mean_time*CONTROL_ALT_BUFF_SIZE/2);
#else
   err_dot = ( error_alt_buff[0].error - error_alt_buff[CONTROL_ALT_BUFF_SIZE-1].error )/( mean_time*(CONTROL_ALT_BUFF_SIZE-1) );
#endif

   return err_dot;
}


void control_alt_filter_input(double *u)
{
   //datos anteriores
   static double y_k_1 = 0,
		 x_k_1 = 0;
   
   double x_k = *u; 

#if !CONTROL_ALT_ADD_ZERO
   //y_{k} = alpha_alt*x_{k} + (1-alpha_alt)*y_{k-1}
   double y_k = alpha_alt*x_k + (1-alpha_alt)*y_k_1;
#else
   //y_{k} = x_{k} + a_alt*x_{k} - b_alt*y_{k-1}
   double y_k = x_k + a_alt*x_k_1 - b_alt*y_k_1;
   x_k_1 = x_k;
#endif

   *u = y_k;
   y_k_1 = y_k;

   return;
}


/*
 * 
 */
double control_alt_calc_input(double alt_d, double alt_measured) 
{

   double u = Kp_alt*(alt_d - alt_measured); 

   error_alt_t new_err;
   new_err.error = (alt_d - alt_measured);
   control_alt_add_error_buff(new_err);
   u += Kp_alt*Td_alt*control_alt_derivate_error();
   //Filtro la entrada a la planta para suavizar los picos
   control_alt_filter_input(&u);

   return u;
}


/*
 * 
 */
double ki = 24.5; //N/m
double alpha_alt_i = 0.001; //promedio de 200 muestras
double control_alt_integral(double alt_d, double alt_measured) 
{

   static double y_k_1 = 0;
   double x_k = alt_d - alt_measured; //senal de error
   double y_k = alpha_alt_i*x_k + (1-alpha_alt_i)*y_k_1;
   if (y_k > 0.1)
	y_k = 0.1;
   if (y_k < -0.1)
	y_k = -0.1;

   y_k_1 = y_k;

   return y_k*ki;
}



static double alt_zero = 0;
void set_alt_zero(double alt_measured)
{
   alt_zero = alt_measured;
   return;
}


double get_alt_zero(void)
{
   return alt_zero;
}


/* Regula la altura deseada durante el despegue
 *
 * retorna 0 mientras la altura no haya sido alcanzada y 1
 * cuando esta ha sido alcanzada.
 */
int control_altitude_takeoff(double *h_d)
{
   *h_d = *h_d + TAKEOFF_ALTITUDE*PORCENTAGE_UPDATE_TA;
   if (*h_d >= TAKEOFF_ALTITUDE)
	return 1;
   else
	return 0;
}



/* Regula la altura deseada durante el aterrizaje
 *
 * retorna 0 mientras la altura no haya sido alcanzada y 1
 * cuando esta ha sido alcanzada.
 */
int control_altitude_land(double *h_d)
{
   *h_d = *h_d - LANDING_ALTITUDE*PORCENTAGE_UPDATE_TA;
   if (*h_d <= LANDING_ALTITUDE)
	return 1;
   else
	return 0;
}


