/**
 ******************************************************************************
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


#ifndef CONTROL_YAW_H
#define CONTROL_YAW_H

#include <stdlib.h>


#define CONTROL_YAW_BUFF_SIZE		4 //usar numeros pares
#define YAW_SAMPLE_TIME			0.05 //en segundos

#define CONTROL_YAW_ADD_DERIVATIVE	0

#define PORCENTAGE_UPDATE_YAW		0.06
#define INITIAL_YAW			M_PI/6


typedef struct error_yaw {
	double error;
	struct timeval ts;
} error_yaw_t;


/*
 * Inicializa el buffer
 */
int control_yaw_init_error_buff(void);


/*
 * Agrega un elemento nuevo al buffer y elimina el ultimo.
 */
int control_yaw_add_error_buff(error_yaw_t new_err);


/*
 * Calcula la derivada discreta del error.
 *
 * Puede hacer promedio opt=1 o saltear muestras opt=0
 */
double control_yaw_derivate_error(int8_t opt);


/*
 * Calcula la senal de error.
 *
 * Control proporcional o PD segun definido por usuario
 */
double control_yaw_calc_error(double yaw_d, double yaw_measured);

double simulate_yaw(double yaw_measured);

static double yaw_zero;
void set_yaw_zero(double yaw_measured);
double get_yaw_zero(void);

#endif // CONTROL_YAW_H



