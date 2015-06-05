/******************************************************************************
 *
 * @file       quadcop_types.h
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

#ifndef UQUAD_TYPES_H
#define UQUAD_TYPES_H

#define VEL_DESIRED		3.0 //m/s
double masa = 1.85; // kg
double g = 9.81; // m/s*s
double B = 1; // coef friccion


// Almacena posicion actual del quad
typedef struct posicion {
   double x;
   double y;
   double z;
   struct timeval ts;
} posicion_t;
posicion_t posicion = {0,0,0,{0,0}};


#if !SIMULATE_GPS
// Almacena velovidad actual del quad
typedef struct velocidad {
   double module;
   double angle;
   struct timeval ts;
} velocidad_t;
#else
// Almacena velocidad actual del quad
typedef struct velocidad {
   double x;
   double y;
   double z;
   struct timeval ts;
} velocidad_t;
#endif



#endif
