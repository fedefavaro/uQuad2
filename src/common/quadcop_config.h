/**
 ******************************************************************************
 *
 * @file       quadcop_config.h
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      Opciones de configuracion para software QuadCop
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
 */

#ifndef UQUAD_CONFIG_h
#define UQUAD_CONFIG_h

#define PC_TEST			1	

#if PC_TEST
   #define SBUS_LOG_TO_FILE	0 //sbus logea en un archivo en lugar de imprimir en stdout
#endif //PC_TEST

#define SIMULATE_GPS		1
#define DISABLE_UAVTALK		1
#define FAKE_YAW		0

#define DEBUG                   1

#if DEBUG
   #define DEBUG_TIMING_MAIN	0 //imprime en stdout la duracion del loop
   #define DEBUG_TIMING_SBUSD	0
#endif //DEBUG






#endif //UQUAD_CONFIG_h
