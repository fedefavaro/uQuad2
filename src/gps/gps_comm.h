/**
 ******************************************************************************
 *
 * @file       gps_comm.h
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      Implementa la comunicacion con el gps a traves de gpsd.
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

#ifndef GPS_COMM_h
#define GPS_COMM_h

#include <uquad_error_codes.h>
#include <quadcop_config.h>
#include <quadcop_types.h>

#include <stdlib.h>
#include <gps.h>
#include <errno.h>

#define	KILL_GPSD		"killall gpsd"
//#define	START_GPSD		"gpsd /dev/ttyUSB0 -S 1234"
#define	START_GPSD_PATH		"/usr/sbin/gpsd"

#define	START_GPSD_DEV		"/dev/ttyUSB0"  // Conectado a pines CN2 del FTDI mini module
#define	START_GPSD_PORT		"1234"

typedef struct gps {
   double latitude;	/* Latitude in degrees (valid if mode >= 2) */
   double longitude;	/* Longitude in degrees (valid if mode >= 2) */
   double altitude;	/* Altitude in meters (valid if mode == 3) */
   double speed;	/* Speed over ground, meters/sec */
   double track;	/* Course made good (relative to true north) */
} gps_t;

int preconfigure_gps(void);
int init_gps(void);
int deinit_gps(void);
/**
 * Inicia el demonio gpsd en paralelo al main
 *
 * @return pid del child process o codigo de error
 */ 
int start_gpsd(void);
int get_gps_data(gps_t* gps);

int gps_connect(const char *device, int baud);
int gps_disconnect(int fd);
int gps_send_command(int fd, const char *command);

void gps_simulate_position(position_t* pos, velocity_t* vel, double yaw_measured, double pitch);

#endif
