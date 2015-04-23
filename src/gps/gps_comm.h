#ifndef GPS_COMM_h
#define GPS_COMM_h

#include <uquad_error_codes.h>
#include <uquad_config.h>

#include <stdlib.h>
#include <gps.h>
#include <errno.h>

#define	KILL_GPSD		"killall gpsd"
#define	START_GPSD		"gpsd /dev/ttyUSB0 -S 1234"

int init_gps(void);
int deinit_gps(void);
int start_gpsd(void);
int get_gps_data(void);

#endif
