#ifndef FUTABA_SBUS_h
#define FUTABA_SBUS_h

#include <uquad_config.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define SBUS_DATA_LENGTH    	25

#if PC_TEST
#define START_SBUS_ARG 		"sbusd.log" //test en un PC linux
#else
#define START_SBUS_ARG		"/dev/ttyO1" //en la beagle
#endif

int futaba_sbus_begin(void);
    
/** Set servo position, raw data, range 1000..2000?
 *
 * &param raw data 0..2048
 */
void futaba_sbus_servo(uint8_t ch, int16_t position);

/** Read failsafe condition
 *
 * &param 0=no failsafe 1=lost signal 3=failsafe
 */
uint8_t futaba_sbus_failsafe(void);

void futaba_sbus_updateServos(void);

uint8_t * futaba_sbus_ptrsbusData(void);

int futaba_sbus_resetServos(void);

#if PC_TEST
int convert_sbus_data(char* buf_str);
#else
int write_sbus_data(int fd);
#endif //PC_TEST

int futaba_sbus_start_daemon(void);

#endif



