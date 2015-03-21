#ifndef FUTABA_SBUS_h
#define FUTABA_SBUS_h

#include <stdbool.h>
#include <stdint.h>

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

int FutabaSBUS_begin(void);
    
/** Set servo position, raw data, range 1000..2000?
 *
 * &param raw data 0..2048
 */
void FutabaSBUS_servo(uint8_t ch, int16_t position);

/** Read failsafe condition
 *
 * &param 0=no failsafe 1=lost signal 3=failsafe
 */
uint8_t FutabaSBUS_failsafe(void);

void FutabaSBUS_updateServos(void);

uint8_t * FutabaSBUS_ptrsbusData(void);

static volatile int rx_timeout;
static volatile int tx_timeout;

#endif
