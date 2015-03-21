#ifndef FUTABA_SBUS_h
#define FUTABA_SBUS_h

#include <Arduino.h>
#include <DueTimer.h>
#include <stdbool.h>

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

#define BAUDRATE	100000
#define CONFIG		SERIAL_8E2
#define port 		Serial3

#define rxSBUS		Timer6
#define timerCount	500 		//microseconds
//#define ALL_CHANNELS

class FutabaSBUS
{
public:
    /** create a FutabaSBUS object initializing serial port
	*   with the defined BAUDRATE and CONFIG. Also sets the
	*   timer specified by rxSBUS to interrupt every timerCount
	*   microseconds.
    *
    *   &param 
    */
    FutabaSBUS(void);		
		
    void begin(void);

    /** Read channel(1..16), digital raw data
    *
    * &param raw data from receiver range from 0 to 4096, normal from 352 1696
    */
    int16_t channel(uint8_t ch);
    
    /** Set servo position, raw data, range 200..2000?
    *
    * &param raw data 0..2048
    */
    void servo(uint8_t ch, int16_t position);

    /** Read failsafe condition
    *
    * &param 0=no failsafe 1=lost signal 3=failsafe
    */
    uint8_t failsafe(void);

    /** Set logical data passtrough - servo values are ignored, using received data
    *
    * &param bool
    */
    void setPassthrough(bool mode);

    /** Read logical data passtrough
    *
    * &param bool
    */
    bool getPassthrough(void);

    void ticker_500us();
    volatile bool servosReady;
    void updateServos(void);
    
private:
    
    void updateChannels(void);
    volatile int rx_timeout;
    volatile int tx_timeout;
};

#endif
