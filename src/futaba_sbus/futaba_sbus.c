#include "futaba_sbus.h"
#include <uquad_aux_time.h>
#include <uquad_error_codes.h>

uint8_t sbusData[25] 	= {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
int16_t servos[18]    	= {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
uint8_t failsafe_status = SBUS_SIGNAL_OK;
static volatile int rx_timeout;
static volatile int tx_timeout;

#if PC_TEST
struct timeval tv_start; // Guarda el tiempo de comienzo del programa
#endif //PC_TEST

int futaba_sbus_begin(void) {
     
   rx_timeout=50;
   rx_timeout=60;
 
#if PC_TEST
   gettimeofday(&tv_start,NULL);
#endif //PC_TEST

     return 0;
}
 

void futaba_sbus_servo(uint8_t ch, int16_t position) {
  // Set servo position
  if ((ch>0) && (ch<=16)) {
    if (position>2048) {
      position=2048;
    }
    servos[ch-1] = position;
  }
}


uint8_t futaba_sbus_failsafe(void) {
  return failsafe_status;
}


// send data out must be done in main program
void futaba_sbus_updateServos(void) {

  // Send data to servos
  uint8_t i;

  for (i=1; i<24; i++)
    sbusData[i] = 0;
  

    // reset counters
  uint8_t ch = 0;
  uint8_t bit_in_servo = 0;
  uint8_t byte_in_sbus = 1;
  uint8_t bit_in_sbus = 0;

  // store servo data
  for (i=0; i<176; i++) {
     if (servos[ch] & (1<<bit_in_servo))
        sbusData[byte_in_sbus] |= (1<<bit_in_sbus);
     
     bit_in_sbus++;
     bit_in_servo++;

     if (bit_in_sbus == 8) {
        bit_in_sbus =0;
        byte_in_sbus++;
     }
     if (bit_in_servo == 11) {
        bit_in_servo =0;
        ch++;
     }
  }

    // DigiChannel 1
    /*if (channels[16] == 1) {
      sbusData[23] |= (1<<0);
    }
    // DigiChannel 2
    if (channels[17] == 1) {
      sbusData[23] |= (1<<1);
    } */

    // Failsafe
  if (failsafe_status == SBUS_SIGNAL_LOST) {
     sbusData[23] |= (1<<2);
  }

  if (failsafe_status == SBUS_SIGNAL_FAILSAFE) {
     sbusData[23] |= (1<<2);
     sbusData[23] |= (1<<3);
  }
    
}

// Return a pointer to the sbusData array
uint8_t * futaba_sbus_ptrsbusData(void) {
  return sbusData;

}


int futaba_sbus_resetServos(void)
{
   int j;
   for(j=1;j<17;j++)
     futaba_sbus_servo(j, 0);
   futaba_sbus_updateServos();

   return 0;
}

#if PC_TEST
/* Si se esta realizando la prueba en un PC, esta funcion se encarga de 
 * loggear en un formato legible el string sbus. */
int convert_sbus_data(char* buf_str)
{
   char* buf_ptr = buf_str;
   int i,ret;
   struct timeval tv_ts; //for timestamp

   // Timestamp
   gettimeofday(&tv_ts,NULL);
   ret = uquad_timeval_substract(&tv_ts, tv_ts, tv_start);
   if(ret > 0)
   {
      buf_ptr += sprintf(buf_ptr, "%04lu:%06lu  ", (unsigned long)tv_ts.tv_sec, (unsigned long)tv_ts.tv_usec);
   }
   else
   {
      err_log("WARN: Absurd timing!");
      return -1;
   }
   
   // Preparo el mensaje sbus
   for(i=0;i<SBUS_DATA_LENGTH;i++)
   {
      buf_ptr += sprintf(buf_ptr, "%02X ", sbusData[i]);
   }
   sprintf(buf_ptr,"\n");
   
   return 0; //char_count?

}
#else

/* Envia el mensaje sbus al puerto serie definido por fd. */
int write_sbus_data(int fd)
{
   int ret = write(fd, sbusData, 25);
   if (ret < 0)
   {
      err_log("write() failed!");
      return -1;
   }

   return 0;

}
#endif //PC_TEST




