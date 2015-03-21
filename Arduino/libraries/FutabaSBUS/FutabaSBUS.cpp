#include "FutabaSBUS.h"

uint8_t sbusData[25] 	= {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
int16_t channels[18]  	= {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
int16_t servos[18]    	= {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
uint8_t failsafe_status = SBUS_SIGNAL_OK;
bool sbus_passthrough 	= false;

FutabaSBUS::FutabaSBUS(void) {

  	//port.begin(BAUDRATE);//,CONFIG);
	//Serial2.begin(115200);
	// attaches ticker_500us() as a timer overflow interrupt
	//rxSBUS.attachInterrupt(ticker_500us); 	
	// initialize Timer1, and set a 500 us period
	//rxSBUS.start(timerCount);
 	rx_timeout=50;
    	tx_timeout=60;
        servosReady = false;
}


void FutabaSBUS::begin(void) {
     //Serial2.begin(BAUDRATE);
     Serial3.begin(BAUDRATE,CONFIG);
}
 

int16_t FutabaSBUS::channel(uint8_t ch) {
  // Read channel data
  if ((ch>0)&&(ch<=16)){
    return channels[ch-1];
  }
  else{
    return 1023;
  }
}


void FutabaSBUS::servo(uint8_t ch, int16_t position) {
  // Set servo position
  if ((ch>0) && (ch<=16)) {
    if (position>2048) {
      position=2048;
    }
    servos[ch-1] = position;
  }
}


uint8_t FutabaSBUS::failsafe(void) {
  return failsafe_status;
}


void FutabaSBUS::setPassthrough(bool mode) {
  // Set passtrough mode, if true, received channel data is send to servos
  sbus_passthrough = mode;
}


bool FutabaSBUS::getPassthrough(void) {
  // Return current passthrough mode
  return sbus_passthrough;
}


void FutabaSBUS::updateServos(void) {

  //Serial.println("Update servos");
  // Send data to servos
  // Passtrough mode = false >> send own servo data
  // Passtrough mode = true >> send received channel data
  uint8_t i;
  if (!sbus_passthrough) {
    // clear received channel data
    for (i=1; i<24; i++) {
      sbusData[i] = 0;
    }

    // reset counters
    uint8_t ch = 0;
    uint8_t bit_in_servo = 0;
    uint8_t byte_in_sbus = 1;
    uint8_t bit_in_sbus = 0;

    // store servo data
    for (i=0; i<176; i++) {
      if (servos[ch] & (1<<bit_in_servo)) {
        sbusData[byte_in_sbus] |= (1<<bit_in_sbus);
      }
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

 /*   // DigiChannel 1
    if (channels[16] == 1) {
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
  
  // send data out
 // for (i=0;i<25;i++) {
    //port.print(sbusData[i]);
    //while (port.available());
    //Serial.print(port.read(), HEX);
  //  Serial.write(sbusData[i]);
  //}
  port.write(sbusData,25); 
  //port.flush();
  
}

void FutabaSBUS::updateChannels(void) {
	// Read all received data and calculate channel data
	uint8_t i;
	uint8_t sbus_pointer = 0;
	
	while (port.available()) 
	{
        uint8_t data = port.read(); // get data from serial rx buffer
        switch (sbus_pointer) {
            case 0: // Byte 1
                if (data==0x0f) {
                    sbusData[sbus_pointer] = data;
                    sbus_pointer++;
                }
                break;

            case 24:    // Byte 25 >> if last byte == 0x00 >> convert data
                if (data==0x00) {
                    sbusData[sbus_pointer] = data;
                    // clear channels[]
                    for (i=0; i<16; i++)
						channels[i] = 0;

                    // reset counters
                    uint8_t byte_in_sbus = 1;
                    uint8_t bit_in_sbus = 0;
                    uint8_t ch = 0;
                    uint8_t bit_in_channel = 0;

                    // process actual sbus data
                    for (i=0; i<176; i++) {
                        if (sbusData[byte_in_sbus] & (1<<bit_in_sbus)) {
                            channels[ch] |= (1<<bit_in_channel);
                        }
                        bit_in_sbus++;
                        bit_in_channel++;

                        if (bit_in_sbus == 8) {
                            bit_in_sbus =0;
                            byte_in_sbus++;
                        }
                        if (bit_in_channel == 11) {
                            bit_in_channel =0;
                            ch++;
                        }
                    }
					
					// Faster method
/*					channels[0]  = ((sbusData[1]    |sbusData[2]<< 8) 		     		& 0x07FF);
					channels[1]  = ((sbusData[2]>>3 |sbusData[3]<<5) 		     		& 0x07FF);
					channels[2]  = ((sbusData[3]>>6 |sbusData[4]<<2|sbusData[5]<<10)   	& 0x07FF);
					channels[3]  = ((sbusData[5]>>1 |sbusData[6]<<7) 		     		& 0x07FF);
					channels[4]  = ((sbusData[6]>>4 |sbusData[7]<<4) 		     		& 0x07FF);
					channels[5]  = ((sbusData[7]>>7 |sbusData[8]<<1|sbusData[9]<<9)    	& 0x07FF);
					channels[6]  = ((sbusData[9]>>2 |sbusData[10]<<6) 		    		& 0x07FF);
					channels[7]  = ((sbusData[10]>>5|sbusData[11]<<3) 		    		& 0x07FF); // & the other 8 + 2 channels if you need them
					#ifdef ALL_CHANNELS
					channels[8]  = ((sbusData[12]	|sbusData[13]<< 8) 					& 0x07FF);
					channels[9]  = ((sbusData[13]>>3|sbusData[14]<<5) 		     		& 0x07FF);
					channels[10] = ((sbusData[14]>>6|sbusData[15]<<2|sbusData[16]<<10) 	& 0x07FF);
					channels[11] = ((sbusData[16]>>1|sbusData[17]<<7) 		     		& 0x07FF);
					channels[12] = ((sbusData[17]>>4|sbusData[18]<<4) 		     		& 0x07FF);
					channels[13] = ((sbusData[18]>>7|sbusData[19]<<1|sbusData[20]<<9)  	& 0x07FF);
					channels[14] = ((sbusData[20]>>2|sbusData[21]<<6) 		     		& 0x07FF);
					channels[15] = ((sbusData[21]>>5|sbusData[22]<<3) 		     		& 0x07FF);
					#endif
*/					
					
/*                  // DigiChannel 1
                    if (sbusData[23] & (1<<0)) {
                        channels[16] = 1;
                    }else{
                        channels[16] = 0;
                    }
                    // DigiChannel 2
                    if (sbusData[23] & (1<<1)) {
                        channels[17] = 1;
                    }else{
                        channels[17] = 0;
                    }
*/
                    // Failsafe
                    failsafe_status = SBUS_SIGNAL_OK;
                    if (sbusData[23] & (1<<2)) {
                        failsafe_status = SBUS_SIGNAL_LOST;
                    }
                    if (sbusData[23] & (1<<3)) {
                        failsafe_status = SBUS_SIGNAL_FAILSAFE;
                    }
                }
                break;

            default:  // collect Channel data (11bit) / Failsafe information
                sbusData[sbus_pointer] = data;
                sbus_pointer++;
        }
    }

}


void FutabaSBUS::ticker_500us(void)
{
  // RX
/*    switch (rx_timeout) {
        case 0:
            break;
        case 1:
            if (port.available())
		updateChannels();
	default:
            rx_timeout--;
    }*/
    // TX
    switch (tx_timeout) {
        case 0:
            updateServos();
            tx_timeout = 28;
        default:
            tx_timeout--;
    }
}



