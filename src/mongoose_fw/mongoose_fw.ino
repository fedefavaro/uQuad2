// uQuad!
//
// Based on:
// 	Mongoose Base AHRS firmware v1.1
// 	Mongoose 9DoF IMU + Barometric pressure sensor
// 	www.ckdevices.com
//
//	Released under Creative Commons License 
// 	Modifications and additions by Cory Duce to allow it to work with Mongoose hardware
// 	Based on code by Doug Weibel and Jose Julio which was based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
//

// Axis definition: 
   // X axis pointing forward (to the battery connector)
   // Y axis pointing to the right 
   // Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise


// The calibration values for the sensors need to be set in the function "mongooseCalibrate" which
// is in the file "ApplicationRoutines.pde"

/* Mongoose Hardware version - v1.0
	
	ATMega328@3.3V w/ external 8MHz resonator
	High Fuse DA
        Low Fuse FF
	
	ADXL345: Accelerometer
	HMC5843: Magnetometer
	IGT-3200: Gyro
        BMP085: Barometric Pressure sensor
        
        Programmer : 3.3v FTDI
        Arduino IDE : Select board  "Arduino Pro or Pro mini (3.3V, 8MHz) w/ Atmega328"
*/

#include <Arduino.h>
#include <HMC58X3.h>
#include <EEPROM.h>
#include <Wire.h>
#include "uquad_kalman.h"

#include <utility/twi.h> // For CPU_FREQ

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

#define FALSE 0
#define TRUE 1

// ADXL345 Sensitivity(from datasheet) => 4mg/LSB   1G => 1000mg/4mg = 256 steps
// Tested value : 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square


// IGT-3200 Sensitivity (from datasheet) => 14.375 LSBs/?/s
// Tested values : 
#define Gyro_Gain_X   14.375 //X axis Gyro gain
#define Gyro_Gain_Y   14.375 //Y axis Gyro gain
#define Gyro_Gain_Z   14.375 //Z axis Gyro gain


int SENSOR_SIGN[9] = { 1,1,1,1,1,1,1,1,1};  //Correct directions x,y,z - gyros, accels, magnetormeter

#define Kp_ROLLPITCH 0.0125
#define Ki_ROLLPITCH 0.000008
#define Kp_YAW 1.2
#define Ki_YAW 0.000008

//========================================
// Output Data Configuration

#define PRINT_DATA 1 // if 0 no data will be printed
//#define PRINT_GPS 0     //Will print GPS data

#define ATOMIC_IMU_FORMAT 1
#define PRINT_EULER             0   //Will print the Euler angles Roll, Pitch and Yaw
#define PRINT_SENSOR_DATA       0   //Will print the Corrected Sensor Data
#define PRINT_SENSOR_DATA_RAW   1   //Will print the raw uncorrected Sensor Data
#define PRINT_DCM               0   //Will print the whole direction cosine matrix
#define PRINT_MagCal            0

#define debugPin 6
#define STATUS_LED 4  //PD4 on the Atmega328. Red LED

//EEPROM storage addesses
#define Start_SensorOffsets    0        //offset data at start of EEPROM
#define Size_SensorOffsets     36       //the offset data is 12 bytes long 

// Sampling setting
#define SAMP_T_INTR 5000UL // us - internal sampling rate
#define SAMP_JITTER_INTR 12UL // us - timer misses by this amount
#define SAMP_INTRS_EXTR 2 //  Main loop runs at SAMP_T_INTR*SAMP_INTRS_EXTR
#define SAMP_DIV_COMPASS 20 // sampled at SAMP_INTRS_EXTR/SAMP_DIV_COMPASS
#define SAMP_DIV_BAROM 200 // sampled at SAMP_INTRS_EXTR/SAMP_DIV_BAROM
#define SAMP_T_EXTR (SAMP_T_INTR*SAMP_INTRS_EXTR) // rate at which data is sent to UART
#define SAMP_JITTER_EXTR (SAMP_T_EXTR>>5) // 4% tolerance
#define SAMP_T_EXTR_MAX (SAMP_T_EXTR + SAMP_JITTER_EXTR)
#define SAMP_T_EXTR_MIN (SAMP_T_EXTR - SAMP_JITTER_EXTR)

struct uquad_timing{
    unsigned long T_intr;
    unsigned long T_ie_ratio;
    unsigned long T_extr;
    unsigned long div_magn;
    unsigned long div_baro;
    unsigned long jitter_intr;
    unsigned long jitter_extr;
    unsigned long T_extr_max;
    unsigned long T_extr_min;
};

uquad_timing timing = {SAMP_T_INTR,
		       SAMP_INTRS_EXTR,
		       SAMP_T_EXTR,
		       SAMP_DIV_COMPASS,
		       SAMP_DIV_BAROM,
		       SAMP_JITTER_INTR,
		       SAMP_JITTER_EXTR,
		       SAMP_T_EXTR_MAX,
		       SAMP_T_EXTR_MIN};

// Special modes
#define ONLY_BMP085 0
#define MAGNETON_FULL_FS 1
#define DEBUG 0
#define WARNINGS 0
#define RELEASE 1

// Debug data
#if DEBUG
#define DEBUG_TX_TIMING 1
static bool print_raw_bmp085 = false;
#endif

// sensors
#define ALL 1
struct sensors_enabled{
    bool acc;
    bool gyro;
    bool compass;
    bool temp;
    bool pressure;
};
sensors_enabled sensors;
void sensors_enabled_set_defaults(void)
{
    sensors.acc = ALL && !ONLY_BMP085;
    sensors.gyro = ALL && !ONLY_BMP085;
    sensors.compass = ALL && !ONLY_BMP085;
    sensors.temp = ALL;
    sensors.pressure = ALL;
}

int incomingByte = 0; // for incoming serial data
bool running = true;
bool print_binary = false;

float G_Dt=0.005;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible
//float G_Dt_ms=G_Dt;
float G_Dt_us = G_Dt*1000;
#if DEBUG_TX_TIMING
float dt_tmp[SAMP_INTRS_EXTR];
int dt_tmp_index = 0;
#endif

unsigned long time_barom_us = 0;
unsigned long time_us = 0;

float AN_OFFSET[9] = {0,0,0,0,0,0,0,0,0}; //Array that stores the Offset of the sensors

//Structure for holding offsets and calibration values for the accel, gyro, and magnetom
struct s_sensor_offsets
{
    
    float gyro_offset[3];
    float accel_offset[3];
    float magnetom_offset[3];
    float magnetom_XY_Theta;
    float magnetom_XY_Scale;
    float magnetom_YZ_Theta;
    float magnetom_YZ_Scale;
    
};


//structure for holding raw and calibration corrected data from the sensors
struct s_sensor_data
{
    //raw data is uncorrected and corresponds to the
    //true sensor axis, not the redefined platform orientation
    int gyro_x_raw;
    int gyro_y_raw;
    int gyro_z_raw;
    int accel_x_raw;
    int accel_y_raw;
    int accel_z_raw;
    int magnetom_x_raw;
    int magnetom_y_raw;
    int magnetom_z_raw;
    unsigned long baro_pres_raw;
    unsigned int baro_temp_raw;
  
    //This data has been corrected based on the calibration values
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float magnetom_x;
    float magnetom_y;
    float magnetom_z;
    float magnetom_heading;
    short baro_temp;  
    long baro_pres;
};

s_sensor_offsets sen_offset = {{0,0,0},{0,0,0},{0,0,0},0,0,0,0};
s_sensor_data sen_data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Store readings to average
struct uquad_reads{
    long int acc[3];
    long int gyro[3];
    long int magnetom[3];
};

struct uquad_reads acum_reads = {{0,0,0}, {0,0,0}, {0,0,0}};

// Promed counter
int loop_counter = 0;

float Accel_Vector[3]= {0,0,0};    //Store the acceleration in a vector
float Mag_Vector[3]= {0,0,0};      //Store the magnetometer direction in a vector
float Gyro_Vector[3]= {0,0,0};     //Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0};    //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};         //Omega Proportional correction
float Omega_I[3]= {0,0,0};         //Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

float DCM_Matrix[3][3]       = {{1,0,0},{0,1,0},{0,0,1}}; 
float Update_Matrix[3][3]    = {{0,1,2},{3,4,5},{6,7,8}}; 
float Temporary_Matrix[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

void setup()
{ 
  Serial.begin(115200);
  
  struct kalman_io_t* kalman;
  kalman = kalman_init();
    
  pinMode (STATUS_LED,OUTPUT);  // Status LED
  pinMode (debugPin,OUTPUT);  // debug LED
#if !RELEASE
  Serial.println();
#if ATOMIC_IMU_FORMAT
  Serial.println("uQuad!");
  Serial.println();
  Serial.println("Output format:");
  Serial.println("A\tG_Dt\ta_x\ta_y\ta_z\tg_x\tg_y\tg_z\tm_x\tm_y\tm_z\tZ");
#else
  Serial.println("ckdevices Mongoose Base AHRS firmware v1.1");
  Serial.println("9 Degree of Freedom Attitude and Heading Reference System with barometric pressure");
  Serial.println("www.ckdevices.com");
#endif
#endif
  delay(300);

  Wire.begin();    //Init the I2C

  //================================
  // Set i2c to 400kHz
  //
  #define TWI_FREQ_NUNCHUCK 400000L
  TWBR = ((F_CPU / TWI_FREQ_NUNCHUCK) - 16) / 2;

  delay(20);
  
  //================================
  // Initialize all the IMU sensors
  //
  Init_Accel();
  Init_Compass();
  Init_Gyro();
  Init_Baro();
  
  // Start with default settings
  sensors_enabled_set_defaults();

  //===============================
  // Get the calibration value for the sensors. These are hard coded right now
  mongooseCalibrate();
  
  
  // All the offset values and calibration factors are now setup for the sensors
  
  digitalWrite(STATUS_LED,HIGH);
    

  time_us = micros();
  delay(20);
  
}
unsigned long Dt;
unsigned long loop_in_us;
unsigned long t_loop_chico;
unsigned long pezon_viejo;

void loop() //Main Loop
{
    if (Serial.available() > 0)
    {
    	// read the incoming byte:
    	incomingByte = Serial.read();
    	if (incomingByte > 0)
    	    if(menu_execute(incomingByte) < 0)
    		Serial.println("Invalid command!");
    }

    // update barom reading state machine
    barom_update_state_machine();
    loop_in_us = micros();
    Dt = loop_in_us - time_barom_us;
    if (loop_counter >= SAMP_INTRS_EXTR)
    {
	//	Serial.print("LG:");
	//	Serial.println(Dt);
	if(loop_counter != 0)
	{
	    // Average acc, gyro and compass
	    sen_data.gyro_x_raw = (int)(acum_reads.gyro[0]/loop_counter);
	    sen_data.gyro_y_raw = (int)(acum_reads.gyro[1]/loop_counter);
	    sen_data.gyro_z_raw = (int)(acum_reads.gyro[2]/loop_counter);
	    sen_data.accel_x_raw = (int)(acum_reads.acc[0]/loop_counter);
	    sen_data.accel_y_raw = (int)(acum_reads.acc[1]/loop_counter);
	    sen_data.accel_z_raw = (int)(acum_reads.acc[2]/loop_counter);
	    sen_data.magnetom_x_raw = (int)(acum_reads.magnetom[0]/loop_counter);
	    sen_data.magnetom_y_raw = (int)(acum_reads.magnetom[1]/loop_counter);
	    sen_data.magnetom_z_raw = (int)(acum_reads.magnetom[2]/loop_counter);
	    // Reset
	    loop_counter=0;
	    acum_reads.acc[0] = 0;acum_reads.acc[1] = 0;acum_reads.acc[2] = 0;
	    acum_reads.gyro[0] = 0;acum_reads.gyro[1] = 0;acum_reads.gyro[2] = 0;
	    acum_reads.magnetom[0] = 0;acum_reads.magnetom[1] = 0;acum_reads.magnetom[2] = 0;

	    //	    Dt = micros();
	    barom_update_state_machine();
	    if(sensors.pressure || sensors.temp)
		Baro_req_update();

	    /* Serial.print("B:"); */
	    /* Serial.println(micros()-Dt); */

	    printdata(); 

	    StatusLEDToggle();        
	    digitalWrite(debugPin,LOW);
	}

	time_barom_us = loop_in_us;

    }

    t_loop_chico = micros();

    if( (t_loop_chico - time_us) >= 
	SAMP_T_INTR - SAMP_JITTER_INTR )
    {

	digitalWrite(debugPin,HIGH);
	        
	//================================================================================//
	//======================  Data adquisition of all sensors ========================//
                
	//=================== Read the Gyro, Accelerometer and Compass ===================//
	if(sensors.acc)
	{
	    Read_Accel();     // Read I2C accelero#endif
	    acum_reads.acc[0] += (long int)sen_data.accel_x_raw;
	    acum_reads.acc[1] += (long int)sen_data.accel_y_raw;
	    acum_reads.acc[2] += (long int)sen_data.accel_z_raw;
	}
	barom_update_state_machine();
	if(sensors.gyro)
	{
	    Read_Gyro();      // Read the data from the I2C Gyro
	    acum_reads.gyro[0] += (long int)sen_data.gyro_x_raw;
	    acum_reads.gyro[1] += (long int)sen_data.gyro_y_raw;
	    acum_reads.gyro[2] += (long int)sen_data.gyro_z_raw;
	}
      	barom_update_state_machine();
	if(sensors.compass)
	{
	     Read_Compass();    // Read I2C magnetometer
	    acum_reads.magnetom[0] += (long int)sen_data.magnetom_x_raw;
	    acum_reads.magnetom[1] += (long int)sen_data.magnetom_y_raw;
	    acum_reads.magnetom[2] += (long int)sen_data.magnetom_z_raw;
	}

	loop_counter++;
	/* Serial.print("lc"); */
	/* Serial.println(t_loop_chico - time_us); */
	/* Serial.print("tlc"); */
	/* Serial.println(micros() - t_loop_chico); */
	time_us = t_loop_chico;
    }
} 
