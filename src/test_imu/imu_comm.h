#ifndef IMU_COMM_H
#define IMU_COMM_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "uquad_aux_math.h"
#include "uquad_aux_time.h"

#define RX_IMU_BUFFER_SIZE 31           // Tamaño del buffer de recepcion
#define IMU_BYTES_T_US    4             // Tamaño del tiempo recibido por la IMU en formato binario


/**
 * Datos crudos de la IMU
 * //Contiene 29 bytes, 27 utiles mas init/end.
 * Contiene 31 bytes, 29 utiles mas init/end.
 *
 *   [1b:Init char]
 *   [4b:T        ]
 *   [2b:Acc X    ]
 *   [2b:Acc Y    ]
 *   [2b:Acc Z    ]
 *   [2b:Gyro X   ]
 *   [2b:Gyro Y   ]
 *   [2b:Gyro Z   ]
 *   [2b:Magn X   ]
 *   [2b:Magn Y   ]
 *   [2b:Magn Z   ]
 *   [1b:Temp     ]
 *   [4b:Press    ]
 *   [2b:US_obs   ]
 *   [1b:End char ]
 * 
 * NOTA: No hay caracter de separacion.
 */
typedef struct imu_frame {
    uint32_t T_us;   // Time since previous sample in us
    int16_t acc [3]; // ADC counts
    int16_t gyro[3]; // ADC counts
    int16_t magn[3]; // ADC counts
    uint16_t temp;   // Tens of °C
    uint32_t pres;   // Pa
    int16_t us_obstacle; // ADC counts // cm?
} imu_raw_t;


/**
 * imu_data_t: Stores calibrated data. This is the final output of imu_comm,
 * and should be passed on the the control loop.
 */
typedef struct imu_data {
    double T_us;         // us
    uquad_mat_t *acc;    // m/s^2
    uquad_mat_t *gyro;   // rad/s
    uquad_mat_t *magn;   // rad - Euler angles - {psi/roll,phi/pitch,theta/yaw}
    double temp;         // °C
    double alt;          // m
    double us_obstacle;   // m
    //uquad_bool_t acc_ok; // acc norm() in range
    //uquad_bool_t magn_ok;// magn norm() in range
} imu_data_t;



bool check_read_locks(int fd);

int write_RX_imu_buffer(unsigned char c);

bool RX_imu_isFull(void);

bool RX_imu_isEmpty(void);

bool check_imu_ready(void);

void reset_imu_ready(void);

void print_imu_buffer(void);

void imu_comm_parse_frame_binary(imu_raw_t *frame);//, unsigned char *data);

void print_imu_raw(imu_raw_t *frame);

void acc_calib_init(void);
void gyro_calib_init(void);
void magn_calib_init(void);
void pres_calib_init(void);

void temp_raw2data(imu_raw_t *raw, imu_data_t *data);
void acc_raw2data(imu_raw_t *raw, imu_data_t *data);
void gyro_raw2data(imu_raw_t *raw, imu_data_t *data);
void magn_raw2data(imu_raw_t *raw, imu_data_t *data);
void pres_raw2data(imu_raw_t *raw, imu_data_t * data);
void imu_raw2data(imu_raw_t *raw, imu_data_t *data);

void imu_data_alloc(imu_data_t *imu_data);

void printDouble( double val, unsigned int precision);


#endif
