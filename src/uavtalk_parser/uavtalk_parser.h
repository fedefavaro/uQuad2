/**
 ******************************************************************************
 *
 * @file       uavtalk_parser.h
 * @author     Joerg-D. Rothfuchs
 * @brief      Implements a subset of the telemetry communication between
 * 	       OpenPilot CC, CC3D, Revolution and Ardupilot Mega MinimOSD
 * 	       with code from OpenPilot and MinimOSD.
 * @see        The GNU Public License (GPL) Version 3
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
 *
 * Modificado por: Federico Favaro, Joaquin Berrutti, Lucas Falkenstein
 */


#ifndef UAVTALK_PARSER_H_
#define UAVTALK_PARSER_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Version number, incrementing this will erase/upload factory settings.
// Only devs should increment this
#define VER 75

// Version
#define VERSION_RELEASE_15_02_1		// OpenPilot-RELEASE 15.02.1	'Ragin' Cajun' .1


#define	FLIGHTTELEMETRYSTATS_OBJID			0x2F7E2902
#define	GCSTELEMETRYSTATS_OBJID				0xABC72744

#define	ATTITUDEACTUAL_OBJID				0x33DAD5E6
#define ATTITUDESTATE_OBJID				0xD7E0D964	// new name since VERSION_RELEASE_14_01_1

#define	FLIGHTSTATUS_OBJID				0x9B6A127E

#define	MANUALCONTROLCOMMAND_OBJID			0x1E82C2D2

#define SYSTEMALARMS_OBJID				0x7BD9C77A


#define	FLIGHTTELEMETRYSTATS_OBJ_LEN			21
#define	FLIGHTTELEMETRYSTATS_OBJ_STATUS			20
#define	FLIGHTTELEMETRYSTATS_OBJ_LEN_001		37		// different since VERSION_RELEASE_14_01_1
#define	FLIGHTTELEMETRYSTATS_OBJ_STATUS_001		36		// different since VERSION_RELEASE_14_01_1

#define	GCSTELEMETRYSTATS_OBJ_LEN			21
#define	GCSTELEMETRYSTATS_OBJ_STATUS			20
#define	GCSTELEMETRYSTATS_OBJ_LEN_001			37		// different since VERSION_RELEASE_14_01_1
#define	GCSTELEMETRYSTATS_OBJ_STATUS_001		36		// different since VERSION_RELEASE_14_01_1

#define	ATTITUDEACTUAL_OBJ_ROLL				16
#define	ATTITUDEACTUAL_OBJ_PITCH			20
#define	ATTITUDEACTUAL_OBJ_YAW				24

#define	FLIGHTSTATUS_OBJ_ARMED				0
#define	FLIGHTSTATUS_OBJ_FLIGHTMODE			1

#define	MANUALCONTROLCOMMAND_OBJ_THROTTLE		0
#define	MANUALCONTROLCOMMAND_OBJ_CHANNEL_0		24
#define	MANUALCONTROLCOMMAND_OBJ_CHANNEL_1		26
#define	MANUALCONTROLCOMMAND_OBJ_CHANNEL_2		28
#define	MANUALCONTROLCOMMAND_OBJ_CHANNEL_3		30
#define	MANUALCONTROLCOMMAND_OBJ_CHANNEL_4		32
#define	MANUALCONTROLCOMMAND_OBJ_CHANNEL_5		34
#define	MANUALCONTROLCOMMAND_OBJ_CHANNEL_6		36
#define	MANUALCONTROLCOMMAND_OBJ_CHANNEL_7		38
#define	MANUALCONTROLCOMMAND_OBJ_CHANNEL_8		40


#define	SYSTEMALARMS_ALARM_CPUOVERLOAD			4
#define	SYSTEMALARMS_ALARM_EVENTSYSTEM			5
#define	SYSTEMALARMS_ALARM_MANUALCONTROL		8


#define FLIGHTSTATUS_OBJID_001				0x0ED79A04	// different ID for unreleased next version
#define FLIGHTSTATUS_OBJID_002				0x1B7AEB74	// different ID for unreleased next version and VERSION_RELEASE_13_06_1
#define FLIGHTSTATUS_OBJID_003				0x0B37AA16	// different ID for VERSION_RELEASE_13_06_2
#define FLIGHTSTATUS_OBJID_004				0xC5FF2D54	// different ID for VERSION_RELEASE_14_06_1 and VERSION_RELEASE_14_10_1
#define FLIGHTSTATUS_OBJID_005				0x8A80EA52	// different ID for VERSION_RELEASE_15_01_1 and VERSION_RELEASE_15_02_1

#define SYSTEMALARMS_OBJID_001				0x09C7CBFE	// different ID for unreleased next version
#define SYSTEMALARMS_OBJID_002				0x1D70DB44	// different ID for unreleased next version and VERSION_RELEASE_13_06_1
#define SYSTEMALARMS_OBJID_003				0x34EEACF8	// different ID for VERSION_RELEASE_14_01_1
#define SYSTEMALARMS_OBJID_004				0xBA9B00A4	// different ID for VERSION_RELEASE_14_06_1 and VERSION_RELEASE_15_01_1
#define SYSTEMALARMS_OBJID_005				0x6B7639EC	// different ID for VERSION_RELEASE_15_02_1

#define FLIGHTTELEMETRYSTATS_OBJID_001			0x6737BB5A	// different ID for VERSION_RELEASE_14_01_1 and VERSION_RELEASE_14_06_1 and VERSION_RELEASE_14_10_1 and VERSION_RELEASE_15_01_1 and VERSION_RELEASE_15_02_1

#define GCSTELEMETRYSTATS_OBJID_001 			0xCAD1DC0A	// different ID for VERSION_RELEASE_14_01_1 and VERSION_RELEASE_14_06_1 and VERSION_RELEASE_14_10_1 and VERSION_RELEASE_15_01_1 and VERSION_RELEASE_15_02_1

#define MANUALCONTROLCOMMAND_OBJID_001			0xB8C7F78A	// different ID for VERSION_RELEASE_14_01_1
#define MANUALCONTROLCOMMAND_OBJID_002			0x161A2C98	// different ID for VERSION_RELEASE_14_06_1 and VERSION_RELEASE_14_10_1 and VERSION_RELEASE_15_01_1 and VERSION_RELEASE_15_02_1


#define	FLIGHTTELEMETRYSTATS_CONNECT_TIMEOUT		10000
#define	GCSTELEMETRYSTATS_SEND_PERIOD			1000

#define HEADER_LEN                                      10

#define	RESPOND_OBJ_LEN					HEADER_LEN
#define	REQUEST_OBJ_LEN					HEADER_LEN

#define UAVTALK_SYNC_VAL				0x3C

#define UAVTALK_TYPE_MASK				0xF8
#define UAVTALK_TYPE_VER				0x20

#define UAVTALK_TYPE_OBJ				(UAVTALK_TYPE_VER | 0x00)
#define UAVTALK_TYPE_OBJ_REQ				(UAVTALK_TYPE_VER | 0x01)
#define UAVTALK_TYPE_OBJ_ACK				(UAVTALK_TYPE_VER | 0x02)
#define UAVTALK_TYPE_ACK				(UAVTALK_TYPE_VER | 0x03)
#define UAVTALK_TYPE_NACK				(UAVTALK_TYPE_VER | 0x04)

#define CC3D_DEVICE	"/dev/ttyO1"
#define SHMSZ     27

typedef enum {
	UAVTALK_PARSE_STATE_WAIT_SYNC = 0,
	UAVTALK_PARSE_STATE_GOT_SYNC,
	UAVTALK_PARSE_STATE_GOT_MSG_TYPE,
	UAVTALK_PARSE_STATE_GOT_LENGTH,
	UAVTALK_PARSE_STATE_GOT_OBJID,
	UAVTALK_PARSE_STATE_GOT_INSTID,
	UAVTALK_PARSE_STATE_GOT_DATA,
	UAVTALK_PARSE_STATE_GOT_CRC
} uavtalk_parse_state_t;


typedef enum {
	TELEMETRYSTATS_STATE_DISCONNECTED = 0,
	TELEMETRYSTATS_STATE_HANDSHAKEREQ,
	TELEMETRYSTATS_STATE_HANDSHAKEACK,
	TELEMETRYSTATS_STATE_CONNECTED
} telemetrystats_state_t;


typedef struct __uavtalk_message {
	uint8_t Sync;
	uint8_t MsgType;
	uint16_t Length;
	uint32_t ObjID;
	uint16_t InstID;
	uint8_t Data[255];
	uint8_t Crc;
} uavtalk_message_t;


typedef struct actitud {
	double roll;
	double pitch;
	double yaw;
	struct timeval ts;
} actitud_t;


/*
 * Estructura para almacenar valor de actitud en memoria compartida
 * entre el main y el proceso uavtalk_parser
 *
 * El valor del flag significa:
 * 0 - No hay dato nuevo
 * 1 - Hay un dato nuevo
 * 2 - Se sobreescribio un dato no leido
 */
typedef struct act_sdata {
	actitud_t act;
	int flag;
} act_sdata_t;

int uavtalk_parser_start(struct timeval main_start);
void uavtalk_print_attitude(actitud_t act);
int uavtalk_to_str(char* buf_str, actitud_t act);
int uavtalk_init_shm(void);
int uavtalk_read(actitud_t *act);

#endif /* UAVTALK_H_ */

