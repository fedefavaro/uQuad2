/**
 ******************************************************************************
 *
 * @file       UAVTalk.h
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

#include "UAVTalk.h"
#include "OSD_Vars.h"

#include "uquad_aux_time.h"
#include "quadcop_config.h"
#include "serial_comm.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

//#define CC3D_DEVICE	"/dev/ttyUSB0" //TODO que onda cuando tenga 2 ftdi?
#define CC3D_DEVICE	"/dev/ttyO1"

//#define CC3D_BAUD_57600
#define CC3D_BAUD_115200

static unsigned long last_gcstelemetrystats_send = 0;
static unsigned long last_flighttelemetry_connect = 0;
static uint8_t gcstelemetrystatus = TELEMETRYSTATS_STATE_DISCONNECTED;

static uint32_t gcstelemetrystats_objid = GCSTELEMETRYSTATS_OBJID_001;
static uint8_t gcstelemetrystats_obj_len = GCSTELEMETRYSTATS_OBJ_LEN_001;
static uint8_t gcstelemetrystats_obj_status = GCSTELEMETRYSTATS_OBJ_STATUS_001;
static uint8_t flighttelemetrystats_obj_status = FLIGHTTELEMETRYSTATS_OBJ_STATUS_001;


// CRC lookup table
static const uint8_t crc_table[256] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};


int uav_talk_init(void)
{
   //uav_talk_get_start_time();
   
   /// Puerto Serie Beagle-CC3D
   int fd = open_port(CC3D_DEVICE);
   if (fd < 0)
      return -1;
   printf("CC3D conectada. fd: %d\n",fd); //dbg
#ifdef CC3D_BAUD_115200
   int ret = configure_port_gps(fd, B115200); //TODO cambiar nombre
#else //CC3D_BAUD_57600
   int ret = configure_port_gps(fd, B57600); //TODO cambiar nombre
#endif
   if (ret < 0)
      return -1;

   // Clean serial buffer from cc3d
   serial_flush(fd);
   
   return fd;
     
}


int uav_talk_deinit(int fd)
{
   int ret = close(fd);
   if(ret < 0)
   {
      printf("No se pudo cerrar puerto. Ya esta cerrado?\n");
      return -1;
   }

   return 0;
}


int32_t uavtalk_get_time_usec(void)
{
	struct timeval tv_diff;
	struct timeval tv_aux;

        gettimeofday(&tv_aux,NULL);
        uquad_timeval_substract(&tv_diff, tv_aux, get_main_start_time());
      	return (int32_t)(tv_diff.tv_sec * 1000000 + tv_diff.tv_usec);
}


void uav_talk_print_attitude(actitud_t act)
{
   printf("Roll: %lf  ", act.roll); 
   printf("Pitch: %lf  ", act.pitch);
   printf("Yaw: %lf  ", act.yaw);
   printf("Time: %04lu:%06lu\n", (unsigned long)act.ts.tv_sec, (unsigned long)act.ts.tv_usec);
   
}

/* devuelve true si puedo leer, false si no puedo */
bool check_read_locks(int fd) {

  fd_set rfds;
  struct timeval tv;
  tv.tv_sec = 0;
#ifdef CC3D_BAUD_115200
//  tv.tv_usec = 100;   //Espero un byte: T_byte=1/(BAUDRATE/10)
  tv.tv_usec = 0;
#else //#ifdef CC3D_BAUD_57600
//  tv.tv_usec = 175;
  tv.tv_usec = 0;
#endif  
   FD_ZERO(&rfds);
   FD_SET(fd, &rfds);
   int retval = select(fd+1, &rfds, NULL, NULL, &tv);
   if(retval < 0)
      printf("select() failed!\n");
     
   return FD_ISSET(fd,&rfds);
}


bool check_write_locks(int fd) {

  fd_set wfds;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  
   FD_ZERO(&wfds);
   FD_SET(fd, &wfds);
   int retval = select(fd+1, NULL, &wfds, NULL, &tv);
   if(retval < 0)
      printf("select() failed!\n");
     
   return FD_ISSET(fd,&wfds);
}

static inline int8_t uavtalk_get_int8(uavtalk_message_t *msg, int pos) {
	return msg->Data[pos];
}


static inline int16_t uavtalk_get_int16(uavtalk_message_t *msg, int pos) {
	int16_t i;
	memcpy(&i, msg->Data+pos, sizeof(int16_t));
	return i;
}


static inline int32_t uavtalk_get_int32(uavtalk_message_t *msg, int pos) {
	int32_t i;
	memcpy(&i, msg->Data+pos, sizeof(int32_t));
	return i;
}


static inline float uavtalk_get_float(uavtalk_message_t *msg, int pos) {
	float f;
	memcpy(&f, msg->Data+pos, sizeof(float));
	return f;
}


void uavtalk_respond_object(int fd, uavtalk_message_t *msg_to_respond, uint8_t type) {
	uavtalk_message_t msg;
	
	msg.Sync	= UAVTALK_SYNC_VAL;
	msg.MsgType	= type;
	msg.Length	= RESPOND_OBJ_LEN;
	msg.ObjID	= msg_to_respond->ObjID;
	
	uavtalk_send_msg(fd,&msg);
	//printf("RESPOND OBJECT\n");
}


void uavtalk_send_gcstelemetrystats(int fd)
{
	uint8_t *d;
	uint8_t i;
	uavtalk_message_t msg;
	
	msg.Sync	= UAVTALK_SYNC_VAL;
	msg.MsgType	= UAVTALK_TYPE_OBJ_ACK;
	msg.Length	= gcstelemetrystats_obj_len + HEADER_LEN;
	msg.ObjID	= gcstelemetrystats_objid;

	d = msg.Data;
	for (i=0; i<gcstelemetrystats_obj_len; i++) {
		*d++ = 0;
	}

	msg.Data[gcstelemetrystats_obj_status] = gcstelemetrystatus;
	// remaining data unused and unset
	
	uavtalk_send_msg(fd,&msg);
	//last_gcstelemetrystats_send = millis();
	last_gcstelemetrystats_send =  uavtalk_get_time_usec();
	//printf("SEND GCSTELEMETRY\n");
}


void uavtalk_request_object(int fd, uint32_t id) {
	uavtalk_message_t msg;
	
	msg.Sync	= UAVTALK_SYNC_VAL;
	msg.MsgType	= UAVTALK_TYPE_OBJ_REQ;
	msg.Length	= REQUEST_OBJ_LEN;
	msg.ObjID	= id;
	uavtalk_print_msg(&msg);	
	uavtalk_send_msg(fd, &msg);
}


void uavtalk_send_msg(int fd, uavtalk_message_t *msg) {
	uint8_t *d;
	uint8_t j;
	uint8_t c;

	//char buff[(msg->Length & 0xff) + 20];
	uint8_t buff[300];
	int i = 0;

	c = (uint8_t) (msg->Sync);
	buff[i]=c;
	msg->Crc = crc_table[0 ^ c];
	
	c = (uint8_t) (msg->MsgType);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	
	c = (uint8_t) (msg->Length & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];

	c = (uint8_t) ((msg->Length >> 8) & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];

	c = (uint8_t) (msg->ObjID & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];

	c = (uint8_t) ((msg->ObjID >> 8) & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];

	c = (uint8_t) ((msg->ObjID >> 16) & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];

	c = (uint8_t) ((msg->ObjID >> 24) & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	
	c = 0; //(uint8_t) (msg->InstID & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	
	c = 0; //(uint8_t) ((msg->InstID >> 8) & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
        
	if (msg->Length > HEADER_LEN) {
	  d = msg->Data;
	  for (j=0; j<msg->Length-HEADER_LEN; j++) {
		c = *d++;
		buff[++i]=c;
		msg->Crc = crc_table[msg->Crc ^ c];
          }
	}
	
	buff[++i]=msg->Crc;
	if(!check_write_locks(fd))
	{
		printf("Unable to write, will lock\n");
		return;
	}
	int ret = write(fd,buff,i+1);
	if (ret < i+1)
	  printf("write failed. chars written %d/%d\n",ret,i);

	return;

}


uint8_t uavtalk_parse_char(uint8_t c, uavtalk_message_t *msg, int fd)
{
	static uint8_t status = UAVTALK_PARSE_STATE_WAIT_SYNC;
	static uint8_t crc = 0;
	static uint8_t cnt = 0;

	switch (status) {
		case UAVTALK_PARSE_STATE_WAIT_SYNC:
			if (c == UAVTALK_SYNC_VAL) {
				//got_sync:;
                                status = UAVTALK_PARSE_STATE_GOT_SYNC;
				msg->Sync = c;
				crc = crc_table[0 ^ c];
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_SYNC:
			crc = crc_table[crc ^ c];
			if ((c & UAVTALK_TYPE_MASK) == UAVTALK_TYPE_VER) {
				status = UAVTALK_PARSE_STATE_GOT_MSG_TYPE;
				msg->MsgType = c;
				cnt = 0;
			}
			else {
				status = UAVTALK_PARSE_STATE_WAIT_SYNC;
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_MSG_TYPE:
			crc = crc_table[crc ^ c];
			cnt++;
			if (cnt < 2) {
				msg->Length = ((uint16_t) c);
			}
			else {
				msg->Length += ((uint16_t) c) << 8;
                                if ((msg->Length < HEADER_LEN) || (msg->Length > 255 + HEADER_LEN)) {
                                       // Drop corrupted messages:
                                       // Minimal length is HEADER_LEN
                                       // Maximum is HEADER_LEN + 255 (Data) + 2 (Optional Instance Id)
                                       // As we are not parsing Instance Id, 255 is a hard maximum. 
				       status = UAVTALK_PARSE_STATE_WAIT_SYNC;
                                } else {
				       status = UAVTALK_PARSE_STATE_GOT_LENGTH;
				       cnt = 0;
                                }
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_LENGTH:
			crc = crc_table[crc ^ c];
			cnt++;
			switch (cnt) {
				case 1:
					msg->ObjID = ((uint32_t) c);
				break;
				case 2:
					msg->ObjID += ((uint32_t) c) << 8;
				break;
				case 3:
					msg->ObjID += ((uint32_t) c) << 16;
				break;
				case 4:
					msg->ObjID += ((uint32_t) c) << 24;
					status = UAVTALK_PARSE_STATE_GOT_OBJID;
					cnt = 0;

/*                                      // Agregado por mi
                                        // descarto cualquier mensaje que no sea actitud
                                        if (msg->ObjID != ATTITUDEACTUAL_OBJID && 
                                            msg->ObjID != ATTITUDESTATE_OBJID)
                                        {
                                           while(read(fd,&c,1) > 0)
                                           {
					      if (c == UAVTALK_SYNC_VAL) {                                                  
                                                 goto got_sync;
					      }
					   } 
					   status = UAVTALK_PARSE_STATE_WAIT_SYNC;
                                           return -1;
                                        }
*/

				break;
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_OBJID:
			crc = crc_table[crc ^ c];
			cnt++;
			switch (cnt) {
				case 1:
					msg->InstID = ((uint32_t) c);
				break;
				case 2:
					msg->InstID += ((uint32_t) c) << 8;
					if (msg->Length == HEADER_LEN) { // no data exists
						status = UAVTALK_PARSE_STATE_GOT_DATA;
					} else {
						status = UAVTALK_PARSE_STATE_GOT_INSTID;
					}
					cnt = 0;
				break;
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_INSTID:
			crc = crc_table[crc ^ c];
			cnt++;
			msg->Data[cnt - 1] = c;
			if (cnt >= msg->Length - HEADER_LEN) {
				status = UAVTALK_PARSE_STATE_GOT_DATA;
				cnt = 0;
			}
		break;
		case UAVTALK_PARSE_STATE_GOT_DATA:
			msg->Crc = c;
			status = UAVTALK_PARSE_STATE_GOT_CRC;
		break;
	}
	
	if (status == UAVTALK_PARSE_STATE_GOT_CRC) {
		status = UAVTALK_PARSE_STATE_WAIT_SYNC;
		if (crc == msg->Crc) {
			return msg->Length;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
}



int uavtalk_read(int fd, actitud_t* act)
{
	int ret = 0;  
        struct timeval tv_aux;
	int32_t start_time = uavtalk_get_time_usec();
	
        static uavtalk_message_t msg;
	uint8_t show_prio_info = 0;
	uint8_t c;

	// grabbing data
	while (!show_prio_info && check_read_locks(fd))
	{				
		ret = read(fd,&c,1);
		if (ret <= 0) {
		  err_log("read failed");
    		  return -1;
		}

		// parse data to msg
                ret = uavtalk_parse_char(c, &msg, fd);
		if (ret > 0) {
			//uavtalk_print_msg(&msg);
			// consume msg
			switch (msg.ObjID) {

				case FLIGHTTELEMETRYSTATS_OBJID:
				case FLIGHTTELEMETRYSTATS_OBJID_001:

					switch (msg.Data[flighttelemetrystats_obj_status]) {
						case TELEMETRYSTATS_STATE_DISCONNECTED:
							gcstelemetrystatus = TELEMETRYSTATS_STATE_HANDSHAKEREQ;
							uavtalk_send_gcstelemetrystats(fd);
							printf("HANDSHAKEREQ\n");
						break;
						case TELEMETRYSTATS_STATE_HANDSHAKEACK:
							gcstelemetrystatus = TELEMETRYSTATS_STATE_CONNECTED;
							uavtalk_send_gcstelemetrystats(fd);
							printf("CONNECTED\n");
						break;
						case TELEMETRYSTATS_STATE_CONNECTED:
							gcstelemetrystatus = TELEMETRYSTATS_STATE_CONNECTED;
							last_flighttelemetry_connect = uavtalk_get_time_usec();
							printf("CONNECTED 2\n");
						break;
					}
				break;



				case ATTITUDEACTUAL_OBJID:
				case ATTITUDESTATE_OBJID:
				   show_prio_info = 1;
        			   act->roll  = uavtalk_get_float(&msg, ATTITUDEACTUAL_OBJ_ROLL);
				   act->pitch = uavtalk_get_float(&msg, ATTITUDEACTUAL_OBJ_PITCH);
				   act->yaw   = uavtalk_get_float(&msg, ATTITUDEACTUAL_OBJ_YAW);
                                   // Timestamp
				   gettimeofday(&tv_aux,NULL);
                                   uquad_timeval_substract(&act->ts, tv_aux, get_main_start_time());
				   
/*                                   //chequeo si muestra anterior no fue hace mucho
                                   ret = uquad_timeval_substract(&tv_aux, act.ts, act_buffer[NUM_PROMEDIOS].ts);
                                   if (ret < 0)
                                   {
					err_log("WARN: Absurd timing!");
                                   } else {
     					if (tv_aux.tv_usec > 80000UL) {
					   err_log("WARN: se perdieron muestras");
					   actitud_t act_aux = act_buffer[NUM_PROMEDIOS];
					   /// TODO si se hace derivada con tiempo real hay que inventar un tiempo para repetir la muestrra anterior.
					   add_to_buff(act_aux);
					} else {
					   add_to_buff(act);
					}
      				   }
*/				   
				   //serial_flush(fd);
				   //while(read(fd,&c,1) > 0);
				   printf("ATTITUDE\n");
				   
				   break;
			}
			if (msg.MsgType == UAVTALK_TYPE_OBJ_ACK) {
				uavtalk_respond_object(fd,&msg, UAVTALK_TYPE_ACK);
			}

		}

        } //while()
	
#ifdef DEBUG
        //uavtalk_print_msg(&msg);
        //uav_talk_print_attitude(act);
#endif

	// check connect timeout
	int32_t current_time_usec = uavtalk_get_time_usec();
	if (last_flighttelemetry_connect + FLIGHTTELEMETRYSTATS_CONNECT_TIMEOUT < current_time_usec)
	{
		gcstelemetrystatus = TELEMETRYSTATS_STATE_DISCONNECTED;
		show_prio_info = 1;
	}
	
	// periodically send gcstelemetrystats
	if (last_gcstelemetrystats_send + 1000*GCSTELEMETRYSTATS_SEND_PERIOD < current_time_usec)
	{
		uavtalk_send_gcstelemetrystats(fd);
	}


        
	return show_prio_info;
}



int uavtalk_state(void)
{
	return gcstelemetrystatus;
}



int uavtalk_to_str(char* buf_str, actitud_t act)
{
   char* buf_ptr = buf_str;
   //int ret;
   
   // Timestamp
   buf_ptr += sprintf(buf_ptr, "%04lu %06lu", (unsigned long)act.ts.tv_sec, (unsigned long)act.ts.tv_usec);
  
   buf_ptr += sprintf(buf_ptr, " %lf", act.roll);
   buf_ptr += sprintf(buf_ptr, " %lf", act.pitch);
   buf_ptr += sprintf(buf_ptr, " %lf ", act.yaw);
   //buf_ptr += sprintf(buf_ptr,"\t");

   return (buf_ptr - buf_str); //char_count

}


void uavtalk_print_msg(uavtalk_message_t *msg)
{
	uint8_t *d;
	uint8_t j;
	uint8_t c;

	//char buff[(msg->Length & 0xff) + 20];
	uint8_t buff[300];
	int i = 0;
	c = (uint8_t) (msg->Sync);
	buff[i]=c;
	msg->Crc = crc_table[0 ^ c];
	c = (uint8_t) (msg->MsgType);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) (msg->Length & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) ((msg->Length >> 8) & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) (msg->ObjID & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) ((msg->ObjID >> 8) & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) ((msg->ObjID >> 16) & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	c = (uint8_t) ((msg->ObjID >> 24) & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	c = 0; //(uint8_t) (msg->InstID & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
	c = 0; //(uint8_t) ((msg->InstID >> 8) & 0xff);
	buff[++i]=c;
	msg->Crc = crc_table[msg->Crc ^ c];
    	if (msg->Length > HEADER_LEN) {
	  d = msg->Data;
	  for (j=0; j<msg->Length-HEADER_LEN; j++) {
		c = *d++;
		buff[++i]=c;
		msg->Crc = crc_table[msg->Crc ^ c];
          }
	}
	buff[++i]=msg->Crc;

        int k;
        for (k=0;k<i+1;k++)
           printf("%02X ", buff[k]);
	printf("\n");

	return;
}





