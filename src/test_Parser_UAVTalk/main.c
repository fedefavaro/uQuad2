/**
 ******************************************************************************
 *
 * @file       main.c
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      test parser uavtalk
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
 */


#include "UAVTalk.h"
#include "OSD_Vars.h"

#include <sys/signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/ioctl.h> 
#include <linux/serial.h>
#include <termios.h>
#include <math.h>

//
#include <semaphore.h>

// shm - shared memory.
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define SHMSZ     27

#define fix(a)                    ((a>0)?floor(a):ceil(a))
#define sign(a)                   ((a < 0.0)?-1.0:1.0)

// -- -- -- -- -- -- -- -- -- 
// GLobal Vars
// -- -- -- -- -- -- -- -- --

int fd_CC3D;
int log_fd;

int shmid;
sem_t * sem_id;

// -- -- -- -- -- -- -- -- -- 
// Funciones auxiliares
// -- -- -- -- -- -- -- -- --
int uav_talk_deinit(int fd);

void quit(void)
{
   int retval;
   retval = uav_talk_deinit(fd_CC3D);
   if(retval != 0)
      puts("Could not close UAVTalk correctly!");

   // Remove shared memory
   shmctl(shmid, IPC_RMID, NULL);

   //Semaphore Close: Close a named semaphore
   if ( sem_close(sem_id) < 0 )
  	perror("sem_close");
    
   //Semaphore unlink: Remove a named semaphore  from the system.
   if ( sem_unlink("/mysem") < 0 )
    	perror("sem_unlink");
    
   /// Log
   close(log_fd);

   exit(0);

}


sigset_t mask;
sigset_t orig_mask;
void uquad_sig_handler(int signal_num)
{
   printf("Caught signal: %d\n",signal_num);
   quit();
}



void set_signals(void)
{
   //struct sigaction saio;

   // Catch signals
   signal(SIGINT,  uquad_sig_handler);
   signal(SIGQUIT, uquad_sig_handler);

}


int uquad_timeval_substract (struct timeval * result, struct timeval x, struct timeval y){
    /* Perform the carry for the later subtraction by updating y. */
    if (x.tv_usec < y.tv_usec) {
	int nsec = (y.tv_usec - x.tv_usec) / 1000000 + 1;
	y.tv_usec -= 1000000 * nsec;
	y.tv_sec += nsec;
    }
    if (x.tv_usec - y.tv_usec > 1000000) {
	int nsec = (y.tv_usec - x.tv_usec) / 1000000;
	y.tv_usec += 1000000 * nsec;
	y.tv_sec -= nsec;
    }
    
    /* Compute the time remaining to wait.
       tv_usec is certainly positive. */
    result->tv_sec = x.tv_sec - y.tv_sec;
    result->tv_usec = x.tv_usec - y.tv_usec;
    
    if(x.tv_sec < y.tv_sec)
	// -1 if diff is negative
	return -1;
    if(x.tv_sec > y.tv_sec)
	// 1 if diff is positive
	return 1;
    // second match, check usec
    if(x.tv_usec < y.tv_usec)
	// -1 if diff is negative
	return -1;
    if(x.tv_usec > y.tv_usec)
	// 1 if diff is positive
	return 1;

    // 0 if equal
    return 0;
}

static struct timeval main_start_time;

void set_main_start_time(void)
{
   gettimeofday(&main_start_time,NULL);
   return;
}


struct timeval get_main_start_time(void)
{
   return main_start_time;
}

/*********************************************/
/****************   IO    ********************/
/*********************************************/

int open_port(char *device)
{
   int fd; /* File descriptor for the port */

   fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
   if (fd == -1)
   {
      puts("Unable to open port");
   }
    
   return fd;
}


int configure_port(int fd, speed_t baudrate) //TODO cambiar nombre
{
  struct termios options;
  int rc;
 
  // get the current options for the port...
  if((rc = tcgetattr(fd, &options)) < 0){
     puts("Error al obtener atributos");
     return -1;
  }

  //set options
  cfsetispeed(&options, baudrate); 			// set the in baud rate...
  cfsetospeed(&options, baudrate);			// set the out baud rate...
  cfmakeraw(&options);
  options.c_cflag |= (CLOCAL | CREAD);			// enable the receiver and set local mode...
  
  // set the new options for the port...
  if((rc = tcsetattr(fd, TCSANOW, &options)) < 0){
     puts("Error al aplicar nuevos atributos");
     return -1;
  }
  
  return 0;

}


/* devuelve true si puedo leer, false si no puedo */
bool check_read_locks(int fd) {

   fd_set rfds;
   struct timeval tv;
   tv.tv_sec = 0;
   tv.tv_usec = 30000; //TODO detemrinar el timeout en microsegundos
   FD_ZERO(&rfds);
   FD_SET(fd, &rfds);
   int retval = select(fd+1, &rfds, NULL, NULL, &tv);
   if(retval < 0)
      puts("select() failed!");
     
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
/*********************************************/
/**************** UAVTalk ********************/
/*********************************************/

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
   /// Puerto Serie Beagle-CC3D
   int fd = open_port(CC3D_DEVICE);
   if (fd < 0)
      return -1;
   printf("CC3D conectada. fd: %d\n",fd); //dbg
   int ret = configure_port(fd, B115200);
   if (ret < 0)
      return -1;
   
   return fd;
     
}


int uav_talk_deinit(int fd)
{
   int ret = close(fd);
   if(ret < 0)
   {
      puts("No se pudo cerrar puerto. Ya esta cerrado?");
      return -1;
   }

   return 0;
}



void uav_talk_print_attitude(actitud_t act)
{
   printf("Roll: %lf  ", act.roll*180/M_PI); 
   printf("Pitch: %lf  ", act.pitch*180/M_PI);
   printf("Yaw: %lf\n", act.yaw*180/M_PI);   
}


int uavtalk_to_str(char* buf_str, actitud_t act)
{
   char* buf_ptr = buf_str;
   //int ret;
   
   // Timestamp
   buf_ptr += sprintf(buf_ptr, "%04lu %06lu", (unsigned long)act.ts.tv_sec, (unsigned long)act.ts.tv_usec);
  
   buf_ptr += sprintf(buf_ptr, " %lf", act.roll);
   buf_ptr += sprintf(buf_ptr, " %lf", act.pitch);
   buf_ptr += sprintf(buf_ptr, " %lf\n", act.yaw);
   //buf_ptr += sprintf(buf_ptr,"\t");

   return (buf_ptr - buf_str); //char_count

}

static inline float uavtalk_get_float(uavtalk_message_t *msg, int pos) {
	float f;
	memcpy(&f, msg->Data+pos, sizeof(float));
	return f;
}



uint8_t uavtalk_parse_char(uint8_t c, uavtalk_message_t *msg)
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







/*********************************************/
/**************** Main ***********************/
/*********************************************/
int main(int argc, char *argv[])
{

   int retval = 0;

   // Control de tiempos
   struct timeval ts;
   struct timeval tv_aux;

   bool CC3D_readOK;
   actitud_t act;
   bool act_updated = false;

   static double last_yaw = 0; //Para fix

   static uavtalk_message_t msg;
   uint8_t data;

   char buff_log[126]; //TODO determinar valor
   int buff_log_len;
   bool log_writeOK = false;


    typedef struct act_sdata {
	actitud_t act;
	int flag;
    } act_sdata_t;

    // shm - shared memory.
    key_t key = 5678;
    act_sdata_t *shm;

   // -- -- -- -- -- -- -- -- --
   // Inicializacion
   // -- -- -- -- -- -- -- -- --
   fd_CC3D = uav_talk_init();
   if(fd_CC3D < 0) 
   {
      puts("Failed to init UAVTalk!");
      quit();  
   } 
   
   set_signals();
  
   /// Log
   log_fd = open("log_uavtalk_parser", O_RDWR | O_CREAT | O_NONBLOCK );
   if(log_fd < 0)
   {
      puts("Failed to open log file!");
      quit();
   }

    // Semaphore open
    sem_id=sem_open("/mysem", O_CREAT, S_IRUSR | S_IWUSR, 1);

    //Create the segment.
    if ((shmid = shmget(key, SHMSZ, IPC_CREAT | 0666)) < 0) {
        perror("shmget");
        exit(1);
    }

    //Now we attach the segment to our data space.
    if ((shm = shmat(shmid, NULL, 0)) == (void *) -1) {
        perror("shmat");
        exit(1);
    }

    // init flag
    shm->flag = 0;


   // -- -- -- -- -- -- -- -- -- 
   // Loop
   // -- -- -- -- -- -- -- -- -- 
   for(;;)
   {
	
        CC3D_readOK = check_read_locks(fd_CC3D);
	if (CC3D_readOK) {
           retval = read(fd_CC3D,&data,1);
	   if (retval <= 0) {
	      puts("read failed");
    	   } else {
              retval = uavtalk_parse_char(data, &msg);
	      if (retval > 0) {
		   // consume msg
		   switch (msg.ObjID) {
			case ATTITUDEACTUAL_OBJID:
			case ATTITUDESTATE_OBJID:
        		   act.roll  = uavtalk_get_float(&msg, ATTITUDEACTUAL_OBJ_ROLL)*M_PI/180;
			   act.pitch = uavtalk_get_float(&msg, ATTITUDEACTUAL_OBJ_PITCH)*M_PI/180;
			   act.yaw   = uavtalk_get_float(&msg, ATTITUDEACTUAL_OBJ_YAW)*M_PI/180;
				   
			   //Correccion de discontinuidad de atan2
			   double dyaw = act.yaw - last_yaw;
			   if (abs(dyaw) >= M_PI)
				act.yaw -= 2.0*M_PI*fix((dyaw+M_PI*sign(dyaw))/(2.0*M_PI));				   last_yaw = act.yaw;
                                  
			   // Timestamp
			   gettimeofday(&tv_aux,NULL);
                           uquad_timeval_substract(&act.ts, tv_aux, get_main_start_time());
   			   if(retval < 0) puts("WARN: Absurd timing!");
			   act_updated = true;
			   break;
		   }
	      }
	   }
	   if(act_updated)
	   {
	      sem_wait(sem_id);
	      shm->act = act;
	      if(shm->flag > 0) {
		 shm->flag = 2;
	      } else shm->flag = 1;
	      sem_post(sem_id);
	      
	      //uav_talk_print_attitude(act);
	      buff_log_len = uavtalk_to_str(buff_log, act);
	      log_writeOK = check_write_locks(log_fd);
	      if (log_writeOK) {
	          retval = write(log_fd, buff_log, buff_log_len);
	          if(retval < 0)
		     puts("Failed to write to log file!");
	       }


	      act_updated = false;
	   }
	}
        
   } // for(;;)

   return 0; //nunca llego aca

} //FIN MAIN


