/**
 ******************************************************************************
 *
 * @file       futaba_sbus.c
 * @author     Federico Favaro, Joaquin Berrutti y Lucas Falkenstein
 * @brief      Implementa la codificacion del mensaje futaba sbus en
 *             base a la informacion de los diferentes canales.
 * @see        ??
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

#include "futaba_sbus.h"
#include <uquad_aux_time.h>
#include <uquad_error_codes.h>

uint8_t sbusData[25] 	= {0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int16_t channels[18]   	= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t failsafe_status = SBUS_SIGNAL_OK;


/** 
 * Esto se tiene que ir...no hace nada
 */
#if PC_TEST
struct timeval tv_start; // Guarda el tiempo de comienzo del programa
int futaba_sbus_begin(void) {

   gettimeofday(&tv_start,NULL);

   return 0;
}
#endif //PC_TEST
 

void futaba_sbus_set_channel(uint8_t channel, int16_t value)
{
   if ((channel>0) && (channel<=16))
   {
      if (value > 2048)
         value = 2048;
      channels[channel-1] = value;
   }
}


void futaba_sbus_reset_channels()
{
   int i;
   for (i=0; i<16; i++)
      channels[i] = 0;
}


uint8_t futaba_sbus_get_failsafe(void)
{
  return failsafe_status;
}


void futaba_sbus_set_failsafe(uint8_t fs)
{
   failsafe_status = fs;
}


void futaba_sbus_update_msg(void)
{
   // Limpia mensaje anterior
   futaba_sbus_reset_msg();

   uint8_t ch = 0;
   uint8_t bit_in_servo = 0;
   uint8_t byte_in_sbus = 1;
   uint8_t bit_in_sbus = 0;

   // Convierte info de canales en mensaje sbus
   int i;
   for (i=0; i<176; i++)
   {
      if (channels[ch] & (1<<bit_in_servo))
         sbusData[byte_in_sbus] |= (1<<bit_in_sbus);
     
      bit_in_sbus++;
      bit_in_servo++;

      if (bit_in_sbus == 8)
      {
         bit_in_sbus =0;
         byte_in_sbus++;
      }
      if (bit_in_servo == 11)
      {
         bit_in_servo =0;
         ch++;
      }
   }

   // Failsafe TODO probar
   if (failsafe_status == SBUS_SIGNAL_LOST)
      sbusData[23] |= (1<<2);
   if (failsafe_status == SBUS_SIGNAL_FAILSAFE)
   {
      sbusData[23] |= (1<<2);
      sbusData[23] |= (1<<3);
   }
}


void futaba_sbus_reset_msg(void)
{
   int i;
   for (i=1; i<24; i++)
      sbusData[i] = 0;
}



/* Envia el mensaje sbus al puerto serie definido por fd. */
int futaba_sbus_write_msg(int fd)
{
   int ret = write(fd, sbusData, 25);
   if (ret < 0)
   {
      err_log("write() failed!");
      return -1;
   }

   return 0;

}

/*
 * Devuelve (al main) el pid del child process o un numero negativo en caso de error.
 * Si el child process falla al ejecutar el demonio, termina su ejecucion.
 */
int futaba_sbus_start_daemon(void)
{
   //Forks main program and starts client
   int child_pid = fork();  

   //-- -- -- El child ejecuta el siguiente codigo -- -- --
   if (child_pid == 0)
   {
      int retval;
      //starts sbus daemon
      retval = execl("./sbusd", "sbusd", START_SBUS_ARG, (char *) 0);
      //only get here if execl failed 
      if(retval < 0)
      {
         err_log_stderr("Failed to run sbusd (execl)!");
         return -1;
      }
   }

   //-- -- -- El parent (main) ejecuta el siguiente codigo -- -- --
   return child_pid;
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
#endif //PC_TEST




