/*
 * Lo tuve que hacer en una libreria separada de serial_comm porque incluir
 * los encabezados <termios.h> y <asm/termios.h> en un mismo archivo daba
 * problemas.
 */

#ifndef CUSTOM_BAUD_h
#define CUSTOM_BAUD_h

int custom_baud(int fd);

#endif


