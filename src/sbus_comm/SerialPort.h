#ifndef SERIAL_PORT_h
#define SERIAL_PORT_h

/*
 * open_port() - Open serial port
 *
 * Returns the file descriptor on success or -1 on error.
 */
int open_port(void);

/*
 * configure_port() - Configure serial port
 *
 */
int configure_port(int fd);

#endif


