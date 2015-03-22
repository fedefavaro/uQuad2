#ifndef SERIAL_PORT_h
#define SERIAL_PORT_h

/*
 * open_port(device) - Open serial port on device
 *
 * Returns the file descriptor on success or -1 on error.
 */
int open_port(char *device);

/*
 * configure_port() - Configure serial port
 *
 */
int configure_port(int fd);

#endif


