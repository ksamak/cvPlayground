#ifndef PLAYGROUND_SERIAL
#define PLAYGROUND_SERIAL

#include <termios.h>

int lin_serial_connect(const char* portname, int speed = B230400, int blocking = 1, int parity = 0);

// speeds are B115200, B230400, B9600, B19200, B38400, B57600, B1200, B2400, B4800
// The values for parity are 0 (meaning no parity), PARENB|PARODD (enable parity and use odd),
// PARENB (enable parity and use even), PARENB|PARODD|CMSPAR (mark parity), and PARENB|CMSPAR (space parity).
int set_interface_attribs(int fd, int speed = B9600, int parity = 0);

void set_blocking(int fd, int should_block);

int lin_serial_send(void* pdata, unsigned int size);

#endif
