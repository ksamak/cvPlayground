#include "serial.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>

struct timeval t0, t1;

/*
 * The serial connection.
 */
int serial;

/*
 * The baud rate in bps.
 */
#define BAUDRATE B230400

/*
 * Connect to a serial port.
 */
int lin_serial_connect(const char* portname, int speed, int blocking, int parity) {
    struct termios options;
    int ret = 0;

    printf("connecting to %s\n", portname);

    if ((serial = open(portname, O_RDWR | O_NOCTTY/* | O_NDELAY*/)) < 0)
    {
        printf("can't connect to %s\n", portname);
        ret = -1;
    }
    else
    {
        tcgetattr(serial, &options);
        cfsetispeed(&options, BAUDRATE);
        cfsetospeed(&options, BAUDRATE);
        options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        options.c_lflag = 0;                // no signaling chars, no echo,
        // no canonical processing
        options.c_oflag = 0;                // no remapping, no delays
        options.c_cc[VMIN]  = 0;            // read doesn't block
        options.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        options.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        options.c_cflag |= parity;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_oflag &= ~OPOST;
        if(tcsetattr(serial, TCSANOW, &options) < 0)
        {
            printf("can't set serial port options\n");
            ret = -1;
        }
        else
        {
            printf("connected to %s\n", portname);
        }
    }

    return ret;
}

/*
 * Send a usb report to the serial port.
 */
int lin_serial_send(void* pdata, unsigned int size) {
    return write(serial, (unsigned char*)pdata, size);
}

void lin_serial_close()
{
    close(serial);
}

int set_interface_attribs (int fd, int speed, int parity) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // ignore break signal
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        return -1;
    }
    return 0;
}

// "Blocking" sets whether a read() on the port waits for the specified number of characters to arrive. Setting no blocking means that a read() returns however many characters are available without waiting for more, up to the buffer limit.
void set_blocking (int fd, int should_block) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        return;
    }
}
