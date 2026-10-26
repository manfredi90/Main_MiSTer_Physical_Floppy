/*
 * Greaseweazle: set a non-standard baud rate on its USB serial port.
 * Setting 10000 baud tells the firmware to reset the communication state
 * (the line coding is just a signal, no data is sent at that speed).
 * Kept in its own file: <asm/termbits.h> can't be mixed with <termios.h>.
 */

#include <sys/ioctl.h>
#include <asm/termbits.h>

int gw_set_baud(int fd, int baud)
{
	struct termios2 t;
	if (ioctl(fd, TCGETS2, &t)) return -1;
	t.c_cflag &= ~CBAUD;
	t.c_cflag |= BOTHER;
	t.c_ispeed = baud;
	t.c_ospeed = baud;
	return ioctl(fd, TCSETS2, &t);
}
