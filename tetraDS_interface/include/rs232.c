#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include "rs232.h"

int open_port(const char *device)
{
	int fd;	//  File descriptor for the port

	fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1) {
		perror("open_port : Unable to open port");
	}
	else {
		fcntl(fd, F_SETFL, 0);
	}
	
	return fd;
}

void close_port(int fd)
{
	close(fd);
}

int set_terminal(int fd, int baudrate, int time)
{
	struct termios term;

	tcgetattr(fd, &term);
	bzero(&term, sizeof(term));

	//  set baud rate
	switch(baudrate)
	{
		case 2400 :
			cfsetispeed(&term, B2400);
			cfsetospeed(&term, B2400);
			break;
		case 4800 :
			cfsetispeed(&term, B4800);
			cfsetospeed(&term, B4800);
			break;
		case 9600 :
			cfsetispeed(&term, B9600);
			cfsetospeed(&term, B9600);
			break;
		case 19200 :
			cfsetispeed(&term, B19200);
			cfsetospeed(&term, B19200);
			break;
		case 38400 :
			cfsetispeed(&term, B38400);
			cfsetospeed(&term, B38400);
			break;
		case 57600 :
			cfsetispeed(&term, B57600);
			cfsetospeed(&term, B57600);
			break;
		case 115200 :
			cfsetispeed(&term, B115200);
			cfsetospeed(&term, B115200);
			break;
		case 230400 :
			cfsetispeed(&term, B230400);
			cfsetospeed(&term, B230400);
			break;
		default :
			cfsetispeed(&term, B38400);
			cfsetospeed(&term, B38400);
			break;
	}

	//  enable the receiver and set local mode
	term.c_cflag |= (CLOCAL | CREAD);
	//  set parity checking(8N1)
	term.c_cflag &= ~PARENB;
	term.c_cflag &= ~CSTOPB;
	term.c_cflag &= ~CSIZE;
	term.c_cflag |= CS8;
	//  set hardware flow control disable
	term.c_cflag &= ~CRTSCTS;
	term.c_cflag &= ~HUPCL;

	//  set Ignore parity errors
	term.c_iflag |= IGNPAR;
	//  set software flow control disable
	term.c_iflag &= ~(IXON | IXOFF | IXANY);
	
	//
	term.c_iflag &= ~ICRNL;
	term.c_iflag &= ~INLCR;
	term.c_iflag &= ~IGNCR;

	term.c_oflag &= ~ONLCR;
	term.c_oflag &= ~OCRNL;
	term.c_oflag &= ~ONOCR;
	term.c_oflag &= ~ONLRET;
	term.c_oflag &= ~OPOST;

	term.c_lflag &= ~ICANON;
	term.c_lflag &= ~(ECHO | ECHOE);
	term.c_lflag &= ~ISIG;

	term.c_cc[VMIN] = 0;
	term.c_cc[VTIME] = time;
	
	tcsetattr(fd, TCSANOW, &term);

	return 0;
}
