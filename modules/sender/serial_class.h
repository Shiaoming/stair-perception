#ifndef _SERIAL_H
#define _SERIAL_H

#include <stdint.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#define BUFFER_SIZE 256
#define TIME_OUT 20 // 20 ms

enum parity
{
	NO,
	EVEN,
	ODD
};

class Cserial
{
	private:
		
	int	m_fd;
	struct termios m_oldtio;
	struct termios m_newtio;
	
	public:
		
		char Open(const char* port, int baud, char bits, parity parity, char stopbit);
		
        void Close(void);

		char Write(uint8_t* buffer, int length);

		int Read(uint8_t* buffer);
};

#endif
