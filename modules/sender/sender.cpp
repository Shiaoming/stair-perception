#include "serial_class.h"

char Cserial::Open(const char* port, int baud, char bits, parity parity, char stopbit)
{
    char tmp[BUFFER_SIZE];
    
    // Convert to string
    sprintf(tmp, "%s", port);
    
    m_fd = open(tmp, O_RDWR | O_NOCTTY | O_NDELAY);//O_NONBLOCK);
    
    // Check opening status 
    if (m_fd < 0)
    {
        printf("Error Opening %s Port\n", tmp);
        return 0;
    }

    if (fcntl(m_fd, F_SETFL, 0)<0)
    {
        printf("fcntl failed!\n");
        return 0;
    }
    
    if (isatty(STDIN_FILENO) == 0)
    {
        printf("standard input is not a terminal device!\n");
        return 0;
    }

    // Get terminal parameters
    if (tcgetattr(m_fd, &m_oldtio) != 0)
    {
        perror("SetupSerial 1");
        return 0;
    }

    bzero(&m_newtio, sizeof(m_newtio));
    m_newtio.c_cflag |= CLOCAL | CREAD;

    // tcgetattr(m_fd, &m_oldtio);

    // Flushes data received but not read       
    // ioctl(m_fd, TCIFLUSH);

    // Set baud rate (in and out)
    switch(baud)
    {
        case 9600   : 
            cfsetispeed(&m_newtio, B9600); 
            cfsetospeed(&m_newtio, B9600); 
            break;
        case 19200  : 
            cfsetispeed(&m_newtio, B19200); 
            cfsetospeed(&m_newtio, B19200);
            break;
        case 38400  : 
            cfsetispeed(&m_newtio, B38400); 
            cfsetospeed(&m_newtio, B38400);
            break;
        case 57600  : 
            cfsetispeed(&m_newtio, B57600); 
            cfsetospeed(&m_newtio, B57600);
            break;
        case 115200 : 
            cfsetispeed(&m_newtio, B115200);
            cfsetospeed(&m_newtio, B115200);
            break;
        default     :
            cfsetispeed(&m_newtio, B9600);
            cfsetospeed(&m_newtio, B9600); 
            break;
    }

    // Set byte size
    m_newtio.c_cflag &= ~CSIZE; 

    switch(bits)
    {
        case 5  : m_newtio.c_cflag |= CS5; break;
        case 6  : m_newtio.c_cflag |= CS6; break;
        case 7  : m_newtio.c_cflag |= CS7; break;
        case 8  : m_newtio.c_cflag |= CS8; break;
        default : m_newtio.c_cflag |= CS8; break;
    }

    // Set parity
    switch(parity)
    {
        case NO   :
                m_newtio.c_cflag &=~ PARENB;    // Disable parity
            break;
            
        case EVEN :
                m_newtio.c_iflag |= (INPCK | ISTRIP);
                m_newtio.c_cflag |= PARENB;     // Enable parity
                m_newtio.c_cflag &= ~PARODD;    // Disable odd parity
            break;
            
        case ODD  :
                m_newtio.c_cflag |= PARENB;     // Enable parity
                m_newtio.c_cflag |= PARODD;     // Enable odd parity
                m_newtio.c_iflag |= (INPCK | ISTRIP);
            break;
            
        default   :
                m_newtio.c_cflag &=~ PARENB;    // Disable parity
    }
    
    // Set stop bit
    switch(stopbit)
    {
        case 1: m_newtio.c_cflag &=~ CSTOPB ; break;    // Disable 2 stop bits
        case 2: m_newtio.c_cflag |= CSTOPB  ; break;    // Enable 2 stop bits
        default: m_newtio.c_cflag &=~ CSTOPB;
    }

    // // Enable receiver (CREAD) and ignore modem control lines (CLOCAL)
    // m_newtio.c_cflag |= (CREAD | CLOCAL); 
    
    // // Disable, canonical mode (ICANON = 0), echo input character (ECHO) and signal generation (ISIG)
    // m_newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    
    // // Disable input parity checking (INPCK)
    // m_newtio.c_iflag &= ~INPCK;        

    // // Disable XON/XOFF flow control on output and input (IXON/IXOFF), framing and parity errors (IGNPAR), and disable CR to NL translation
    // m_newtio.c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR | ICRNL);

    // // Disable implementation-defined output processing (OPOST)
    // m_newtio.c_oflag &=~ OPOST;  

    // // Set terminal parameters
    // tcsetattr(m_fd, TCSAFLUSH, &m_newtio);

    m_newtio.c_cc[VTIME] = 0;
    m_newtio.c_cc[VMIN] = 0;
    tcflush(m_fd, TCIFLUSH);

    if ((tcsetattr(m_fd, TCSANOW, &m_newtio)) != 0)
    {
        perror("com set error");
        return 0;
    }
    
    // Display settings
    printf("%s | BaudRate: %d | Bits: %d | Parity: %d | StopBits: %d\n", tmp, baud, bits, parity, stopbit);

    return 1;
}

void Cserial::Close(void)
{
    // Set old parameters
    tcsetattr(m_fd, TCSANOW, &m_oldtio);
    
    // Close serial port
    if(m_fd > 0)
    {
        close(m_fd);
    }
}

char Cserial::Write(uint8_t* buffer, int length)
{
    ssize_t r;
    const uint8_t *pos = (const uint8_t*)buffer;

    // Send data
    while(length > 0)
    {
        r = write(m_fd, pos, length);
        
        if(r < 1)
        {
            return 0;
        }

        length -= r;
        pos += r;
    }

    return 1;
}

int Cserial::Read(uint8_t* buffer)
{
    ssize_t r = 1;
    int length = 0;

    int retval;
    fd_set rfds;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = TIME_OUT*1000;
    
    while(1)
    {
        FD_ZERO(&rfds);
        FD_SET(m_fd, &rfds);
        retval = select(m_fd + 1, &rfds, NULL, NULL, &tv);

        if (retval == -1)
        {
            perror("select()");
            break;
        }
        else if (retval)
        {
            // Protect buffer
            if(length >= (BUFFER_SIZE - 1)) return length;

            r = read(m_fd, (buffer + length), 1);

            if(r > 0)
                length++;
            else if (-1 == r)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
    
    return length;
}