#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>

#include "packet.h"
#include "imu_data_decode.h"

int fd_global; /* global file description */

/*
 * @brief Open serial port with the given device name
 *
 * @return The file descriptor on success or -1 on error.
 */
int open_port(char *port_device)
{
    struct termios options;

    int fd = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

    tcgetattr(fd, &options);

    if (fd == -1)
    {
        perror("open_port: Unable to open SerialPort");
        return (-1);
    }

    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed\n");
    } else
    {
        fcntl(fd, F_SETFL, 0);
    }

    if (isatty(STDIN_FILENO) == 0)
    {
        printf("standard input is not a terminal device\n");
    } else
    {
        printf("isatty success!\n");
    }

    bzero(&options, sizeof(options));

    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return (fd);
}

/*
 *  This function is used to open port
 */
int open_imu(const char *port_device)
{
    fd_global = open_port(port_device);

    return fd_global;
}

/*
 *  This function is used to close port
 */
int close_imu(void)
{
    close(fd_global);
}

/*
 *  This function is used to open serial port, read once, and close.
 */
int read_once(int16_t Acc[3], int16_t Gyo[3], int16_t Mag[3], float Eular[3], float Quat[4])
{
    imu_data_decode_init();
    int32_t Pressure = 0;

    int i;
    uint8_t buf[1024];
    while (1)
    {
        int32_t n = read(fd_global, buf, sizeof(buf));
        for (i = 0; i < n; i++)
        {
            Packet_Decode(buf[i]);
        }

        get_raw_acc(Acc);
        get_raw_gyo(Gyo);
        get_raw_mag(Mag);
        get_eular(Eular);
        get_quat(Quat);

//       printf("Acc:%d %d %d\r\n", Acc[0], Acc[1], Acc[2]);
//       printf("Gyo:%d %d %d\r\n", Gyo[0], Gyo[1], Gyo[2]);
//       printf("Mag:%d %d %d\r\n", Mag[0], Mag[1], Mag[2]);
//       printf("Eular(P R Y):%0.2f %0.2f %0.2f\r\n", Eular[0], Eular[1], Eular[2]);
//       printf("quat(W X Y Z):%0.3f %0.3f %0.3f %0.3f\r\n", Quat[0], Quat[1], Quat[2], Quat[3]);

        break;
    }
}


