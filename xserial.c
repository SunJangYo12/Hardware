#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

struct termios SerialPortSettings;

void main()
{
    int fd;
    fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);

    if (fd == -1) {
        printf("\n Error! in Opening ttyUSB1\n");
    }else{
        char pilih;
        printf("\n ttyUSB1 Opcened Successfully pilih metode? r/w: ");scanf("%c", &pilih);

        if (pilih == 'w') {
            printf("\n +----------------------------------+");
            printf("\n |        Serial Port Write         |");
            printf("\n +----------------------------------+");

            tcgetattr(fd, &SerialPortSettings);
            cfsetispeed(&SerialPortSettings, B9600);
            cfsetospeed(&SerialPortSettings, B9600);

            SerialPortSettings.c_cflag &= ~PARENB;
            SerialPortSettings.c_cflag |= PARENB;
            SerialPortSettings.c_cflag &= CSTOPB;
            SerialPortSettings.c_cflag &= ~CSIZE;
            SerialPortSettings.c_cflag |= CS8;

            // Other options
            SerialPortSettings.c_cflag &= ~CRTSCTS; //turn off RTS/CTS
            SerialPortSettings.c_cflag |= CREAD | CLOCAL; //turn on receiverr
            SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); //turn off based flow control (XON/XOFF)

            SerialPortSettings.c_iflag &- ~(ICANON | ECHO | ECHOE | ISIG);

            tcsetattr(fd, TCSANOW, &SerialPortSettings);

            char write_buffer[] = "A";
            int byte_written = 0;

            byte_written = write(fd, write_buffer, sizeof(write_buffer));
        }
        if (pilih == 'r') {
            printf("\n +----------------------------------+");
            printf("\n |        Serial Port Read         |");
            printf("\n +----------------------------------+");
        }

    }
    close(fd);
}
