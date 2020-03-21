#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h> //ioctl() call defenitions
#include <time.h>

struct termios SerialPortSettings;
char pilih, dpilih;

void msleep(long msec) {
    struct timespec ts;
    int res;

    if (msec < 0){
        errno = EINVAL;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

}

void main()
{
    int fd;
    fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        printf("\n Error! in Opening ttyUSB1\n");
    }else{
        printf("\nttyUSB1 Opcened Successfully\n\n");
        printf("Pilih metode read/write/data? r/w/d: ");scanf("%c", &pilih);

        if (pilih == 'd') {
            printf("\n +----------------------------------+");
            printf("\n |        Serial Port Data         |");
            printf("\n +----------------------------------+\n");

            char dpilih[20];
            printf("Enter input/output? i/o: ");scanf("%s", dpilih);

            int flag, random;
            random = 0;
            if (strcmp(dpilih, "i") == 0) 
            {
                char dpilihpin[20];
                int s, dat;
                printf("Enter pin? CTS/DSR: ");scanf("%s", dpilihpin);

                if (strcmp(dpilihpin, "CTS") == 0)
                    flag = TIOCM_CTS;
                else if (strcmp(dpilihpin, "DSR") == 0) 
                    flag = TIOCM_DSR;

                while (1) {
                    ioctl(fd, TIOCMGET, &s);
                    dat = (s & flag);

                    printf("Monitor data: %d\n", dat);
                    msleep(500);
                }
            }
            else if (strcmp(dpilih, "o") == 0) 
            {
                char dpilihpin[20];
                printf("Enter pin? RTS/DTR: ");scanf("%s", dpilihpin);

                if (strcmp(dpilihpin, "RTS") == 0)
                    flag = TIOCM_RTS;
                else if (strcmp(dpilihpin, "DTR") == 0) 
                    flag = TIOCM_DTR;

                while (1) {
                    printf("data: %d\n", random);

                    ioctl(fd, TIOCMBIS, &flag);//Set RTS pin
                    msleep(500);
                    ioctl(fd, TIOCMBIC, &flag);//Clear RTS pin
                    msleep(500);
                    random++;
                }
            }

            close(fd);
        }

        if (pilih == 'w') {
            printf("\n +----------------------------------+");
            printf("\n |        Serial Port Write         |");
            printf("\n +----------------------------------+");

            tcgetattr(fd, &SerialPortSettings); /* Get the current attributes of the Serial port */
            cfsetispeed(&SerialPortSettings,B9600); /* Set Read  Speed as 9600                       */
            cfsetospeed(&SerialPortSettings,B9600); /* Set Write Speed as 9600                       */

            SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
            SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
            SerialPortSettings.c_cflag &= ~CSIZE;    /* Clears the mask for setting the data size             */
            SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
    
            SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
            SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
        
            SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
            SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

            SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

            if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
                printf("\n  ERROR ! in Setting attributes");
            else
                printf("\n  BaudRate = 9600 \n  StopBits = 1 \n  Parity   = none");
            
            /*------------------------------- Write data to serial port -----------------------------*/
            char write_buffer[] = "A";  /* Buffer containing characters to write into port       */ 
            int  bytes_written  = 0;    /* Value for storing the number of bytes written to the port */ 
            bytes_written = write(fd,write_buffer,sizeof(write_buffer));/* use write() to send data to port                                            */

            printf("\n  %s written to ttyUSB1",write_buffer);
            printf("\n  %d Bytes written to ttyUSB1", bytes_written);
            printf("\n +----------------------------------+\n\n");
            close(fd);
        }
        if (pilih == 'r') {
            printf("\n +----------------------------------+");
            printf("\n |        Serial Port Read         |");
            printf("\n +----------------------------------+");

            tcgetattr(fd, &SerialPortSettings); /* Get the current attributes of the Serial port */
            /* Setting the Baud rate */
            cfsetispeed(&SerialPortSettings,B9600); /* Set Read  Speed as 9600                       */
            cfsetospeed(&SerialPortSettings,B9600); /* Set Write Speed as 9600                       */

            /* 8N1 Mode */
            SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
            SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
            SerialPortSettings.c_cflag &= ~CSIZE;    /* Clears the mask for setting the data size             */
            SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
        
            SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
            SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
        
            SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
            SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

            SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/
        
            /* Setting Time outs */
            SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
            SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */

            if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
                printf("\n  ERROR ! in Setting attributes");
            else
                printf("\n  BaudRate = 9600 \n  StopBits = 1 \n  Parity   = none");
            
            /*------------------------------- Read data from serial port -----------------------------*/
            tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */
            char read_buffer[32];   /* Buffer to store the data received              */
            int  bytes_read = 0;    /* Number of bytes read by the read() system call */
            int i = 0;

            bytes_read = read(fd,&read_buffer,32); /* Read the data                   */
            
            printf("\n\n  Bytes Rxed -%d", bytes_read); /* Print the number of bytes read */
            printf("\n\n  ");

            for(i=0;i<bytes_read;i++)    /*printing only the received characters*/
                printf("%c",read_buffer[i]);
            printf("\n +----------------------------------+\n\n\n");

            close(fd); /* Close the serial port */
        }
    }
}
