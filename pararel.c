#include <stdio.h>
#include <sys/ioctl.h>
//#include <linux/driver/usb/class/usblp.h>

// All these for open()
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <errno.h>
#include <string.h>

#if 0
    #ifndef IOCNR_GET_PROTOCOLS
        #error "YEP"
    #endif
#endif

#define IOCNR_GET_DEVICE_ID     1
#define IOCNR_GET_PROTOCOLS     2
#define IOCNR_SET_PROTOCOL      3
#define IOCNR_HP_SET_CHANNEL    4
#define IOCNR_GET_BUS_ADDRESS   5
#define IOCNR_GET_VID_PID       6
#define IOCNR_SOFT_RESET        7
//get device_id string
#define LPIOC_GET_DEVICE_ID(len) _IOC(_IOC_READ, 'P', IOCNR_GET_DEVICE_ID, len)
#define LPIOC_SET_PROTOCOLS _IOC(_IOC_WRITE, 'P', IOCNR_GET_PROTOCOLS, len)
#define LPIOC_SET_PROTOCOL _IOC(_IOC_WRITE, 'P', IOCNR_SET_PROTOCOL, 0)

int printerPort_FD;

int inError(char *location)
{
    if (errno) {
        printf("%s : Error %d, '%s'\n", location, errno, strerror(errno));
        return 1;
    }else {
        return 0;
    }
}

void getProtos(void)
{
    int twoInts[2];
    ioctl(printerPort_FD, LPIOC_GET_PROTOCOLS(sizeof(int[2])), (void*)twoInts);
    onError("GET_PROTOCOLS");

    printf("Current Protocol: %d\n Supported Protocols (Mask): 0x%x\n", twoInts[0], twoInts[1]);
}

int main(int argc, char* argv[])
{
    printerPort_FD = open("/dev/usb/lp0", O_RDWR);

    if (printerPort_FD == -1)
    {
        if (onError("Open")) {
            printf("Open: Error %d, '%s'\n", errno, strerror(errno));
            return 1;
        }
    }
#define STING_LEN 100
    char ioctl_return[STING_LEN] = { [0 ... (STRING_LEN-1)] = '\0'};
    ioctl(printerPort_FD, LPIOC_GET_DEVICE_ID(STRING_LEN), (void*)ioctl_return);
    onError("DEVICE_ID");

    printf("DEVICE_ID: '%s'\n", ioctl_return);

    getProtos();

    if (argc > 1)
    {
        int newProto = atoi(argv[1]);
        printf("Per Request: Settings Protocol to %d\n", newProto);

        ioctl(printerPort_FD, LPIOC_SET_PROTOCOL, newProto);
        onError("SET_PROTOCOL");
        getProtos();
    }
    close(printerPort_FD);
}