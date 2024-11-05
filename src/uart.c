
/* Includes ---------------------------------------------------------------------------------------------*/
#include "uart.h"

/* Defines ----------------------------------------------------------------------------------------------*/
#define MAX_DEVICE_LEN      16

/* Variables --------------------------------------------------------------------------------------------*/
static struct termios options;
static char portName[MAX_DEVICE_LEN];
int serialPort;

/* Functions --------------------------------------------------------------------------------------------*/
/* Open serial port*/
int openSerialPort(char *device, int port){
  if(device == NULL)
    device = DEV_ACM;

  snprintf(portName, MAX_DEVICE_LEN, "%s%d", device, port);
  serialPort = open(portName, O_RDWR | O_NOCTTY);

  if (serialPort != -1){
    if(!strcmp(device, DEV_ACM)) {  /* USB CDC (virtual COM port ) */
      tcgetattr(serialPort, &options);                          /* get current port options */
      cfsetispeed(&options, B115200);                           /* set baud rate */
      cfsetospeed(&options, B115200);

      options.c_cflag |= (CLOCAL | CREAD | CS8);                        /* control options - local mode, enable receiver, 8N1 configuration, no flow control */
      options.c_cflag &= ~(CSIZE | PARENB | CSTOPB );
      options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | IEXTEN | ISIG);     /* local options - canonical input */
      options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

      options.c_oflag &= ~OPOST;                                        /* output options - raw output */

      tcsetattr(serialPort, TCSANOW, &options);
    } else {   /* /dev/ttyO, /dev/serial or /dev/ttyUSB */
      tcgetattr(serialPort, &options);
      options.c_cflag = B38400 | CS8 | CLOCAL | CREAD;          /* set baud rate */
      options.c_iflag = IGNPAR;
      options.c_oflag = 0;
      options.c_lflag &= ~ICANON; /* Set non-canonical mode */
      options.c_lflag = 0;
      options.c_cc[VTIME] = 2; /* Set timeout in 'deci-sec' */
      options.c_cc[VMIN] = 0; /* no minimum character */
      tcflush(serialPort, TCIFLUSH);
      tcsetattr(serialPort, TCSANOW, &options);
    }
    
    signal (SIGINT, (void*)signalHandler);
  }
  else {
    printf("Unable to open %s.  (Is device powered on?)\n", portName);
  }
  return (serialPort);
}

void closeSerialPort(int port){
  tcsetattr(port, TCSANOW, &options);
  close(port);
}

int readSerial(char *readBuffer, uint32_t bytesToRead) {
  int len;
  uint32_t bytesRead = 0, retries = 0;

  do {
    len = read(serialPort, readBuffer, bytesToRead);

    if(len < 0) {
      printf("%s: error %s (%d)\n", __FUNCTION__, strerror(errno), errno);
      return 0;
    }

    if(len == 0) {
      if (retries++ > 2) break;
      else continue;
    }

    readBuffer += len;
    bytesRead += len;
    bytesToRead -= len;
  } while (bytesToRead != 0);

  if(bytesRead < bytesToRead) {
    if(bytesRead == 0)
      errno = ETIMEDOUT;
  }
  return bytesRead;
}

void serialWrite(char *writeBuffer, uint32_t len){
  int bytesWritten;

  bytesWritten = write(serialPort, writeBuffer, len);
  if (bytesWritten < len){
    printf("Write failed \n");
  }
}

/* close serial port on ctrl+c */
void signalHandler(int sig){
  closeSerialPort(serialPort);
  exit (sig);
}
