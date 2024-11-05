/*********************************************************************************************************
*                                       Copyright Notice
*
*********************************************************************************************************/

/********************************************************************************************************==*
*                                      UART test client for NNTS
* Filename      : uartTest.c
* Version       : V1.2.0
* Programmers(s): Hank Yung, Omega Zareno, Rohith Ramnath
**********************************************************************************************************
* Notes         : The purpose of this code is to demonstrate the MPS Sensor UART API
*/
#define __MAIN_C

/* Includes ---------------------------------------------------------------------------------------------*/
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>

/* Defines ----------------------------------------------------------------------------------------------*/
/*
 * Conversion macros for switching between Little and Big Endian.
*/
#define SWAP16(num)        (((num & 0xff00) >> 8) | (num << 8))
#define SWAP32(num)        (((num & 0xff000000) >> 24) | ((num & 0x00ff0000) >> 8) | ((num & 0x0000ff00) << 8) | (num << 24))

/* Command Status */
#define UART_SUCCESS           0x00
#define UART_CRC_ERROR         0x01
#define UART_BAD_PARAM         0x02
#define UART_EXE_FAILED        0x03
#define UART_NO_MEM            0x04
#define UART_UNKNOWN_CMD       0x05

#define UART_LOCAL_ERROR       0xFF   /* Error generated locally - not from sensor */

/* commands */
#define CMD_ANSWER       0x01
#define CMD_ENGDATA      0x09
#ifdef FLAMMABLE
#define CMD_CONC         0x03
#define CMD_ID           0x04
#endif

#define CMD_TEMP         0x21
#define CMD_PRES         0x22
#define CMD_REL_HUM      0x23
#define CMD_ABS_HUM      0x24

#define CMD_STATUS       0x41
#define CMD_VERSION      0x42
#define CMD_SENSOR_INFO  0x43

#define CMD_MEAS         0x61
#define CMD_SHUTDOWN     0x62
#define CMD_UNKNOWN		 0x99

#define RQST_HDR_LENGTH     sizeof(uartRqstHeader_t)
#define REPLY_HDR_LENGTH    sizeof(uartReplyHeader_t)
#define NUM_OF_CMDS         (sizeof(uart_cmds) / sizeof(uart_cmd_t))
#define UART_MAX_DATA_SIZE  (1024*8)    /* maximum packet:  header + payload */
#define ENGDATA_CHUNKSIZE   512         /* size of each chunk of engineering data */
#define FINAL_PACKET        0x8000      /* bit to indicate last chunk of engineering data */

#define GAS_NAME_LENGTH     64

#define POST_SKU_CHANGE_FW  4100

/* Structure definitions --------------------------------------------------------------------------------*/
typedef struct {
  uint16_t cmdID;
  uint16_t length;
  uint16_t reserved;
  uint16_t cksum;
} uartRqstHeader_t;

typedef struct {
  uint8_t cmdID;
  uint8_t status;
  uint16_t length;
  uint16_t cksum;
} uartReplyHeader_t;

typedef struct {
  uint8_t cmdID;
  uint16_t req_size;   /* Request size */
  uint16_t res_size;   /* Response size */
  uint32_t (*func)(uint8_t cmdID, uint8_t *data, uint16_t size);
} uart_cmd_t;

typedef struct {
  uint8_t sw_w;
  uint8_t sw_x;
  uint8_t sw_y;
  uint8_t sw_z;
  uint8_t hw_w;
  uint8_t hw_x;
  uint8_t proto_w;
  uint8_t proto_x;
} uart_version_t;

typedef struct {
  uint8_t sensorName[32];  /* Serial name (zero-padded ASCII string) */
  uint32_t sensorType;   /* Sensor Type/Model */
  uint8_t sku[32];  /* Product SKU */
  uint8_t calDate[16];   /* Calibration date */
  uint8_t mfgDate[16];   /* Manufacturing  date */
} uart_sensor_info_t;

typedef struct {
  uint8_t sensorName[32];  /* Serial name (zero-padded ASCII string) */
  uint32_t sensorType;   /* Sensor Type/Model */
  uint8_t calDate[16];   /* Calibration date */
  uint8_t mfgDate[16];   /* Manufacturing  date */
} uart_sensor_info_no_sku_t;

#ifdef FLAMMABLE
typedef struct {
  int32_t cycleCount;
  float concentration;
  uint32_t flamID;
  float temp;
  float pressure;
  float relHumidity;
  float absHumidity;
} answer_t;

typedef struct {
  uint32_t length;
  uint8_t data[ENGDATA_CHUNKSIZE];
} uart_engdata_t;
#else
#error Need to define expected answer type!
#endif

typedef struct {
  float temp;
  float pressure;
  float humidity;
  float absHumidity;
  float humidAirDensity;
} enviro_reply_t;

#define RESERVED_DEFAULT 0x0000
#define CHKSUM_DEFAULT 0x0000
#define NULL_PAYLOAD 0x00

/* List of commands */
typedef struct {
  uint8_t cmd_str[16];
  uartRqstHeader_t header;
  uint8_t payload;
} commandList_t;

commandList_t command_list[] = {
  {"ANSWER",      {CMD_ANSWER,      0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
#ifdef FLAMMABLE
  {"CONC",        {CMD_CONC,        0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
  {"ID",          {CMD_ID,          0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
#endif
  {"TEMP",        {CMD_TEMP,        0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
  {"PRES",        {CMD_PRES,        0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
  {"REL_HUM",     {CMD_REL_HUM,     0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
  {"ABS_HUM",     {CMD_ABS_HUM,     0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
  {"STATUS",      {CMD_STATUS,      0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
  {"VERSION",     {CMD_VERSION,     0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
  {"SENSOR_INFO", {CMD_SENSOR_INFO, 0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
  {"MEAS",        {CMD_MEAS,        0x0001, RESERVED_DEFAULT, CHKSUM_DEFAULT}, 0x02        }, // START measuring (ISO)
  {"MEAS",        {CMD_MEAS,        0x0001, RESERVED_DEFAULT, CHKSUM_DEFAULT}, 0x22        }, // START measuring (IEC)
  {"MEAS",        {CMD_MEAS,        0x0001, RESERVED_DEFAULT, CHKSUM_DEFAULT}, 0x03        }, // STOP measuring 
  {"SHUTDOWN",    {CMD_SHUTDOWN,    0x0000, RESERVED_DEFAULT, CHKSUM_DEFAULT}, NULL_PAYLOAD},
};
#define NUM_CMDS_LIST        (sizeof(command_list) / sizeof(commandList_t))

/* Functions --------------------------------------------------------------------------------------------*/
extern int openSerialPort(char *device, int port);
extern void closeSerialPort(int);
extern int readSerial(char *readBuffer, uint32_t bytesToRead);
extern uint16_t crc_generate(uint8_t *buffer, size_t length, uint16_t startValue);
static uint32_t uartSingleRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint8_t uartSend(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint8_t uartReSend(uint8_t cmdID);
static uint32_t uartRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
static uint32_t ReadFloat(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadInteger(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadVersion(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadString(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadAnswer(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadSensorInfo(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadByte(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t WriteByte(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t WriteFloat(uint8_t cmdID, uint8_t *data, uint16_t size);
static uint32_t ReadEngData(uint8_t cmdID, uint8_t *data, uint16_t size);
static void DumpRqstHdr(uartRqstHeader_t *);
static void DumpReplyHdr(uartReplyHeader_t *);
static void DumpHexa(uint8_t *p, uint32_t len);

/* Variables --------------------------------------------------------------------------------------------*/
int uartFP;
uint32_t verbose = 0, hexdump = 0;
uint32_t numOfRetries = 0;
uint32_t rxTimeout = 0, rxBytes = 0, uartState = 0;
static uartRqstHeader_t pktHdrCache;
static uint8_t payloadCache[256];
static uint32_t payloadCacheLen = 0;
char *filename = NULL;

uart_cmd_t uart_cmds[] = {
  {CMD_ANSWER, 0, sizeof(answer_t), ReadAnswer},
  {CMD_MEAS, 1, 0, WriteByte},
#ifdef FLAMMABLE
  {CMD_CONC, 0, 4, ReadFloat},
  {CMD_ID, 0, 4, ReadInteger},
#endif
  {CMD_ENGDATA, 0, sizeof(uart_engdata_t), ReadEngData},
  {CMD_TEMP, 0, 4, ReadFloat},
  {CMD_PRES, 0, 4, ReadFloat},
  {CMD_REL_HUM, 0, 4, ReadFloat},
  {CMD_ABS_HUM, 0, 4, ReadFloat},
  {CMD_STATUS, 0, 1, ReadByte},
  {CMD_VERSION, 0, sizeof(uart_version_t), ReadVersion},
  {CMD_SENSOR_INFO, 0, sizeof(uart_sensor_info_no_sku_t), ReadSensorInfo},
  {CMD_SHUTDOWN, 0, 0, WriteByte},
  {CMD_UNKNOWN, 0, 0, WriteByte}
};

void usage(void) {
  printf("\nUsage:\n");
  printf("  uartTest -c <cmdID> [ -v <value>]\n");
  printf("   -c:  <cmdID> to execute in hex (e.g. 0x61)\n");
  printf("   -D:  <device> name (default:  /dev/ttyAMA)\n");
  printf("   -f:  full path of the file <name> (e.g. /tmp/abc.bin)\n");
  printf("   -p:  COM port number (default: port 0)\n");
  printf("   -v:  <value> of parameter to send with the command in hex (e.g. 0x01)\n");
  printf("   -V:  verbose mode\n");
  printf("   -x:  hex dump\n");
  printf("   -l:  list UART command examples\n");
  printf("\nExamples:\n");
  printf("  uartTest -l \n");
  printf("  uartTest -c 0x21 (default: /dev/ttyAMA0, read temp)\n");
  printf("  uartTest -D /dev/ttyUSB -p 1 -c 0x42 (/dev/ttyUSB1, read version information)\n");
  printf("  uartTest -D /dev/ttyAMA -p 1 -c 0x01 (/dev/ttyAMA1, read answer)\n");
  printf("  uartTest -D /dev/ttyAMA -p 0 -c 0x61 -v 0x2 (/dev/ttyAMA0, do 'continuous' measurement; set MEAS to 2)\n");
  exit(1);
}

void listCommands(void) {
  uint16_t cksum;

  printf("\nNevadaNano MPS Sensor API Commands\n\n");
  printf("Hex Code  Command ID\tRequest Header                                  Payload\n");
  printf("--------------------------------------------------------------------------------\n");
  printf("                   \t  CMD_ID  |  Length   |  Reserved | Checksum  | Payload\n");

  for(int i = 0; i < NUM_CMDS_LIST; i++) {
    printf("0x%02x     %11s\t", command_list[i].header.cmdID, command_list[i].cmd_str);

    // Calculate checksum of header
    cksum = crc_generate((uint8_t *) &(command_list[i].header), RQST_HDR_LENGTH, 0xFFFF);
    
    // Calculate checksum of payload
    if (command_list[i].payload != NULL_PAYLOAD) {
      cksum = crc_generate((uint8_t *)&command_list[i].payload, command_list[i].header.length, cksum);
    }

    // Print header packet
    printf("0x%02x 0x%02x | 0x%02x 0x%02x | 0x%02x 0x%02x | 0x%02x 0x%02x | ",
            (uint8_t)(command_list[i].header.cmdID & 0xFF),
            (uint8_t)(command_list[i].header.cmdID >> 8),
            (uint8_t)(command_list[i].header.length & 0xFF),
            (uint8_t)(command_list[i].header.length >> 8),
            (uint8_t)(command_list[i].header.reserved & 0xFF),
            (uint8_t)(command_list[i].header.reserved >> 8),
            (uint8_t)(cksum & 0xFF),
            (uint8_t)(cksum >> 8)        
            );
    
    if (command_list[i].payload != NULL_PAYLOAD) {
      printf("0x%02x", command_list[i].payload);
    }

    printf("\n");    

  }
  exit(1);
}

static uint8_t uartSend(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  uartRqstHeader_t header;
  uint16_t cksum, rxCksum, length;

  memset(&header, 0, RQST_HDR_LENGTH);
  header.cmdID = cmdID;
  header.length = payloadLen;

  cksum = crc_generate((uint8_t *) &header, RQST_HDR_LENGTH, 0xFFFF);
  header.cksum = cksum;

  if(payloadLen != 0) {
    if(payload == NULL) {
      printf("No payload given but payload lengh is non-zero\n");
      return 1;
    }
    cksum = crc_generate(payload, payloadLen, cksum);
  }
  header.cksum = cksum;

  if(verbose) {
    DumpRqstHdr(&header);
    if(hexdump)
      DumpHexa((uint8_t *) &header, RQST_HDR_LENGTH);
  }
  
  if(write(uartFP, (uint8_t *) &header, RQST_HDR_LENGTH) != RQST_HDR_LENGTH) {
    printf("Failed to send header: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
    return 1;
  }
  if(numOfRetries != 0) {
    pktHdrCache = header;
  }

  if(payloadLen) {
    if(hexdump) {
      printf("  Payload");
      DumpHexa(payload, payloadLen);
    }

    if(write(uartFP, payload, payloadLen) != payloadLen) {
      printf("Failed to send payload: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
      return 1;
    }

    if(numOfRetries != 0) {
      memcpy(payloadCache, payload, payloadLen);
      payloadCacheLen = payloadLen;
    }
  }

  return 0;
}

uint32_t uartRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  int32_t retry = 1;
  uint32_t status;

  status = uartSingleRecv(cmdID, payload, payloadLen);
  if((status == UART_SUCCESS) || (numOfRetries == 0))
    return status;

  do {
    if((status = uartReSend(cmdID)) != 0) {
      break;
    }

    status = uartSingleRecv(cmdID, payload, payloadLen);
  } while ((retry++ < numOfRetries) && (status != UART_SUCCESS));

  return status;
}

static uint32_t uartSingleRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  uint16_t rxCksum, cksum;
  int rxLen;
  uint32_t timeout;
  uartRqstHeader_t header;
  uartReplyHeader_t *reply;
  uint8_t buffer[UART_MAX_DATA_SIZE+1];

  memset(buffer, 0, sizeof(buffer));

  rxLen = readSerial(buffer, sizeof(uartReplyHeader_t));
  if(rxLen <= 0) {
    printf("Failed to get reply: %s (%d)\n", strerror(errno),  errno);
    return UART_LOCAL_ERROR;
  }

  reply = (uartReplyHeader_t *) buffer;
  if(rxLen < REPLY_HDR_LENGTH) {
    printf("Incomplete header received: %d bytes\n", rxLen);
    DumpReplyHdr(reply);
    return UART_LOCAL_ERROR;
  }

  if(reply->length != 0) {  /* Is there a payload for this reply? */
    rxLen = readSerial(&buffer[REPLY_HDR_LENGTH], reply->length);
    if(rxLen < reply->length) {
      printf("Failed to get reply payload: %s (%d)\n", strerror(errno),  errno);
      return UART_LOCAL_ERROR;
    }
  }

  rxCksum = reply->cksum;
  reply->cksum = 0;  /* zero out checksum field */
  cksum = crc_generate(buffer, REPLY_HDR_LENGTH + reply->length, 0xFFFF);
  if(rxCksum != cksum) {
    printf("Checksum failed: expected 0x%x, received 0x%x\n", cksum, rxCksum);
    reply->cksum = rxCksum;   /* restore received checksum */
    DumpReplyHdr(reply);
    return UART_LOCAL_ERROR;
  }

  reply->cksum = rxCksum;   /* restore received checksum */

  if(reply->status != UART_SUCCESS) {
    if(reply->status >= 0x20) {
      printf("Sensor hardware error: 0x%x\n", reply->status);
    } else {
      printf("Command returned error status: 0x%x\n", reply->status);
      DumpReplyHdr(reply);
      return (reply->status);  /* Sensor sent communication error */
    }
  }
  
  if(reply->cmdID != cmdID) {
    printf("cmdID mismatch: expected 0x%x, received 0x%x\n", cmdID, reply->cmdID);
    DumpReplyHdr(reply);
    return UART_LOCAL_ERROR;
  }

  if (reply->length == 0)
    return UART_SUCCESS;  /* No payload, we are done. */

  if(payloadLen < reply->length) {
    printf("Buffer too small for payload (%d < %d)\n", payloadLen, reply->length);
    return UART_LOCAL_ERROR;
  }

  memset(payload, 0, payloadLen);
  memcpy(payload, &buffer[REPLY_HDR_LENGTH], reply->length);

  if(verbose) {
    DumpReplyHdr(reply);
    if(hexdump)
      DumpHexa((uint8_t *)reply, REPLY_HDR_LENGTH);

    if(payloadLen) {
      if(hexdump) {
        printf("  Payload");
        DumpHexa(payload, payloadLen);
      }
    }
  }

  return UART_SUCCESS;
}

static uint8_t uartReSend(uint8_t cmdID) {
  uartReplyHeader_t reply;
  uint16_t cksum, rxCksum, length;

  if(write(uartFP, (uint8_t *) &pktHdrCache, RQST_HDR_LENGTH) != RQST_HDR_LENGTH) {
    printf("Failed to send header: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
    return 1;
  }

  if(payloadCacheLen) {
    if(write(uartFP, payloadCache, payloadCacheLen) != payloadCacheLen) {
      printf("Failed to send payload: 0x%x, %s (%d)\n", cmdID, strerror(errno), errno);
      return 1;
    }
  }

  return 0;
}

int main(int argc, char *argv[]) {
  uint8_t cmdID;
  int c, ii;
  int portNumber = 0;
  char *device = "/dev/ttyAMA";
  uint32_t oper = 0, value = 0, sts;
  uint8_t reply[UART_MAX_DATA_SIZE];

  while ((c = getopt(argc, argv, "hlVxnzc:f:D:p:r:t:w:v:")) != -1) {
    switch (c) {

    case 'c':
      cmdID = strtol(optarg, NULL, 16);
      break;
    case 'D':
      device = optarg;
      break;
    case 'f':
      filename = optarg;
      break;
    case 'p':
      portNumber = atoi(optarg);
      break;
    case 'v':
      value = strtol(optarg, NULL, 16);
      break;
    case 'V':
      verbose = 1;
      break;
    case 'x':
      hexdump = 1;
      verbose = 1;
      break;
    case 'l':
      listCommands();
      break;
    case 'h':
    default:
      usage();
    }
  }

  if(portNumber == -1) {
    printf("Invalid COM port number (-p)!\n");
    usage();
  }

  if ((uartFP = openSerialPort(device, portNumber)) == -1) {
    printf("Failed to open %s%d: %s (%d)\n", device, portNumber, strerror(errno), errno);
    exit(1);
  }

  for(ii = 0; ii < NUM_OF_CMDS; ii++) {
    if(uart_cmds[ii].cmdID != cmdID)
      continue;

    if(uart_cmds[ii].req_size) {
      sts = uart_cmds[ii].func(cmdID, (uint8_t *) &value, uart_cmds[ii].req_size);
    } else if(uart_cmds[ii].res_size) {
      sts = uart_cmds[ii].func(cmdID, reply, uart_cmds[ii].res_size);
    } else {
      sts = uart_cmds[ii].func(cmdID, NULL, 0);
    }
    break;
  }  

  if(ii == NUM_OF_CMDS) {
    printf("No such command: 0x%x\n", cmdID);
	sts = uart_cmds[ii-1].func(cmdID, NULL, 0);
    //sts = 1;
  }

  closeSerialPort(uartFP);
  exit(sts);
}

static uint32_t ReadFloat(uint8_t cmdID, uint8_t *data, uint16_t size) {
  float *value;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  value = (float *) data;
  printf("Command[0x%02x]: %f\n", cmdID, *value);

  return 0;
}

static uint32_t ReadInteger(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uint32_t *value;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  value = (uint32_t *) data;
  printf("Command[0x%02x]: %u\n", cmdID, *value);

  return 0;
}

static uint32_t ReadSensorInfo(uint8_t cmdID, uint8_t *data, uint16_t size) {
	
  uint16_t fwVersion = 0;

  //Get the FW version
  uart_version_t *version;
  
  if(uartSend((uint8_t)CMD_VERSION, NULL, 0) != 0)
    return 1;

  if(uartRecv((uint8_t)CMD_VERSION, data, sizeof(uart_version_t)) != 0)
    return 1;

  version = (uart_version_t *) data;  
  fwVersion = (version->sw_w * 1000) + (version->sw_x * 100) + (version->sw_y * 10) + (version->sw_z * 1);
  
  //Process Sensor Info command
  if(fwVersion >= POST_SKU_CHANGE_FW){
	uart_cmds[11].res_size = sizeof(uart_sensor_info_t);
	size = sizeof(uart_sensor_info_t);
  }
  
  uart_sensor_info_t *sensor;
  uart_sensor_info_no_sku_t *sensor_no_sku;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;
 
  if(fwVersion >= POST_SKU_CHANGE_FW){
	sensor = (uart_sensor_info_t *) data;
	printf("Serial Number: %s\nSensor Type: %d\nSKU: %s\nCalibration Date: %s\nManufactured Date: %s\n",
    sensor->sensorName, sensor->sensorType, sensor->sku, sensor->calDate, sensor->mfgDate);
  }
  else{
	sensor_no_sku = (uart_sensor_info_no_sku_t *) data;
	printf("Serial Number: %s\nSensor Type: %d\nCalibration Date: %s\nManufactured Date: %s\n",
    sensor_no_sku->sensorName, sensor_no_sku->sensorType, sensor_no_sku->calDate, sensor_no_sku->mfgDate);	  
  }

  return 0;
}

static uint32_t ReadVersion(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uart_version_t *version;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  version = (uart_version_t *) data;
  printf("SW Version: %u.%u.%u.%u\nHW Version: %u.%u\nProtocol: %u.%u\n",
         version->sw_w, version->sw_x, version->sw_y, version->sw_z,
         version->hw_w, version->hw_x, version->proto_w, version->proto_x);  

  return 0;
}

static uint32_t ReadString(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uart_version_t *version;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  printf("%s\n", data);
  return 0;
}

static uint32_t ReadAnswer(uint8_t cmdID, uint8_t *data, uint16_t size) {
  answer_t *answer;

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  answer = (answer_t *) data;
#ifdef FLAMMABLE
  printf("Cycle: %i\nGas: %d\nConcentration: %f\nTEMP: %f\nPRESS: %f\nREL_HUM: %f\nABS_HUM: %f\n",
         answer->cycleCount, answer->flamID, answer->concentration, answer->temp, answer->pressure, answer->relHumidity, answer->absHumidity);
#endif
  return 0;
}

static uint32_t ReadByte(uint8_t cmdID, uint8_t *data, uint16_t size) {

  if(uartSend(cmdID, NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0)
    return 1;

  printf("Command[0x%02x]: 0x%x\n", cmdID, *data);

  return 0;
}

static uint32_t WriteByte(uint8_t cmdID, uint8_t *data, uint16_t size) {
  if(uartSend(cmdID, data, size) != 0)
    return 1;
  
  if(uartRecv(cmdID, NULL, 0) != 0)
    return 1;

  return 0;
}

static uint32_t WriteFloat(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uint32_t val;
  float fval;

  val = *((uint32_t *) data);
  fval = ((float) val) / 100.0;

  printf("%s: %d %f\n", __FUNCTION__, val, fval);
  if(uartSend(cmdID, (uint8_t *) &fval, sizeof(fval)) != 0)
    return 1;

  if(uartRecv(cmdID, NULL, 0) != 0)
    return 1;

  return 0;
}

static uint32_t ReadEngData(uint8_t cmdID, uint8_t *data, uint16_t size) {
  uart_engdata_t *engdata;
  uint8_t *p;
  FILE *fp;

  if(filename == NULL) {
    printf("Missing filename parameters ('-f' option)\n");
    return 0;
  }

  fp = fopen(filename, "ab");
  if(fp == NULL) {
    printf("File open (%s) failed: %s\n", filename, strerror(errno));
    return 1;
  }

  if(uartSend(cmdID, (uint8_t*) NULL, 0) != 0)
    return 1;

  if(uartRecv(cmdID, data, size) != 0) {
    printf("Failed to read\n");
    return 1;
  }

  engdata = (uart_engdata_t *) data;
  fwrite(engdata, engdata->length + 4 /* +4 bytes 'length' */, 1, fp);
  fflush(stdout);

  fclose(fp);
  return 0;
}

static void DumpRqstHdr(uartRqstHeader_t *rqst) {
  printf("----\nREQUEST:\n");
  printf("  Hdr Size: %lu\n", sizeof(uartRqstHeader_t));
  printf("  CmdID: 0x%x\n", rqst->cmdID);
  printf("  Length: %d\n", rqst->length);
  printf("  Reserved: 0x%x\n", rqst->reserved);
  printf("  Checksum: 0x%x\n", rqst->cksum);
}

static void DumpReplyHdr(uartReplyHeader_t *reply) {
  printf("----\nREPLY:\n");
  printf("  CmdID: 0x%x\n", reply->cmdID);
  printf("  Status: 0x%x\n", reply->status);
  printf("  Length: %d\n", reply->length);
  printf("  Checksum: 0x%x\n", reply->cksum);
}

static void DumpHexa(uint8_t  *p, uint32_t len) {
  int ii;

  for(ii = 0; ii < len; ii++) {
    if((ii % 8) == 0)
      printf("\n    [%02d]: ", ii);

    printf("0x%02x ", *p++);
  }
  printf("\n");
}
