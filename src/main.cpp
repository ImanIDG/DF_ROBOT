/*!
 * @file gainConcentration.ino
 * @brief Get the current concentration of PM in the air
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      PengKaixing(kaixing.peng@dfrobot.com)
 * @version  V1.0.0
 * @date  2021-11-23
 * @url https://github.com/dfrobot/DFRobot_AirQualitySensor
 */
#include <Arduino.h>
#include "DFRobot_AirQualitySensor.h"
#include "uart.h"

#define I2C_ADDRESS    0x19
  DFRobot_AirQualitySensor particle(&Wire ,I2C_ADDRESS);

#define RQST_HDR_LENGTH     sizeof(uartRqstHeader_t)
#define REPLY_HDR_LENGTH    sizeof(uartReplyHeader_t)
#define UART_MAX_DATA_SIZE  (1024*8)    /* maximum packet:  header + payload */
#define UART_SUCCESS           0x00
union a16to8 {
	 uint16_t a16;
	 uint8_t a8[2];
};
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
  uint8_t sw_w;
  uint8_t sw_x;
  uint8_t sw_y;
  uint8_t sw_z;
  uint8_t hw_w;
  uint8_t hw_x;
  uint8_t proto_w;
  uint8_t proto_x;
} uart_version_t;
uint16_t crc_generate(uint8_t *buffer, size_t length, uint16_t startValue);
uint8_t MPS_status(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
uint8_t MPS_version(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
uint32_t uartRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);
uint32_t uartSingleRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen);

uint32_t numOfRetries = 0;
//uint8_t MPS_start_meas();
//uint8_t MPS_ans();
void setup() {
  Serial.begin(9600);
  Serial2.begin(38400);

  //Sensor init, initialize I2C, determined by the communication method currently used
  while(!particle.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }
  Serial.println("DFRobot sensor begin success!");
  delay(1000);

  //Get sensor version number
  uint8_t version = particle.gainVersion();
  Serial.print("DFRobot version is : ");
  Serial.println(version);
  delay(3000);
  uint8_t error;
  do {
    error = MPS_status(0x41, NULL, 0);
  }while (error != 0);
  delay(1000);
  while (1) {
    error = MPS_version(0x42, NULL, 0);
    delay(1000);
  }
  /*
  do {
    error = MPS_start_meas();
  }while (error != 0);
  */
  delay(2000);
}

void loop() {
  /**
   *@brief  Get PM1.0 concentration
   *@param  PARTICLE_PM1_0_STANDARD   
   *@n      PARTICLE_PM2_5_STANDARD   
   *@n      PARTICLE_PM10_STANDARD    
   *@n      PARTICLE_PM1_0_ATMOSPHERE 
   *@n      PARTICLE_PM2_5_ATMOSPHERE 
   *@n      PARTICLE_PM10_ATMOSPHERE  
   */  
  uint16_t concentration = particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_STANDARD);
  Serial.print("PM1.0 concentration:");
  Serial.print(concentration);
  Serial.println(" mg/m3");
  //MPS_ans();
  delay(1000);
}

uint16_t crc_generate(uint8_t *buffer, size_t length, uint16_t startValue ) {
  uint16_t crc;
  uint8_t *p;
  int ii;

  crc = startValue;

  for(p = buffer, ii = 0; ii < length; ii++) {
    crc = (crc << 8) ^ crc_table[(crc >> 8) ^ *p];
    p++;
  }

  return crc;

}

uint8_t MPS_status(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  uartRqstHeader_t header;
  uint16_t cksum, rxCksum, length;

  memset(&header, 0, RQST_HDR_LENGTH);
  header.cmdID = cmdID;
  header.length = payloadLen;

  cksum = crc_generate((uint8_t *) &header, RQST_HDR_LENGTH, 0xFFFF);
  header.cksum = cksum;
  Serial.println("MPS status: ");
  for (int i = 0; i < RQST_HDR_LENGTH; i++) {
    Serial.printf("0x%02X ", (uint8_t *) &header);
  }
  Serial.println();
  Serial2.write((uint8_t *) &header, RQST_HDR_LENGTH);
  delay(100);
  Serial.println("MPS status reply: ");
  uint8_t ret;
  while (Serial2.available()) {
    ret = Serial2.read();
    Serial.printf("0x%02X ", ret);
  }
  Serial.println();
  return ret;
}

uint8_t MPS_version(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  uint16_t fwVersion = 0;

  //Get the FW version
  uart_version_t *version;
  uartRqstHeader_t header;
  uint16_t cksum, rxCksum, length;

  memset(&header, 0, RQST_HDR_LENGTH);
  header.cmdID = cmdID;
  header.length = payloadLen;

  cksum = crc_generate((uint8_t *) &header, RQST_HDR_LENGTH, 0xFFFF);
  header.cksum = cksum;
  Serial.println("MPS version: ");
  Serial2.write((uint8_t *) &header, RQST_HDR_LENGTH);
  delay(300);
  int i = 0;
  while(Serial2.available()) {
    Serial.print(i);
    i++;
    Serial.println(Serial2.read());
  }
  /*
  uint8_t *data;
  uint16_t size;
  uartRecv(cmdID, data, size);
  version = (uart_version_t *) data;
  Serial.printf("SW Version: %u.%u.%u.%u\nHW Version: %u.%u\nProtocol: %u.%u\n",
         version->sw_w, version->sw_x, version->sw_y, version->sw_z,
         version->hw_w, version->hw_x, version->proto_w, version->proto_x);
  */
  uint8_t ret = 0;
  return ret;
}

uint32_t uartRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  int32_t retry = 1;
  uint32_t status;

  status = uartSingleRecv(cmdID, payload, payloadLen);
  if((status == UART_SUCCESS) || (numOfRetries == 0))
    return status;

  do {
    status = uartSingleRecv(cmdID, payload, payloadLen);
  } while ((retry++ < numOfRetries) && (status != UART_SUCCESS));

  return status;
}

uint32_t uartSingleRecv(uint8_t cmdID, uint8_t *payload, uint16_t payloadLen) {
  uint16_t rxCksum, cksum;
  int rxLen;
  uint32_t timeout;
  uartRqstHeader_t header;
  uartReplyHeader_t *reply;
  uint8_t buffer[UART_MAX_DATA_SIZE+1];

  memset(buffer, 0, sizeof(buffer));

  rxLen = Serial2.read(buffer, sizeof(uartReplyHeader_t));
  if(rxLen <= 0) {
    printf("Failed to get reply: %s (%d)\n", strerror(errno),  errno);
    return 0xffffffff;
  }

  reply = (uartReplyHeader_t *) buffer;
  if(rxLen < REPLY_HDR_LENGTH) {
    Serial.printf("Incomplete header received: %d bytes\n", rxLen);
    return 0xffffffff;
  }

  if(reply->length != 0) {  /* Is there a payload for this reply? */
    rxLen = Serial2.read(&buffer[REPLY_HDR_LENGTH], reply->length);
    if(rxLen < reply->length) {
      Serial.printf("Failed to get reply payload: %s (%d)\n", strerror(errno),  errno);
      return 0xffffffff;
    }
  }

  rxCksum = reply->cksum;
  reply->cksum = 0;  /* zero out checksum field */
  cksum = crc_generate(buffer, REPLY_HDR_LENGTH + reply->length, 0xFFFF);
  if(rxCksum != cksum) {
    Serial.printf("Checksum failed: expected 0x%x, received 0x%x\n", cksum, rxCksum);
    reply->cksum = rxCksum;   /* restore received checksum */
    return 0xffffffff;
  }

  reply->cksum = rxCksum;   /* restore received checksum */

  if(reply->status != UART_SUCCESS) {
    if(reply->status >= 0x20) {
      printf("Sensor hardware error: 0x%x\n", reply->status);
    } else {
      printf("Command returned error status: 0x%x\n", reply->status);
      return (reply->status);  /* Sensor sent communication error */
    }
  }
  
  if(reply->cmdID != cmdID) {
    printf("cmdID mismatch: expected 0x%x, received 0x%x\n", cmdID, reply->cmdID);
    return 0xffffff;
  }

  if (reply->length == 0)
    return UART_SUCCESS;  /* No payload, we are done. */

  if(payloadLen < reply->length) {
    printf("Buffer too small for payload (%d < %d)\n", payloadLen, reply->length);
    return 0xffffff;
  }

  memset(payload, 0, payloadLen);
  memcpy(payload, &buffer[REPLY_HDR_LENGTH], reply->length);

  return UART_SUCCESS;
}

/*
uint8_t MPS_start_meas() {
  union a16to8 checksumBytes;
  cmdID[1] = 0x61;
  cmdID[0] = 0x00;
  length[1] = 0x00;
  length[0] = 0x01;
  memcpy(buffHeader, cmdID, 2);
  memcpy(buffHeader + 2, length, 2);
  memcpy(buffHeader + 4, reserved, 2);
  checksumBytes.a16 = crc_generate(buffHeader, 6, 0xFFFF);
  memcpy(buffHeader + 6, checksumBytes.a8, 2);
  uint8_t command = 0x12;
  memcpy(buffHeader + 8, &command, 1);
  Serial.println("MPS start meas: ");
  for (int i = 0; i < 9; i++) {
    Serial.printf("0x%02X ", buffHeader[i]);
  }
  Serial.println();
  Serial2.write(buffHeader, 9);
  delay(100);
  Serial.println("MPS start meas reply: ");
  uint8_t ret;
  while (Serial2.available()) {
    ret = Serial2.read();
    Serial.printf("0x%02X ", ret);
  }
  Serial.println();
  ret = 0;
  return ret;
}

uint8_t MPS_ans() {
  union a16to8 checksumBytes;
  cmdID[1] = 0x01;
  cmdID[0] = 0x00;
  length[1] = 0x00;
  length[0] = 0x00;
  memcpy(buffHeader, cmdID, 2);
  memcpy(buffHeader + 2, length, 2);
  memcpy(buffHeader + 4, reserved, 2);
  checksumBytes.a16 = crc_generate(buffHeader, 6, 0xFFFF);
  memcpy(buffHeader + 6, checksumBytes.a8, 2);
  Serial.println("MPS answer: ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("0x%02X ", buffHeader[i]);
  }
  Serial.println();
  Serial2.write(buffHeader, 8);
  delay(100);
  Serial.println("MPS answer reply: ");
  uint8_t ret;
  while (Serial2.available()) {
    ret = Serial2.read();
    Serial.printf("0x%02X ", ret);
  }
  Serial.println();
  ret = 0;
  return ret;
}
*/