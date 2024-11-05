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

uint8_t cmdID[2];
uint8_t length[2];
uint8_t reserved[2] = {0x00 , 0x00};
uint8_t checksum[2];
uint8_t payload[10];
uint8_t buffHeader[18];
union a16to8 {
	 uint16_t a16;
	 uint8_t a8[2];
};
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
  union a16to8 checksumBytes;
  cmdID[1] = 0x41;
  cmdID[0] = 0x00;
  length[1] = 0x00;
  length[0] = 0x00;
  memcpy(buffHeader, cmdID, 2);
  memcpy(buffHeader + 2, length, 2);
  memcpy(buffHeader + 2, reserved, 2);
  checksumBytes.a16 = crc_generate(buffHeader, 0, 0xFFFF);
  memcpy(buffHeader + 2, checksumBytes.a8, 2);
  Serial.println("MPS status: ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("0x%02X ", buffHeader[i]);
  }
  Serial.println();
  Serial2.write(buffHeader, 8);
  delay(100);
  Serial.println("MPS status reply: ");
  while (Serial2.available()) {
    Serial.printf("0x%02X ", Serial2.read());
  }
  Serial.println();
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
  delay(1000);
}