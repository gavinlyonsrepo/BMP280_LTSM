/*!
	@file I2C_Normal.ino
	@brief arduino C++ bmp280 library test file, basic use, normal mode , I2C hardware
	@details bmp280 is a digital pressure sensor with temperature measurement capabilities.
          Connection ESP32 I2C SCLK = D22 , SDATA = D21
          Normal mode comprises an automated
          perpetual cycling between an active measurement period and an inactive standby period.
  @author Gavin lyons at LionTron Systems
*/

#include "bmp280_ltsm.hpp"

#define I2C_ADDRESS 0x76  // I2C address of the sensor, try 0x76 or 0x77
BMP280_Sensor bmp280(I2C_ADDRESS, &Wire);

void setup() {
  Serialinit();
}

void loop() {
  Serial.println("--- START Normal I2C ---");
  while (!bmp280.InitSensor()) {
    delay(3000);
    Serial.println("- BMP280 sensor not connected  -");
  }

  // optional
  // while (bmp280.CheckConnectionI2C() < 0) {
  // 	Serial.println("Failed to connect to BMP280 sensor");
  // 	delay(3000);
  // }

  uint8_t chipID = 0;
  chipID = bmp280.readForChipID();
  Serial.print("CHIP ID: ");
  Serial.println(chipID, HEX);  // Should read 0x56-0x58 for BMP280, 0x60 for BME280
  delay(2000);

  uint16_t counter = 0;
  while (counter < 60) {
    Serial.print("Test Count: ");
    Serial.println(counter, DEC);
    // Test 1 Temperature
    Serial.print("Temperature oversampling: ");
    Serial.println(static_cast<uint8_t>(bmp280.readOversampling(BMP280_Sensor::DataType_e::Temperature)));
    Serial.print("Temperature: [C] = ");
    Serial.println(bmp280.readTemperature(), 2);
    // Test 2 Pressure
    Serial.print("Pressure oversampling: ");
    Serial.println(static_cast<uint8_t>(bmp280.readOversampling(BMP280_Sensor::DataType_e::Pressure)));
    Serial.print("Pressure: [hPa] = ");
    Serial.println(bmp280.readPressure(BMP280_Sensor::PressureUnit_e::hPa), 2);
    Serial.println("");
    // Test 2C Altitude adjustment, optional
    //uint16_t altMets = 95; // altitude in meters
    //Serial.print("Pressure adjusted: [hPa]");
    //Serial.println(bmp280.seaLevelForAltitude(altMets, bmp280.readPressure(BMP280_Sensor::PressureUnit_e::hPa)));
    delay(3000);
    counter++;
  }
  Serial.println("--- END ---");
  while (1) {};
}

//Function to setup serial called from setup
void Serialinit() {
  Serial.begin(38400);
  delay(1000);
  Serial.println("--Comms UP BMP280--");
}