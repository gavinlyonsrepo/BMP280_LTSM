/*!
	@file SPI_Forced.ino
	@brief arduino C++ bmp280 library test file, basic use, forced mode , SPI hardware
	@details bmp280 is a digital pressure sensor with temperature measurement capabilities.
            connections for ESP32 , D23 = MOSI(SDA) , D19 = MISO(SDO), D18 = SCLK , User defined = CS(CSB)
            In forced mode, a single measurement is performed. When the measurement is finished, the
            sensor returns to sleep mode.
  @author Gavin lyons at LionTron Systems
*/

#include "bmp280_ltsm.hpp"

uint8_t ChipSelectPin = 4;  // User defined chip select pin, D4
BMP280_Sensor bmp280(ChipSelectPin);

void setup() {
  Serialinit();
}

void loop() {
  printf("\n--- START Forced SPI---\n");

  bmp280.InitSensor();

  uint8_t chipID = 0;
  chipID = bmp280.readForChipID();
  Serial.print("CHIP ID: ");
  Serial.println(chipID, HEX);  // Should read 0x56-0x58 for BMP280 060 for BME280
  delay(2000);

  bmp280.setPowerMode(BMP280_Sensor::PowerMode_e::Forced); //Set forced mode ON

  uint16_t counter = 0;
  while (counter < 60) {
    Serial.print("Test Count: ");
    Serial.println(counter, DEC);
    if (bmp280.takeForcedMeasurement()) {
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
      delay(3000);
      counter++;
    } else {
      Serial.println("Failed to take forced measurement");
      delay(2000);
      break;
    }
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