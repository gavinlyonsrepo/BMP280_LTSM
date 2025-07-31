/*!
	@file I2C_Normal
	@brief arduino C++ bmp280 library test file, basic use, normal mode , I2C hardware
	@details bmp280 is a digital pressure sensor with temperature measurement capabilities.
  @author LionTron Systems
*/

#include "bmp280_ltsm.hpp"

#define I2C_ADDRESS 0x76 // I2C address of the sensor, try 0x76 or 0x77

BMP280_Sensor bmp280(I2C_ADDRESS, &Wire);

void setup(){
  Serialinit();
}

void loop() {
	Serial.println("--- START Normal I2C ---");
	while (!bmp280.InitSensor())
  {
      delay(3000);
      Serial.println("- BMP280 sensor not connected  -");
  }

	// while (bmp280.CheckConnectionI2C() < 0) {
	// 	Serial.println("Failed to connect to BMP280 sensor");
	// 	delay(3000);
	// }
	uint8_t chipID = 0;
	chipID = bmp280.readForChipID();

	Serial.print("Chip ID: "); // Should read 0x56 - 0x58 for BMP280 060 for BME280
  Serial.println(chipID, HEX);
	uint16_t counter = 0;
	while(counter < 60)
	{
		
    Serial.print("Test Count: ");
    Serial.println(counter, DEC);
		// Test 1 Temperature 
		Serial.print("Temperature oversampling: ");
		Serial.println(static_cast<uint8_t>(bmp280.readOversampling(BMP280_Sensor::DataType_e::Temperature)));
    Serial.print("Temperature: ");
		Serial.println(bmp280.readTemperature(), 2);
		// Test 2 Pressure
		Serial.print("Pressure oversampling: ");
		Serial.println(bmp280.readPressure(BMP280_Sensor::PressureUnit_e::hPa), 2);

		Serial.println("");
		delay(3000);
		counter++;
	}
	Serial.println("--- END ---");
  while(1){};
}

//Function to setup serial called from setup FOR debug
void Serialinit()
{
  Serial.begin(38400);
  delay(1000);
  Serial.println("--Comms UP--BMP_280.ino--");
}