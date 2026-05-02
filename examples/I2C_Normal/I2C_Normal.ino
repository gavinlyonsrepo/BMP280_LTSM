/*!
	@file    I2C_Normal.ino
	@brief   arduino C++ bmp280 library test file, basic use, normal mode , I2C hardware
	@details bmp280 is a digital pressure sensor with temperature measurement capabilities.
           Connection ESP32 I2C SCLK = D22 , SDATA = D21, CS-HIGH, SDO-set I2C address,
           0x76:SDO low ,0x77:SD0 high.
           Normal mode comprises an automated
           perpetual cycling between an active measurement period and an inactive standby period.
  @author Gavin lyons at LionTron Systems
*/

#include "bmp280_ltsm.hpp"

uint8_t  I2C_ADDRESS = 0x76;  // I2C address sensor, 0x76:SDO low ,0x77:SD0 high
uint32_t I2C_BUS_SPEED = 100000;
BMP280_Sensor bmp280(I2C_ADDRESS, &Wire, I2C_BUS_SPEED);
// Local Pressure , Replace with today's QNH from forecast [hPa]
#define LOCAL_PRESSURE  1013.25  

void setup() {
  Serialinit();
  SensorInit();
}

void loop() {
  PrintSensorInfo();
  Serial.println("--- END ---");
  while (1) {};
}

//Function to setup serial called from setup
void Serialinit() {
  Serial.begin(38400);
  delay(1000);
  Serial.println("--Comms UP BMP280--");
}

void SensorInit(){
  Serial.println("--- START Normal I2C ---");
  while (!bmp280.InitSensor()) {
    delay(3000);
    Serial.println("- BMP280 sensor not connected  -");
  }
  // optional
  while (bmp280.CheckConnectionI2C() < 0) {
  	Serial.println("Failed to connect to BMP280 sensor");
  	delay(3000);
  }
  uint8_t chipID = 0;
  chipID = bmp280.readForChipID();
  Serial.print("CHIP ID: ");
  Serial.println(chipID, HEX);  // Should read 0x56-0x58 for BMP280, 0x60 for BME280
  delay(2000);
}

// Print Sensor info
void PrintSensorInfo(){
  uint16_t counter = 0;
  while (counter < 60) {
    Serial.print("Test Count: ");
    Serial.println(counter, DEC);
    Serial.println("---");
    // Test 1 Temperature
    Serial.print("Temperature oversampling: ");
    Serial.println(static_cast<uint8_t>(bmp280.readOversampling(BMP280_Sensor::DataType_e::Temperature)));
    Serial.print("Temperature: [C] = ");
    Serial.println(bmp280.readTemperature(), 2);
    Serial.println("---");
    // Test 2 Pressure
    Serial.print("Pressure oversampling: ");
    Serial.println(static_cast<uint8_t>(bmp280.readOversampling(BMP280_Sensor::DataType_e::Pressure)));
    Serial.print("Pressure: [hPa] = ");
    Serial.println(bmp280.readPressure(BMP280_Sensor::PressureUnit_e::hPa), 2);
    Serial.println("---");
    //Test 3 Altitude adjustment optional
    Serial.print("altitude: [meters]  = ");
    double altMets = bmp280.readAltitude(LOCAL_PRESSURE); // altitude in meters method 1
    //double altMets = 96;                                // altitude in meters method 2
    Serial.println(altMets);
    Serial.print("Pressure adjusted: [hPa  = ");
    Serial.println(bmp280.seaLevelForAltitude(altMets, bmp280.readPressure(BMP280_Sensor::PressureUnit_e::hPa)));
    Serial.println("---");
    delay(3000);
    counter++;
  }
}