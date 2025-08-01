[![Website](https://img.shields.io/badge/Website-Link-blue.svg)](https://gavinlyonsrepo.github.io/)  [![Rss](https://img.shields.io/badge/Subscribe-RSS-yellow.svg)](https://gavinlyonsrepo.github.io//feed.xml)  [![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/paypalme/whitelight976)

# BMP280_LTSM README

## Table of contents

## Table of contents

- [Overview](#overview)
- [Installation](#installation)
- [Software](#software)
	- [Default settings](#default-settings)
	- [Power modes](#power-modes)
	- [Debug mode](#debug-mode)
	- [API documentation](#api-documentation)
- [Hardware](#hardware)
	- [SPI connections](#spi-connections)
	- [I2C connections](#i2c-connections)
- [Output](#output)


## Overview

* Name: BMP280_LTSM
* Description:

Arduino Library for Bosch BMP280 Digital pressure sensor.

* Supports sensors features:

1. Read pressure data
2. Read temperature data
3. Tested on SPI interface and I2C interface, Interface is selected by user constructor overload(see examples)
4. Oversampling settings, standby times, Filter settings and can be set thru API.
5. Normal mode, sleep mode and forced mode supported.
6. This supports the BMP280 sensor fully only. The humidity functionality of the BME280 is not tested or supported.
7. 3 Examples files 2 for SPI and 1 for I2C.
8. Chip ID should be 0x56-0X58 for BMP280, 0x60 BME280. 
9. Hardware SPI and I2C supported.

* [Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)

## Installation

The library is included in the official Arduino library manger 
and the optimum way to install it is using the library manager 
which can be opened by the *manage libraries* option in Arduino IDE. 

## Software

### Default settings

| BMP280 Setting | Default Enumeration |
|------------|----------|
| Oversampling Temperature | Sampling_X16 (5)|
| Oversampling Pressure    | Sampling X2 (2)  |
| Standby Duration         | StandBy_MS_1 (0) |
| Filter                   | Filter_Off (0)|
| Power mode               | Normal (2) |

### Power modes 

BMP280 can be operated in three power modes.

* sleep mode
* normal mode
* forced mode

In sleep mode, no measurements are performed. Normal mode comprises an automated
perpetual cycling between an active measurement period and an inactive standby period. In
forced mode, a single measurement is performed. When the measurement is finished, the
sensor returns to sleep mode.

### Debug mode

Debug mode with serial messages on 38400 baud can be enabled.

```
// --- Debug settings ---
#define BMP280_DEBUG 0 /*! Enable debug messages , 38400 baud, set to 1 to enable debug messages*/
```

### API documentation

The code is commented for doxygen and an application programming interface can be created using the doxygen software program.

## Hardware

The Sensor uses SPI or I2C for communication's. GPIO numbers in tables below is for ESP32,
adjust for your arduino MCU type.
The BMP280 is a 3.3V device.

![ Pinout](https://github.com/gavinlyonsrepo/sensors_PICO/blob/main/extra/images/bmp280.jpg)

### SPI connections

The BMP280 can be connected to the MCU using SPI. 
The following table shows the pin connections between the BMP280 and MCU(ESP32 in this case).
The BMP280 has a CSB pin which can be connected to any GPIO pin. 
Can be set up for any SPI interface and bus speed by changing SPI settings in header file.

| BMP280 Pin | Function | ESP32| Notes        |
|------------|----------|-----------|--------------|
| CSB        | Chip Select (CS) | GPIO 4   | pick any GPIO u want |
| SDA        | MOSI (Data In)   | GPIO 23 MOSI VSPI  | Master Out Slave In |
| SCL        | SCK (Clock)      | GPIO 18 CLK VSPI  | SPI Clock |
| SDO        | MISO (Data Out)  | GPIO 19 MISO VSPI  | Master In Slave Out |

### I2C connections

The BMP280 can be connected to the MCU using I2C. 
The following table shows the pin connections between the BMP280 and the MCU(ESP32 in this case).
The BMP280 has a CSB pin which should be connected to 3.3V not left floating. 
The CSB pin is active low, so it should be pulled high when not in use. 
The I2C address of the BMP280 is 0x76 or 0x77 depending on the SDO pin connection.

| BMP280 Pin | Function | ESP32 GPIO | Notes        |
|------------|----------|-----------|--------------|
| SDA        | Data     | GPIO 21   | I2C DATA |
| SCL        | Clock   |  GPIO 22   | I2C Clock |
| CSB        | Chip Select  | n/a  | set high |
| SDO        | MISO   | n/a  | used to select I2C address, If SDO is high then the I2C address is 0x77. If SDO is low the I2C address is 0x76. |

## Output

![ op](https://github.com/gavinlyonsrepo/sensors_PICO/blob/main/extra/images/bmpoutput.png)
