/*!
	@file bmp280.cpp
	@brief source header file for bmp280 pressure sensor 
*/

#include "bmp280_ltsm.hpp"

/*!
	@brief Constructor.
	@param spi Instance of SPI.
	@param baudRate SPI speed in Hertz
	@param cs Chip-select Pin.
	@param mosi Master out slave in pin.
	@param sck Serial clock pin.
	@param miso Master in slave out pin.
*/
// BMP280_Sensor::BMP280_Sensor(spi_inst_t *spi, uint32_t baudRate, uint8_t cs, uint8_t mosi, uint8_t sck, uint8_t miso)
// {
// 	_spiInst = spi;
// 	_baudRate = baudRate;
// 	_cs = cs;
// 	_mosi = mosi;
// 	_sck = sck;
// 	_miso = miso;
// 	_commMode = CommMode_e::SPI;
// }

/*!
	@brief Constructor I2C mode.
	@param address 
	@param twi
*/
BMP280_Sensor::BMP280_Sensor(uint8_t address, TwoWire *twi){
	_address  = address;
	wire = twi;
	_commMode = CommMode_e::I2C;
}

/*! 
	@brief Init hardware
*/
bool BMP280_Sensor::InitSensor(void)
{

	if (_commMode == CommMode_e::I2C)
	{
			int I2CReturnCode= 0;
			wire->begin();
			wire->beginTransmission(_address);
			I2CReturnCode = wire->endTransmission();
			if (I2CReturnCode!= 0)
				{
					#if BMP280_DEBUG
						Serial.print("1201 : ");
						Serial.print("I2C error  wire.endTransmission : ");
						Serial.print("I2CReturnCode: ");
						Serial.println(I2CReturnCode);
					#endif
					return false;      //Check if the sensor  is connected
				}else{
					#if BMP280_DEBUG
						Serial.print("I2C Success Init : ");
					#endif
				}
	} else {
		// SPI communication not implemented in this version
	}
	delay(100); 
	StartUpRoutine();
	delay(100);
	return true;
}


/*! 
	@brief start up routine
	@details Set default power mode, oversampling and get trimming parameters.
*/
void BMP280_Sensor::StartUpRoutine(void)
{
	// Set default power mode.
	setPowerMode(PowerMode_e::Sleep);
	getTrimmingParameters();
	// Set default oversampling.
	setOversampling(DataType_e::Temperature, sensorSampling_e::Sampling_X16);
	setOversampling(DataType_e::Pressure, sensorSampling_e::Sampling_X2);
	setPowerMode(PowerMode_e::Normal);
}

/*!
 * @brief Default destructor.
 */
BMP280_Sensor::~BMP280_Sensor() {
	// Destructor body
}

/*!
 * @brief Read chip ID register
 * @return Chip ID value 0x56 - 0x58 for BMP280, 0x60 for BME280
 */
uint8_t BMP280_Sensor::readForChipID()
{
	uint8_t result = readRegister(Chip_ID);
	_chipID = result;
	return result;
}

/*!
	@brief Getter for chip ID variable member 
	@return Chip ID value 0x56 - 0x58 for BMP280, 0x60 for BME280
*/
uint8_t BMP280_Sensor::getChipID() const
{
	return _chipID;
}

/*!
	@brief Read register value from sensor.
		Function reads 1 or 3 registers of data from sensor.
	@param reg Type of register.
	@param threeRegsRead If true, function reads 3 bytes of data from sensor, if false, only 1 byte.
	@return Value of registers, will return -1 if I2C error occurs
*/
int32_t BMP280_Sensor::getData(Registers_e reg, bool threeRegsRead)
{
	uint8_t buffer[3] = {0x00, 0x00, 0x00};
	int32_t result = 0;
	uint8_t regVal = static_cast<uint8_t>(reg);
	if (_commMode == CommMode_e::SPI)
	{
		// SPI communication not implemented in Arduino version
		// You can implement using Arduino SPI functions if needed
	}
	else
	{
		// Write register address
		Wire.beginTransmission(_address);
		Wire.write(regVal);
		int returnValue = Wire.endTransmission(false); // repeated start
		if (returnValue != 0)
		{
			#if BMP280_DEBUG
			Serial.print("I2C write error: getData ");
			Serial.println(returnValue);
			#endif
			return -1;
		}
		if (threeRegsRead)
		{
			int bytesRead = Wire.requestFrom((int)_address, 3);
			if (bytesRead < 3)
			{
				#if BMP280_DEBUG
				Serial.print("I2C read error: getData ");
				Serial.println(bytesRead);
				#endif
				return -1;
			}
			buffer[0] = Wire.read();
			buffer[1] = Wire.read();
			buffer[2] = Wire.read();

			result = ((uint32_t)buffer[0] << 12) |
			         ((uint32_t)buffer[1] << 4) |
			         ((uint32_t)buffer[2] >> 4);
		}
		else
		{
			int bytesRead = Wire.requestFrom((int)_address, 1);
			if (bytesRead < 1)
			{
				#if BMP280_DEBUG
				Serial.print("I2C read error: getData ");
				Serial.println(bytesRead);
				#endif
				return -1;
			}
			buffer[0] = Wire.read();
			result = buffer[0];
		}
	}
	return result;
}


/*!
	@brief Set value of specified register. After setting check parameter to true,
		sends command to read value of this register. Default value of check
		is set to False. If check is set to false,
		function will always return true, and register's value won't be checked.
	@param reg Type of register.
	@param config Value to put into register.
	@param check Default value is False. True - check if value was set, False - dont check.
	@return true if register was set with given value, otherwise false.
*/
bool BMP280_Sensor::setRegister(Registers_e reg, uint8_t config, bool check)
{
	uint8_t regVal = static_cast<uint8_t>(reg);
	if (_commMode == CommMode_e::I2C)
	{
		Wire.beginTransmission(_address);
		Wire.write(regVal);
		Wire.write(config);
		int returnValue = Wire.endTransmission(); // send with stop
		if (returnValue != 0)
		{
			#if BMP280_DEBUG
			Serial.print("I2C write error: setRegister ");
			Serial.println(returnValue);
			#endif
			return false;
		}
	}
	else
	{
		// SPI communication not implemented yet
		// Use Arduino SPI library if needed
	}

	if (!check)
		return true;

	uint8_t value = readRegister(reg);
	return (value == config);
}

/*!
	@brief Read value of specified register via command.
	@param reg Type of register.
	@return Value of specified register., return 0xFF if I2C error occurs
*/
uint8_t BMP280_Sensor::readRegister(Registers_e reg)
{
	uint8_t buffer = 0;
	if (_commMode == CommMode_e::I2C)
	{
		uint8_t regVal = static_cast<uint8_t>(reg);

		// Write the register address with a repeated start
		Wire.beginTransmission(_address);
		Wire.write(regVal);
		int returnValue = Wire.endTransmission(false); // repeated start, no stop
		if (returnValue != 0)
		{
			#if BMP280_DEBUG
			Serial.print("I2C write error: readRegister ");
			Serial.println(returnValue);
			#endif
			return 0xFF;
		}
		// Request 1 byte from the register
		int bytesRead = Wire.requestFrom((int)_address, 1);
		if (bytesRead < 1)
		{
			#if BMP280_DEBUG
			Serial.print("I2C read error: readRegister ");
			Serial.println(bytesRead);
			#endif
			return 0xFF;
		}
		buffer = Wire.read();
	}
	else
	{
		// SPI communication not implemented in this version
	}

	return buffer;
}

/*!
	@brief Set power mode of BMP280 sensor.
	@param mode PowerMode enum to set.
	@param check Default value is set to false. True - check if value was set, False - dont check.
	@return true if action succeeded, otherwise false.
*/
bool BMP280_Sensor::setPowerMode(PowerMode_e mode, bool check)
{
	static constexpr uint8_t CTRL_MEAS_MASK_PWRMODE = 0xFC; // Bits [1:0] = 00 (mask for power mode)
	uint8_t config = readRegister(Ctrl_Meas);
	switch (mode)
	{
	case PowerMode_e::Sleep:
		config = ((config & CTRL_MEAS_MASK_PWRMODE ) | 0x00); 
		break;
	case PowerMode_e::Normal:
		config = ((config & CTRL_MEAS_MASK_PWRMODE ) | 0x03);
		break;
	case PowerMode_e::Forced:
		config = ((config & CTRL_MEAS_MASK_PWRMODE ) | 0x02);
		break;
	}
	bool value = setRegister(Ctrl_Meas, config, check);
	if (value == true)
	{
		_powerMode = mode;
	}
	return value;
}

/*!
	@brief Get current power mode via sending SPI command.
	@return PowerMode enum.
*/
BMP280_Sensor::PowerMode_e BMP280_Sensor::readPowerMode()
{
	static constexpr uint8_t CTRL_MEAS_MASK_PWRMODE = 0xFC; // Bits [1:0] = 00 (mask for power mode)
	uint8_t data = readRegister(Ctrl_Meas);
	data &= ~CTRL_MEAS_MASK_PWRMODE;
	PowerMode_e mode;
	switch (data)
	{
		case 0: mode = PowerMode_e::Sleep; break;
		case 3: mode = PowerMode_e::Normal; break;
		case 2: mode = PowerMode_e::Forced; break;
		case 1: mode = PowerMode_e::Forced; break;
		default: mode = PowerMode_e::Normal;
	}
	_powerMode = mode;
	return mode;
}

/*!
	@brief Get PowerMode value of object variable.
	@return PowerMode enum.
*/
BMP280_Sensor::PowerMode_e BMP280_Sensor::getPowerMode() const
{
	return _powerMode;
}

/*!
	@brief Set oversampling of BMP280 sensor via SPI command.
	@param type Temperature or Pressure.
	@param oversampling Sensor oversampling value.
		0x00 - 0x05, 0x00 - no oversampling, 0x01 - 1x oversampling, 0x02 - 2x oversampling,
		0x03 - 4x oversampling, 0x04 - 8x oversampling, 0x05 - 16x oversampling.
	@param check Default value is set to false. True - check if value was set, False - dont check.
	@return True if register was set with given value, otherwise False.
*/
bool BMP280_Sensor::setOversampling(DataType_e type, sensorSampling_e oversampling, bool check)
{
	static constexpr uint8_t CTRL_MEAS_MASK_T_OS = 0x1F; // Preserve bits [4:0], clear [7:5] (temperature OS)
	static constexpr uint8_t CTRL_MEAS_MASK_P_OS = 0xE3; // Preserve bits [7:5],[1:0], clear [4:2] (pressure OS)
	uint8_t registerValue = readRegister(Ctrl_Meas);
	uint8_t osVal = static_cast<uint8_t>(oversampling);

	//Validate enum range (0x00 to 0x05)__
	if (osVal > static_cast<uint8_t>(sensorSampling_e::Sampling_X16))
	{
		return false;
	}
	uint8_t value = osVal;
	switch (type)
	{
	case DataType_e::Temperature:
		value <<= 5;
		registerValue = (registerValue & CTRL_MEAS_MASK_T_OS) | value; // Preserve ctrl_meas bits values for temperature oversampling and power mode
		break;

	case DataType_e::Pressure:
		value <<= 2;
		registerValue = (registerValue & CTRL_MEAS_MASK_P_OS) | value; // Preserve  ctrl_meas bits values for pressure oversampling and power mode
		break;
	}

	bool result = setRegister(Ctrl_Meas, registerValue, check);
	if (result)
	{
		switch (type)
		{
		case DataType_e::Temperature:
			_temperatureOversampling = oversampling;
			break;
		case DataType_e::Pressure:
			_pressureOversampling = oversampling;
			break;
		}
	}
	delay(OverSamplingDelay);
	return result;
}


/*!
	@brief Read current temperature oversampling via SPI command.
	@param type Type::Temperature or Type::Pressure.
	@return Value of the oversampling setting.
*/
BMP280_Sensor::sensorSampling_e BMP280_Sensor::readOversampling(DataType_e type)
{
	static constexpr uint8_t CTRL_MEAS_MASK_P_OS_READ = 0x1C; // Bits [4:2] pressure OS
	static constexpr uint8_t CTRL_MEAS_MASK_T_OS_READ = 0xE0; // Bits [7:5] temperature OS
	uint8_t reg = readRegister(Ctrl_Meas);
	sensorSampling_e result = sensorSampling_e::Sampling_None;
	switch (type)
	{
	case DataType_e::Pressure:
		reg &=  CTRL_MEAS_MASK_P_OS_READ; // Mask off Bit 4, 3, 2 osrs_p[2:0] Ctrl Meas register
		reg >>= 2;
		switch (reg)
		{
			case 0: result = sensorSampling_e::Sampling_None; break;
			case 1: result = sensorSampling_e::Sampling_X1; break;
			case 2: result = sensorSampling_e::Sampling_X2; break;
			case 3: result = sensorSampling_e::Sampling_X4; break;
			case 4: result = sensorSampling_e::Sampling_X8; break;
			default: result = sensorSampling_e::Sampling_X16; break;
		}
		_pressureOversampling = result;
		return result;
		break;
	case DataType_e::Temperature:
		reg &= CTRL_MEAS_MASK_T_OS_READ; // Mask off Bit 7, 6, 5 osrs_t[2:0] Ctrl Meas register
		reg >>= 5;
		switch (reg)
		{
			case 0: result = sensorSampling_e::Sampling_None; break;
			case 1: result = sensorSampling_e::Sampling_X1; break;
			case 2: result = sensorSampling_e::Sampling_X2; break;
			case 3: result = sensorSampling_e::Sampling_X4; break;
			case 4: result = sensorSampling_e::Sampling_X8; break;
			case 5:
			case 6:
			case 7:
				result = sensorSampling_e::Sampling_X16;
				break;
		}
		_temperatureOversampling = result;
		return result;
		break;
	default:
		#if BMP280_DEBUG
		Serial.println("BMP280_Sensor::readOversampling: Invalid type");
		#endif
		return sensorSampling_e::Sampling_None;
		break;
	}
}

/*!
	@brief Get oversampling value of object variable.
	@param type Temperature or Pressure.
	@return Oversampling config.
*/
BMP280_Sensor::sensorSampling_e BMP280_Sensor::getOversampling(DataType_e type) const
{
	switch (type)
	{
	case DataType_e::Temperature:
		return _temperatureOversampling;
		break;
	case DataType_e::Pressure:
		return _pressureOversampling;
		break;
	default:
		#if BMP280_DEBUG
		Serial.println("BMP280_Sensor::getOversampling: Invalid type");
		#endif
		return sensorSampling_e::Sampling_None;
		break;
	}
}

/*!
	@brief Read current temperature in raw format via SPI command.
	@return Temperature in raw format.
*/
int32_t BMP280_Sensor::readRawTemperature()
{
	// Based of datasheet formula.
	int32_t adc_T = getData(Temp_MSB, true);
	int32_t var1, var2, temp;
	var1 = ((((adc_T >> 3) - ((int32_t)calib_data_t.dig_T1 << 1))) * ((int32_t)calib_data_t.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)calib_data_t.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data_t.dig_T1))) >> 12) * ((int32_t)calib_data_t.dig_T3)) >> 14;
	_tFine = var1 + var2;
	temp = (_tFine * 5 + 128) >> 8;
	_rawTemperature = temp;
	return temp;
}

/*!
 @brief Get temperature of object variable.
 @return Temperature in raw format.
*/
int32_t BMP280_Sensor::getRawTemperature() const
{
	return _rawTemperature;
}

/*!
	@brief Read temperature in celsius deg double format via SPI command.
	@return Temperature in celsius deg double.
*/
double BMP280_Sensor::readTemperature()
{
	double temp = readRawTemperature() / 100.0;
	_temperature = temp;
	return (readRawTemperature() / 100.0);
}

/*!
	@brief Get temperature of object variable.
	@return Temperature in celsius deg double.
*/
double BMP280_Sensor::getTemperature() const
{
	return _temperature;
}

/*!
	@brief Read current pressure in raw format
	@return Pressure in raw format.
*/
uint32_t BMP280_Sensor::readRawPressure()
{
	// Based on datasheet formula.
	int32_t adc_P = getData(Press_MSB, true);
	int64_t var1, var2, p;
	var1 = ((int64_t)_tFine) - 128000;
	var2 = var1 * var1 * (int64_t)calib_data_t.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calib_data_t.dig_P5) << 17);
	var2 = var2 + (((int64_t)calib_data_t.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)calib_data_t.dig_P3) >> 8) + ((var1 * (int64_t)calib_data_t.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data_t.dig_P1) >> 33;
	if (var1 == 0)
	{
		return 0;
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)calib_data_t.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)calib_data_t.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data_t.dig_P7) << 4);
	_rawPressure = (uint32_t)p;
	return _rawPressure;
}

/*!
 @brief Get pressure of object variable.
 @return Pressure in raw format.
*/
uint32_t BMP280_Sensor::getRawPressure() const
{
	return _rawPressure;
}

/*!	
 @brief Read pressure in celsius deg double format via SPI command.
 @param unit If != 0 method returns pressure in [hPa], returns pressure in [Pa] by default.
 @return Pressure [Pa] or [hPa] in double.
*/
double BMP280_Sensor::readPressure(PressureUnit_e unit)
{
	double pressure = readRawPressure() / 256;
	_pressure = pressure;
	return (unit == PressureUnit_e::Pa ? pressure : (pressure / 100.0));
}


/*!
	@brief Get pressure of object variable.
	@param unit If != 0 method returns pressure in [hPa], returns pressure in [Pa] by default.
	@return Pressure [Pa] or [hPa] in double.
*/
double BMP280_Sensor::getPressure(PressureUnit_e unit) const
{
		return (unit == PressureUnit_e::Pa ? _pressure : (_pressure / 100.0));
}


/*!
 @brief Power on reset of BMP280 module.
 @details  If the value 0xB6 is written to the register, the device is reset using the complete power-on-reset procedure.
*/
void BMP280_Sensor::reset()
{
	setRegister(Reset, 0xB6);
}

#include <Wire.h>

bool BMP280_Sensor::getTrimmingParameters()
{
	uint8_t buffer[24] = {};
	uint8_t regVal = DIG_T1_Reg;

	if (_commMode == CommMode_e::I2C)
	{
		// Write the starting register address with repeated start
		Wire.beginTransmission(_address);
		Wire.write(regVal);
		int returnValue = Wire.endTransmission(false); // repeated start
		if (returnValue != 0)
		{
			#if BMP280_DEBUG
			Serial.print("I2C write error: getTrimmingParameters ");
			Serial.println(returnValue);
			#endif
			return false;
		}
		// Read 24 bytes of trimming data
		int bytesRead = Wire.requestFrom((int)_address, 24);
		if (bytesRead < 24)
		{
			#if BMP280_DEBUG
			Serial.print("I2C read error: getTrimmingParameters ");
			Serial.println(bytesRead);
			#endif
			return false;
		}

		for (uint8_t i = 0; i < 24; ++i)
			buffer[i] = Wire.read();
	}
	else
	{
		// SPI communication not implemented
	}

	calib_data_t.dig_T1 = (buffer[1] << 8) | buffer[0];
	calib_data_t.dig_T2 = (buffer[3] << 8) | buffer[2];
	calib_data_t.dig_T3 = (buffer[5] << 8) | buffer[4];
	calib_data_t.dig_P1 = (buffer[7] << 8) | buffer[6];
	calib_data_t.dig_P2 = (buffer[9] << 8) | buffer[8];
	calib_data_t.dig_P3 = (buffer[11] << 8) | buffer[10];
	calib_data_t.dig_P4 = (buffer[13] << 8) | buffer[12];
	calib_data_t.dig_P5 = (buffer[15] << 8) | buffer[14];
	calib_data_t.dig_P6 = (buffer[17] << 8) | buffer[16];
	calib_data_t.dig_P7 = (buffer[19] << 8) | buffer[18];
	calib_data_t.dig_P8 = (buffer[21] << 8) | buffer[20];
	calib_data_t.dig_P9 = (buffer[23] << 8) | buffer[22];

	return true;
}


/*!
* @brief Read the sensor's config register into _configReg.
* @return True if read was successful.
*/
bool BMP280_Sensor::readConfig() {
	uint8_t value = readRegister(Registers_e::Config);
	_configReg.timeSb = (value >> 5) & 0x07;
	_configReg.filter = (value >> 2) & 0x07;
	_configReg.spi3w_en = value & 0x01;
#ifdef BMP280_DEBUG
	printf("BMP280 Config Read Register: 0x%02X\n", value);
	printf("  Standby Time       : 0x%X (%u)\n", _configReg.timeSb, _configReg.timeSb);
	printf("  Filter Setting     : 0x%X (%u)\n", _configReg.filter, _configReg.filter);
	printf("  SPI 3-wire enabled : %s\n", _configReg.spi3w_en ? "Yes" : "No");
#endif
	return true;
}

/*!
 * @brief Set the sensor's config register with standby, filter, and SPI3W settings.
 * @param standby Standby duration.
 * @param filter Filter level.
 * @param spi3wEn Enable 3-wire SPI (true to enable).
 * @return True if write was successful.
 */
bool BMP280_Sensor::writeConfig(StandBy_e standby, Filter_e filter, bool spi3wEn) {
	_configReg.timeSb    = static_cast<uint8_t>(standby) & 0x07;
	_configReg.filter    = static_cast<uint8_t>(filter) & 0x07;
	_configReg.spi3w_en  = spi3wEn ? 1 : 0;
	_configReg.none      = 0;

	uint8_t value = _configReg.get();
	bool ok = setRegister(Registers_e::Config, value);
#ifdef BMP280_DEBUG
	printf("BMP280 Config Register Write: 0x%02X\n", value);
	printf("  Standby Time      : 0x%X (%u)\n", _configReg.timeSb, _configReg.timeSb);
	printf("  Filter Setting    : 0x%X (%u)\n", _configReg.filter, _configReg.filter);
	printf("  SPI 3-wire enabled: %s\n", _configReg.spi3w_en ? "Yes" : "No");
#endif
	return ok;
}

/*!
 * @brief Calculates the approximate altitude using barometric pressure and the
 * 	supplied sea level hPa as a reference.
 * @param seaLevelhPa The current hPa at sea level.
 * @return The approximate altitude above sea level in meters.
 */
double BMP280_Sensor::readAltitude(double seaLevelhPa) {
	double altitude;
  
	double pressure = readPressure(); // in Si units for Pascal
	pressure /= 100;
  
	altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
  
	return altitude;
  }
  
  /*!
   * @brief Calculates the pressure at sea level (QNH) from the specified altitude,
   * and atmospheric pressure (QFE).
   * @param  altitude      Altitude in m
   * @param  atmospheric   Atmospheric pressure in hPa
   * @return The approximate pressure in hPa
   */
  double BMP280_Sensor::seaLevelForAltitude(double altitude, double atmospheric) {
	return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
  }

  /*!
	@brief  Take a new measurement (only possible in forced mode)
	@return true if successful, otherwise false
 */
bool BMP280_Sensor::takeForcedMeasurement() {
	// If we are in forced mode, the BMP sensor goes back to sleep after each
	// measurement and we need to set it to forced mode once at this point, so
	// it will take the next measurement and then return to sleep again.
	// In normal mode simply does new measurements periodically.
	if (_powerMode == PowerMode_e::Forced) {
		setPowerMode(PowerMode_e::Forced);
		while (readRegister(Registers_e::Status) & 0x08){ 
			delay(1);
		}
		return true;
	}
	return false;
}


/*!
	@brief Check Connection Function
		Check if device is on the bus asks for one byte
	@return int16_t if less than 1 = error 
*/
int16_t BMP280_Sensor::CheckConnectionI2C(void)
{
	uint8_t rxData = 0;
	int16_t returnValue = 0;
	// Request 1 byte from the device
	returnValue = Wire.requestFrom((int)_address, 1);
	if (returnValue >= 1)
	{
		rxData = Wire.read(); // read the byte
	}

#if BMP280_DEBUG
	Serial.println("BMP280::CheckConnection. Info");
	Serial.print("I2C Return value = ");
	Serial.print(returnValue);
	Serial.print(" , RxData = ");
	Serial.println(rxData);
	if (returnValue >= 1)
		Serial.println("Connected.");
	else
		Serial.println("Not Connected.");
#endif

	return returnValue;
}

// ---------------- End of file ----------------
