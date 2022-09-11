/*
 * tmp102.h
 * MIT License
 *
 * Copyright (c) 2022 Vandevelde Niel
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions: 
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "tmp102.h"

/**
  * @brief  Check if the internal register pointer needs to be changed
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @param 	reg TMP102REG enum to new internal register pointer
  * @retval HAL status
  */
static HAL_StatusTypeDef setRegisterPointer(tmp102_device* dev, TMP102REG reg)
{
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t register_pointer;
	//Check if current pointer is to the same register
	if(dev->register_pointer != reg)
	{
		register_pointer = reg;
		dev->state = TMP102_WRITE;
		ret = HAL_I2C_Master_Transmit(dev->com.handle, dev->com.address, &register_pointer, 1, HAL_MAX_DELAY);
		dev->state = TMP102_READY;
		if(ret != HAL_OK)
		{
		  // HAL unable to send I2C data
		  return ret;
		}
		// Register pointer update
		dev->register_pointer = reg;
	}
	return ret;
}

/**
  * @brief  Check the state of the communication and return correct type
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @retval HAL status
  */
static HAL_StatusTypeDef getHalStatus(tmp102_device* dev)
{
	if(dev->state == TMP102_READY)
	{
		return HAL_OK;
	}
	else if (dev->state == TMP102_ERROR || dev->state == TMP102_UNINITIALIZED)
	{
		return HAL_ERROR;
	}
	else
	{
		return HAL_BUSY;
	}
}

/**
  * @brief  Read register value from TMP102
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @param 	buffer Pointer to a uint8_t array to store receive values
  * @retval HAL status
  */
static HAL_StatusTypeDef receiveRegisterValue(tmp102_device* dev, uint8_t* buffer)
{
	HAL_StatusTypeDef ret = HAL_OK;
	dev->state = TMP102_READ;
	ret = HAL_I2C_Master_Receive(dev->com.handle, dev->com.address, buffer, 2, HAL_MAX_DELAY);
	dev->state = TMP102_READY;
	if(ret != HAL_OK)
	{
	  // HAL unable to receive I2C data
	  return ret;
	}
	return ret;
}

/**
  * @brief  Set the values of the configuration register struct based on the received value
  * @param  response Pointer to a struct_configuration_register structure that will contain the configuration information.
  * @param 	buffer Pointer to a uint8_t array with read configuration register information
  */
static void setRcvConfigurationReg(struct_configuration_register* response, uint8_t* buffer)
{
	response->extended_mode = (buffer[1] & 1<<4) ? 1 : 0;
	response->altert_bit = (buffer[1] & 1<<5) ? 1 : 0;
	response->conversion_rate = buffer[1] >> 6;

	response->shutdown_mode = (buffer[0] & 1<<0) ? 1 : 0;
	response->thermostat_mode = (buffer[0] & 1<<1) ? 1 : 0;
	response->polarity = (buffer[0] & 1<<2) ? 1 : 0;
	response->fault_queue = (buffer[0] & 3 << 3) >>3;
	response->converter_resolution = (buffer[1] & 3 << 5) >>5;
	response->one_shot = (buffer[0] & 1<<7) ? 1 : 0;
}

/**
  * @brief  Set the buffer based on the configuration register struct
  * @param  status Pointer to a struct_configuration_register structure that will contain the configuration information.
  * @param 	buffer Pointer to a uint8_t array with write configuration register information
  */
static void setConfigurationRegBuffer(struct_configuration_register* status, uint8_t* buffer)
{
		buffer[2] |= (status->extended_mode&0x01) << 4;
		buffer[2] |= (status->altert_bit&0x01) << 5;
		buffer[2] |= (status->conversion_rate&0x03) << 6;

		buffer[1] |= (status->shutdown_mode&0x01) << 0;
		buffer[1] |= (status->thermostat_mode&0x01) << 1;
		buffer[1] |= (status->polarity&0x01) << 2;
		buffer[1] |= (status->fault_queue&0x03) << 3;
		buffer[1] |= (status->converter_resolution&0x03) << 5;
		buffer[1] |= (status->one_shot&0x01) << 7;
}

/**
  * @brief  Write to a temperature related register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @param 	reg Enum of type TMP102REG, register to write to
  * @param 	value Int with the register value
  * @retval HAL status
  */
static HAL_StatusTypeDef writeRegister(tmp102_device* dev, TMP102REG reg, int16_t value)
{
	HAL_StatusTypeDef ret = HAL_OK;
	if(dev->registers.config_reg.extended_mode == 0x01)
		value = value << 3;

	else
		value = value << 4;

	uint8_t buffer[3] = {reg, value >> 8, value};
	dev->state = TMP102_WRITE;
	ret = HAL_I2C_Master_Transmit(dev->com.handle, dev->com.address, buffer, 3, HAL_MAX_DELAY);
	dev->state = TMP102_READY;
	if(ret != HAL_OK)
	{
	  return ret;
	}
	dev->register_pointer = reg;
	return ret;
}

/**
  * @brief  Convert temperature to a register value
  * @param  temp Float with temperature to convert
  * @retval converted temperature value
  */
int16_t converterTempToRegisterValue(float temp)
{
	return temp/0.0625;
}

/**
  * @brief  Read the configuration register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @retval HAL status
  */
static HAL_StatusTypeDef readConfigurationRegister(tmp102_device* dev)
{
	uint8_t reg = TMP102_CONFIGURATION_REG;
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t buffer[2];
	ret = setRegisterPointer(dev, reg);
	if(ret != HAL_OK)
	{
	  return ret;
	}
	ret = receiveRegisterValue(dev, buffer);
	if(ret != HAL_OK)
	{
	  return ret;
	}
	setRcvConfigurationReg(&dev->registers.config_reg, buffer);
	return ret;
}

/**
  * @brief  Read high threshold register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @retval HAL status
  */
static HAL_StatusTypeDef readHighThresholdRegister(tmp102_device* dev)
{
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t buffer[2];
	uint8_t reg = TMP102_TEMP_HIGH_REG;
	ret = setRegisterPointer(dev, reg);
	if(ret != HAL_OK)
	{
	  return ret;
	}

	ret = receiveRegisterValue(dev, buffer);
	if(ret != HAL_OK)
	{
	  return ret;
	}

	if(dev->registers.config_reg.extended_mode == 0x01)	// 13 bit mode
	{
		dev->registers.t_high_threshold = ((int16_t)buffer[0] << 5) | (buffer[1] >> 3);
		if ( dev->registers.t_high_threshold > 0xFFF )
			dev->registers.t_high_threshold |= 0xE000;
	}
	else // 12 bit mode
	{
		dev->registers.t_high_threshold = ((int16_t)buffer[0] << 4) | (buffer[1] >> 4);
		// Convert to 2's complement, since temperature can be negative
		if ( dev->registers.t_high_threshold > 0x7FF )
			dev->registers.t_high_threshold |= 0xF000;
	}
	return ret;
}

/**
  * @brief  Convert a register value to a real temperature in degree Celcius
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @param 	reg Enum of type TMP102REG, register to get temperature from.
  * @retval HAL status
  */
float tmp102GetTemperature(tmp102_device* dev, TMP102REG reg)
{
	if(getHalStatus(dev)!=HAL_OK)
		return -1;
	float ret = 0;
	if(reg == TMP102_TEMP_HIGH_REG)
	  ret = dev->registers.t_high_threshold * 0.0625;
	else if (reg == TMP102_TEMP_LOW_REG)
	  ret = dev->registers.t_low_threshold * 0.0625;
	else if (reg ==  TMP102_TEMPERATURE_REG)
	  ret = dev->registers.temp_reg * 0.0625;
	else
	  ret = -1;
	return ret;
}

/**
  * @brief  Read temperature register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @retval HAL status
  */
static HAL_StatusTypeDef readTemperatureRegister(tmp102_device* dev)
{
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t buffer[2];
	uint8_t reg = TMP102_TEMPERATURE_REG;
	ret = setRegisterPointer(dev, reg);
	if(ret != HAL_OK)
	{
	  return ret;
	}

	ret = receiveRegisterValue(dev, buffer);
	if(ret != HAL_OK)
	{
	  return ret;
	}

	if(dev->registers.config_reg.extended_mode == 0x01)	// 13 bit mode
	{
		dev->registers.temp_reg = ((int16_t)buffer[0] << 5) | (buffer[1] >> 3);
		if ( dev->registers.temp_reg > 0xFFF )
			dev->registers.temp_reg |= 0xE000;
	}
	else // 12 bit mode
	{
		dev->registers.temp_reg = ((int16_t)buffer[0] << 4) | (buffer[1] >> 4);
		// Convert to 2's complement, since temperature can be negative
		if ( dev->registers.temp_reg > 0x7FF )
			dev->registers.temp_reg |= 0xF000;
	}
	return ret;
}

/**
  * @brief  Read low threshold register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @retval HAL status
  */
static HAL_StatusTypeDef readLowThresholdRegister(tmp102_device* dev)
{
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t buffer[2];
	uint8_t reg = TMP102_TEMP_LOW_REG;
	ret = setRegisterPointer(dev, reg);
	if(ret != HAL_OK)
	{
	  return ret;
	}

	ret = receiveRegisterValue(dev, buffer);
	if(ret != HAL_OK)
	{
	  return ret;
	}

	if(dev->registers.config_reg.extended_mode == 0x01)	// 13 bit mode
	{
		dev->registers.t_low_threshold = ((int16_t)buffer[0] << 5) | (buffer[1] >> 3);
		if ( dev->registers.t_low_threshold > 0xFFF )
			dev->registers.t_low_threshold |= 0xE000;
	}
	else // 12 bit mode
	{
		dev->registers.t_low_threshold = ((int16_t)buffer[0] << 4) | (buffer[1] >> 4);
		// Convert to 2's complement, since temperature can be negative
		if ( dev->registers.t_low_threshold > 0x7FF )
			dev->registers.t_low_threshold |= 0xF000;
	}
	return ret;
}

/**
  * @brief  Update the cached temperature register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @retval HAL status
  */
HAL_StatusTypeDef tmp102UpdateCachedTemperatureRegister(tmp102_device* dev)
{
	HAL_StatusTypeDef ret = getHalStatus(dev);
	if(ret != HAL_OK)
	{
		return ret;
	}

	ret = readTemperatureRegister(dev);
	if(ret != HAL_OK)
	{
		dev->state = TMP102_ERROR;
		return ret;
	}
	return ret;
}

/**
  * @brief  Update the cached configuration register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @retval HAL status
  */
HAL_StatusTypeDef tmp102UpdateCachedConfigurationRegister(tmp102_device* dev)
{
	HAL_StatusTypeDef ret = getHalStatus(dev);
	if(ret != HAL_OK)
	{
		return ret;
	}

	ret = readConfigurationRegister(dev);
	if(ret != HAL_OK)
	{
		dev->state = TMP102_ERROR;
		return ret;
	}
	return ret ;
}

/**
  * @brief  Update cached High Threshold Temperature Register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @retval HAL status
  */
HAL_StatusTypeDef tmp102UpdateCachedHighThresholdRegister(tmp102_device* dev)
{
	HAL_StatusTypeDef ret = getHalStatus(dev);
	if(ret != HAL_OK)
	{
		return ret;
	}

	ret = readHighThresholdRegister(dev);
	if(ret != HAL_OK)
	{
		dev->state = TMP102_ERROR;
		return ret;
	}
	return ret;
}

/**
  * @brief  Update cached Low Threshold Temperature register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @retval HAL status
  */
HAL_StatusTypeDef tmp102UpdateCachedLowThresholdRegister(tmp102_device* dev)
{
	HAL_StatusTypeDef ret = getHalStatus(dev);
	if(ret != HAL_OK)
	{
		return ret;
	}

	ret = readLowThresholdRegister(dev);
	if(ret != HAL_OK)
	{
		dev->state = TMP102_ERROR;
		return ret;
	}
	return ret;
}

/**
  * @brief  Update all cached registers
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @retval HAL status
  */
HAL_StatusTypeDef tmp102UpdateAllCachedRegisters(tmp102_device* dev)
{
	HAL_StatusTypeDef ret = getHalStatus(dev);
	if(ret != HAL_OK)
	{
		return ret;
	}
	ret = readConfigurationRegister(dev);
	if(ret != HAL_OK)
	{
		dev->state = TMP102_ERROR;
		return ret;
	}

	ret = readLowThresholdRegister(dev);
	if(ret != HAL_OK)
	{
		dev->state = TMP102_ERROR;
		return ret;
	}

	ret = readHighThresholdRegister(dev);
	if(ret != HAL_OK)
	{
		dev->state = TMP102_ERROR;
		return ret;
	}

	ret = readTemperatureRegister(dev);
	if(ret != HAL_OK)
	{
		dev->state = TMP102_ERROR;
		return ret;
	}


	return ret;
}

/**
  * @brief  Update the device I2C bus information
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @param 	bus Pointer to a struct_i2c_bus with the I2C information
  * @retval HAL status
  */
HAL_StatusTypeDef tmp102UpdateI2cBus(tmp102_device* dev, struct_i2c_bus* bus)
{
	dev->com.handle = bus->handle;
	TMP102ADDR address_range_base = TMP102_ADD0_GROUND;
	for(uint8_t i = 0; i < TMP102_NUMBER_OF_I2C_ADDRESSES; i++)
	{
		if(bus->address == address_range_base+i)
		{
			dev->com.address = bus->address<<1;
			return HAL_OK;
		}
	}
	dev->com.address = TMP102_ADD0_GROUND;
	return HAL_ERROR;
}

/**
  * @brief  Write value to configuration register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @param 	status Pointer to a struct_configuration register with updated value for configuration register
  * @retval HAL status
  */
HAL_StatusTypeDef tmp102WriteConfigurationRegister(tmp102_device* dev, struct_configuration_register* status)
{
	HAL_StatusTypeDef ret = getHalStatus(dev);
	if(ret != HAL_OK)
	{
		return ret;
	}
	uint8_t buffer[3] = {TMP102_CONFIGURATION_REG, 0x00, 0x00};
	setConfigurationRegBuffer(status, buffer);
	ret = HAL_I2C_Master_Transmit(dev->com.handle, dev->com.address, buffer, 3, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
	  return ret;
	}
	dev->register_pointer = TMP102_CONFIGURATION_REG;
	return ret;
}

/**
  * @brief  Check passed parameters for the temperature
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @param 	temp Float with passed temperature
  * @retval HAL status
  */
static HAL_StatusTypeDef checkParamtersThreshold(tmp102_device* dev, float temp)
{
	if(dev->registers.config_reg.extended_mode == 0x01)
	{
		if(temp > -256.0 && temp < 256.0)
		{
			return HAL_OK;
		}
		else
			return HAL_ERROR;
	}
	else
	{
		if(temp > -128.0 && temp < 128.0)
		{
			return HAL_OK;
		}
		else
			return HAL_ERROR;
	}
}

/**
  * @brief  Write High Threshold Temperature register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @param 	temp Float with new temperature value
  * @retval HAL status
  */
HAL_StatusTypeDef tmp102WriteHighThresholdRegister(tmp102_device* dev, float temp)
{
	if(checkParamtersThreshold(dev,temp) != HAL_OK)
	{
		return HAL_ERROR;
	}
	HAL_StatusTypeDef ret;
	int16_t reg_value = converterTempToRegisterValue(temp);
	ret = writeRegister(dev, TMP102_TEMP_HIGH_REG, reg_value);
	if(ret != HAL_OK)
	{
		return ret;
	}
	dev->registers.t_high_threshold = reg_value;
	return ret;
}

/**
  * @brief  Write Low Threshold Temperature register
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @param 	temp Float with new temperature value
  * @retval HAL status
  */
HAL_StatusTypeDef tmp102WriteLowThresholdRegister(tmp102_device* dev, float temp)
{
	if(checkParamtersThreshold(dev,temp) != HAL_OK)
	{
		return HAL_ERROR;
	}
	HAL_StatusTypeDef ret;
	int16_t reg_value = converterTempToRegisterValue(temp);
	ret = writeRegister(dev, TMP102_TEMP_LOW_REG, reg_value);
	if(ret != HAL_OK)
	{
		return ret;
	}
	dev->registers.t_low_threshold = reg_value;
	return ret;
}


/**
  * @brief  Initialize the tmp102 device
  * @param  dev Pointer to a tmp102_device structure that contains the configuration information for the specified TMP102.
  * @param 	hi2c I2C_HandleTypeDef
  * @param 	i2c_addr Enum of type TMP102ADDR with used I2C address based on the ADD0 pin
  * @retval HAL status
  */
HAL_StatusTypeDef tmp102Initialize(tmp102_device* dev, I2C_HandleTypeDef *hi2c, TMP102ADDR i2c_addr)
{
	HAL_StatusTypeDef ret;
	dev->state = TMP102_UNINITIALIZED;
	struct_i2c_bus bus;
	bus.handle = hi2c;
	bus.address = i2c_addr;

	ret = tmp102UpdateI2cBus(dev, &bus);
	if(ret != HAL_OK)
	{
		dev->state = TMP102_ERROR;
		return ret;
	}
	dev->state = TMP102_READY;
	ret = tmp102UpdateAllCachedRegisters(dev);
	if(ret != HAL_OK)
	{
		dev->state = TMP102_ERROR;
		return ret;
	}
	return ret;
}
