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

#ifndef INC_TMP102_H_
#define INC_TMP102_H_
#ifdef __cplusplus
 extern "C" {
#endif

/* Driver includes */
#include "main.h"
/* User defines */
#define TMP102_NUMBER_OF_I2C_ADDRESSES 4	//Number of possible I2C address the device could have

/* Driver defines */

/* Driver variables */

/* Driver enum */

 /* Enum with the possible states of the communication */
typedef enum TMP102STATE
{
	TMP102_READY = 0x00,			// Device ready to send or receive
	TMP102_READ,					// Device is reading from the IC
	TMP102_WRITE,					// Device is writing to the IC
	TMP102_UNINITIALIZED,			// Device is uninitialized
	TMP102_ERROR,					// Error in the setup of the device
}TMP102STATE;

/* Enum with the possible slave addresses */
typedef enum TMP102ADDR
{
	TMP102_ADD0_GROUND = 0x48,		// Address if ADD0 is connected to GND
	TMP102_ADD0_VCC = 0x49,			// Address if ADD0 is connected to V+
	TMP102_ADD0_SDA = 0x4A,			// Address if ADD0 is connected to SDA
	TMP102_ADD0_SCL = 0x4B			// Address if ADD0 is connected to SCL
}TMP102ADDR;

/* Enum with the possible registers */
typedef enum TMP102REG
{
	TMP102_TEMPERATURE_REG=0x00,	// Temperature Register (Read Only)
	TMP102_CONFIGURATION_REG=0x01,	// Configuration Register (Read/Write)
	TMP102_TEMP_LOW_REG=0x02,		// TLOW Register (Read/Write)
	TMP102_TEMP_HIGH_REG=0x03,		// THIGH Register (Read/Write)
}TMP102REG;

/* Driver structs */

/* Struct with the content of the configuration register */
typedef struct struct_configuration_register
{
	uint8_t extended_mode;			// Extended mode (EM)
	uint8_t altert_bit;				// Alert (AL Bit
	uint8_t conversion_rate;		// Conversion rate
	uint8_t shutdown_mode;			// Shutdown Mode (SD)
	uint8_t thermostat_mode;		// Thermostat Mode (TM)
	uint8_t polarity;				// Polarity (POL)
	uint8_t fault_queue;			// Fault Queue (F1/F0)
	uint8_t converter_resolution;	// Converter Resolution (R1/R2)
	uint8_t one_shot;				// One-shot/Conversion Ready (OS)
}struct_configuration_register;

/* Struct to store cached values from the registers */
typedef struct struct_registers
{
	struct_configuration_register config_reg;	// Configuration Register
	int16_t t_high_threshold;					// THIGH Register
	int16_t t_low_threshold;					// TLOW Register
	int16_t temp_reg;							// Temperature Register
}struct_registers;

/* Struct to store I2C related information */
typedef struct struct_i2c_bus
{
	I2C_HandleTypeDef* handle;			// STM32 I2C handle
	uint8_t address;					// I2C address
}struct_i2c_bus;

/* Struct to store TMP102 related information */
typedef struct tmp102_device
{
	TMP102STATE state;					// State of the communication
	struct_registers registers;			// Cached register values
	struct_i2c_bus com;					// I2C communication values
	TMP102REG register_pointer;			// Current register pointer of the TMP102

}tmp102_device;


/* Driver functions */

HAL_StatusTypeDef tmp102Initialize(tmp102_device* dev, I2C_HandleTypeDef *hi2c, TMP102ADDR i2c_addr); 		// Initialization of the device

HAL_StatusTypeDef tmp102WriteConfigurationRegister(tmp102_device* dev, struct_configuration_register* status);	// Write to the configuration register
HAL_StatusTypeDef tmp102WriteHighThresholdRegister(tmp102_device* dev, float temp);								// Write to THIGH Register, values will be rounded if they are not a multiple of 0.0625
HAL_StatusTypeDef tmp102WriteLowThresholdRegister(tmp102_device* dev, float temp);								// Write to TLOW Register, values will be rounded if they are not a multiple of 0.0625

HAL_StatusTypeDef tmp102UpdateAllCachedRegisters(tmp102_device* dev);				// Update all cached values / read all registers content
HAL_StatusTypeDef tmp102UpdateCachedTemperatureRegister(tmp102_device* dev);		// Update temperature cached value / read temperature register content
HAL_StatusTypeDef tmp102UpdateCachedConfigurationRegister(tmp102_device* dev);		// Update configuration cached value / read configuration register content
HAL_StatusTypeDef tmp102UpdateCachedHighThresholdRegister(tmp102_device* dev);		// Update THIGH cached value / read THIGH register content
HAL_StatusTypeDef tmp102UpdateCachedLowThresholdRegister(tmp102_device* dev);		// Update TLOW cached value / read TLOW register content

float tmp102GetTemperature(tmp102_device* dev, TMP102REG reg);	// Get temperature value (°C) based on passed register

HAL_StatusTypeDef tmp102UpdateI2cBus(tmp102_device* dev, struct_i2c_bus* bus);	// Update the I2C information


#ifdef __cplusplus
}
#endif

#endif /* INC_TMP102_H_ */
