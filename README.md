# STM32 TMP102 driver

## Introduction
This is a driver for the TMP102 temperature sensor. Data can be read from the device with I2C. This C written driver should make the use of the device easy and straight forward. The alert pin control is not implemented in the driver and should be taken care of by the user. The driver currently uses the STM32 HAL drivers for the I2C communication.

## How it works
One structure is created that will store all information of the TMP102. This structure will store the read register values, the status of the communication, the current register pointer of the external device and I2C communication handle and address.
``` 
typedef struct tmp102_device
{
	TMP102STATE state;					// State of the communication
	struct_registers registers;			// Cached register values
	struct_i2c_bus com;					// I2C communication values
	TMP102REG register_pointer;			// Current register pointer of the TMP102

}tmp102_device;
``` 
The *registers* struct will store the read values from the TMP102 (cached). The configuration register is also a struct to skip the need of doing bit shifts after every read operation.

``` 

typedef struct struct_registers
{
	struct_configuration_register config_reg;	// Configuration Register
	int16_t t_high_threshold;					// THIGH Register
	int16_t t_low_threshold;					// TLOW Register
	int16_t temp_reg;							// Temperature Register
}struct_registers;
```
Every mode/function of each bit will be set by the read value. Some of these are 1 bit, other are 2 bit. The find more information on these bits, check the datasheet.
```
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
```

The communication state (*TMP102STATE state*) will update if the I2C is sending or receiving. It will be used internally to check that the device is not reading and writing at the same time. 

The register pointer (*TMP102REG register_pointer*) will prevent sending the same register twice to the device of the external register pointer, the one on the TMP102, is already pointing to the correct register. 
## How to use

### 1. Include
Add the files to your project. In the main file add the following line:
```
#include "tmp102.h"
```
### 2. Device creation
Create a structure of the type *tmp102_device*. This structure will hold information about the registers, register pointer, communication status and i2c information.
```
tmp102_device dev;
``` 

### 3. Device initilialization
The following function will initialize the previous created struct by passing it as the first argument. The second argument should be the I2C handle. The last argument is an enum value of the type *TMP102ADDR*. The value for this argument is based on the connection of the TMP102 pin. The address will automatically be shifted to the correct value. 

First this will update the struct_i2c_bus struct with the correct values. After that it will read all register values and store them in the struct_registers struct. The *TMP102STATE* enum and *TMP102REG* enum will be updated based on the output of these actions.
```
HAL_StatusTypeDef tmp102Initialize(tmp102_device* dev, I2C_HandleTypeDef *hi2c, TMP102ADDR i2c_addr);
``` 
### 4. Device ready
If this setup has worked correctly and didn't return an error code the device is ready to be used. The following functions can be used to modifiy the registers of the TMP102 or the update the cached register values. 
``` 
HAL_StatusTypeDef tmp102WriteConfigurationRegister(tmp102_device* dev, struct_configuration_register* status);	// Write to the configuration register
HAL_StatusTypeDef tmp102WriteHighThresholdRegister(tmp102_device* dev, float temp);								// Write to THIGH Register, values will be rounded if they are not a multiple of 0.0625
HAL_StatusTypeDef tmp102WriteLowThresholdRegister(tmp102_device* dev, float temp);								// Write to TLOW Register, values will be rounded if they are not a multiple of 0.0625

HAL_StatusTypeDef tmp102UpdateAllCachedRegisters(tmp102_device* dev);				// Update all cached values / read all registers content
HAL_StatusTypeDef tmp102UpdateCachedTemperatureRegister(tmp102_device* dev);		// Update temperature cached value / read temperature register content
HAL_StatusTypeDef tmp102UpdateCachedConfigurationRegister(tmp102_device* dev);		// Update configuration cached value / read configuration register content
HAL_StatusTypeDef tmp102UpdateCachedHighThresholdRegister(tmp102_device* dev);		// Update THIGH cached value / read THIGH register content
HAL_StatusTypeDef tmp102UpdateCachedLowThresholdRegister(tmp102_device* dev);		// Update TLOW cached value / read TLOW register content
``` 

### 5. Get temperature
The created *tmp102_device* stores all information of the registers of the TMP102. To get the temperature value (type : float) of a certain register, the following command can be used:
``` 
float temperature = tmp102GetTemperature(&dev, TMP102_TEMPERATURE_REG);	// Get temperature value (°C) based on passed register
``` 
Other values that can be passed are TMP102_TEMP_HIGH_REG and TMP102_LOW_HIGH_REG



