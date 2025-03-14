#include <stdlib.h>
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	device_INA219State; //Renamed and modified this variable - baud rates, delay fro reading and writing bits on I2C bus will stay the same (not touching lines of code below).
extern volatile uint32_t			gWarpI2cBaudRateKbps; 
extern volatile uint32_t			gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t			gWarpSupplySettlingDelayMilliseconds;

// Required for parsing the 2 byte values from each of the registers based on conversions from the datasheet
#define SHUNT_LSB_micro 10  // 10 µV per LSB
#define BUS_LSB_milli 4     // 4 mV per LSB
uint32_t current_LSB_micro = 10;   // 60 mA - SSD1331 datasheet based.  /2^15 --> 1.83 microAmps/ bit 

// 5.117 V / 0.1 = 51.17 A/ 2^15 bits = 1.56e-3 A/bit --> Scale by 1e6 for uA/ bit 
                                    // Note that the calibraiton register might still need to be set manually - the 2011 datasheet suggests just picvk a value that will NOT result in overflows and scale the calibration accordingly.
void
init_INA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{	
	device_INA219State.i2cAddress					= i2cAddress;
	device_INA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}



WarpStatus
writeSensorRegister_INA219(uint8_t deviceRegister, uint16_t payload) // This has 2 bytes per payload - registers are still 2 nibble addresses 
{
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x00:  // Configuration Register
		case 0x01:  // Shunt Voltage Register (Read-Only)
		case 0x02:  // Bus Voltage Register (Read-Only)
		case 0x03:  // Power Register
		case 0x04:  // Current Register
		case 0x05:  // Calibration Register
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = device_INA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(device_INA219State.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = (payload >> 8) & 0xFF; // MSB - need to modify to accomodate 2 bytes
	payloadByte[1] = payload & 0xFF;        // LSB

	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		2,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



WarpStatus
readSensorRegister_INA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF}; // Command buff Hex vaalue is correct, identical to MMA8451Q
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00:  // Configuration Register
		case 0x01:  // Shunt Voltage Register (Read-Only)
		case 0x02:  // Bus Voltage Register (Read-Only)
		case 0x03:  // Power Register
		case 0x04:  // Current Register
		case 0x05:  // Calibration Register
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = device_INA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(device_INA219State.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance - does thhis need incrementing? Given it is another i2c peripheral*/,
		&slave,
		cmdBuf,
		2,  // 2 bytes being read - instead of 1, then concatenated for the MMA8251Q
		(uint8_t *)device_INA219State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



// Passing in 16 bit values, as compared to the previously function from the MMA8451Q, these are 16 bit fields.
WarpStatus configureSensor_INA219(uint16_t configPayload, uint16_t calibrationPayload)  
{
	WarpStatus i2cWriteStatus1, i2cWriteStatus2;

	// Set operating voltage for INA219
	warpScaleSupplyVoltage(device_INA219State.operatingVoltageMillivolts);

	// Write to Configuration Register (0x00)
	i2cWriteStatus1 = writeSensorRegister_INA219(
		kWarpSensorConfigurationRegister_INA219_SETUP, // 0x00
		configPayload // Payload: Operating mode, gain, and resolution
	);

	// Write to Calibration Register (0x05)
	i2cWriteStatus2 = writeSensorRegister_INA219(
		kWarpSensorConfigurationRegister_INA219_CALIB, // 0x05
		calibrationPayload // Payload: Calibration value based on shunt resistor
	);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}


// Function to compute the value based on the register being read
void parseINA219Register(uint8_t regAddress, uint16_t rawValue, int CALIB_VALUE) {
	warpPrint("\r\tRegister Address (Dec): %d \n-->", regAddress); 

	// Process based on the actual decimal values observed
	if ((int)regAddress == 0) {  // Configuration Register
		warpPrint("\r\tConfiguration Register: 0x%04x\n", rawValue);

	} else if ((int)regAddress == 1) {  // Shunt Voltage Register
		warpPrint("\r\tShunt Voltage: %d uV\n", (int)rawValue*SHUNT_LSB_micro);
		
		//int current_expected = (((int)rawValue * 1000 * CALIB_VALUE)/4096);   --> Do not need this for the moment, have commented out 
		//warpPrint("\r\tExpected current: %d mA\n", current_expected);

	} else if ((int)regAddress == 2) {  // Bus Voltage Register
		uint16_t busRaw = (rawValue >> 3) & 0x1FFF;  // Ignore lower 3 bits - right shift
		warpPrint("\r\tBus Voltage: %d mV\n",(int)busRaw * BUS_LSB_milli);

		// Equation 5 - Current_LSB * 20 * Power register contents gives power in W, as current_LSB = uA/bit gets rescaled
	} else if ((int)regAddress == 3) {  
		int power = ((int)rawValue) * 20 * current_LSB_micro;  
		warpPrint("\r\tPower: %.d uW\n", power);
	
	} else if ((int)regAddress == 4) {  // Current Register DEC value x Current_LSB gives value in uA/ bit. 
		warpPrint("\r\tCurrent scaled: %d uA\n", (int)rawValue * current_LSB_micro);

	} else if ((int)regAddress == 5){ // Calibration register - set at the start 
		warpPrint("\r\t Calibration Register raw value: %d \n", rawValue);
	}
	else {
		warpPrint("\r\tWARNING: Unknown Register %d\n", regAddress);
	}
}




// void printSensorData_INA219(bool hexModeFlag)
// {
// 	uint16_t readSensorRegisterValue;
// 	WarpStatus i2cReadStatus;

// 	warpScaleSupplyVoltage(device_INA219State.operatingVoltageMillivolts);

// 	// Read Shunt Voltage (Register 0x01)
// 	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_SHUNT_VOLTAGE, &readSensorRegisterValue);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		warpPrint(" ----,");
// 	}
// 	else
// 	{
// 		if (hexModeFlag)
// 		{
// 			warpPrint(" 0x%04x,", readSensorRegisterValue);
// 		}
// 		else
// 		{
// 			warpPrint(" %d,", (int16_t)readSensorRegisterValue);
// 		}
// 	}

// 	// Read Bus Voltage (Register 0x02)
// 	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_BUS_VOLTAGE, &readSensorRegisterValue);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		warpPrint(" ----,");
// 	}
// 	else
// 	{
// 		// Bus voltage register has the last 3 bits reserved, so we shift right by 3
// 		readSensorRegisterValue >>= 3;

// 		if (hexModeFlag)
// 		{
// 			warpPrint(" 0x%04x,", readSensorRegisterValue);
// 		}
// 		else
// 		{
// 			warpPrint(" %d mV,", readSensorRegisterValue * 4); // LSB = 4mV
// 		}
// 	}

// 	// Read Current (Register 0x04)
// 	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_CURRENT, &readSensorRegisterValue);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		warpPrint(" ----,");
// 	}
// 	else
// 	{
// 		if (hexModeFlag)
// 		{
// 			warpPrint(" 0x%04x,", readSensorRegisterValue);
// 		}
// 		else
// 		{
// 			warpPrint(" %d mA,", (int16_t)readSensorRegisterValue);
// 		}
// 	}

// 	// Read Power (Register 0x03)
// 	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_POWER, &readSensorRegisterValue);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		warpPrint(" ----");
// 	}
// 	else
// 	{
// 		if (hexModeFlag)
// 		{
// 			warpPrint(" 0x%04x", readSensorRegisterValue);
// 		}
// 		else
// 		{
// 			warpPrint(" %d mW", readSensorRegisterValue);
// 		}
// 	}
// }



// /*
// The above is necessary to print and display the data in the first place - the following function stores the INA219 data in a buffer.
// */
// uint8_t appendSensorData_INA219(uint16_t* buf)
// {
// 	uint8_t index = 0;
// 	uint16_t readSensorRegisterValue;
// 	WarpStatus i2cReadStatus;

// 	warpScaleSupplyVoltage(device_INA219State.operatingVoltageMillivolts);

// 	// Read and store Shunt Voltage (0x01)
// 	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_SHUNT_VOLTAGE, &readSensorRegisterValue);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		buf[index++] = 0;
// 		buf[index++] = 0;
// 	}
// 	else
// 	{
// 		buf[index++] = (uint8_t)(readSensorRegisterValue >> 8);
// 		buf[index++] = (uint8_t)(readSensorRegisterValue);
// 	}

// 	// Read and store Bus Voltage (0x02)
// 	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_BUS_VOLTAGE, &readSensorRegisterValue);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		buf[index++] = 0;
// 		buf[index++] = 0;
// 	}
// 	else
// 	{
// 		readSensorRegisterValue >>= 3; // Remove last 3 bits

// 		buf[index++] = (uint8_t)(readSensorRegisterValue >> 8);
// 		buf[index++] = (uint8_t)(readSensorRegisterValue);
// 	}

// 	// Read and store Current (0x04)
// 	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_CURRENT, &readSensorRegisterValue);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		buf[index++] = 0;
// 		buf[index++] = 0;
// 	}
// 	else
// 	{
// 		buf[index++] = (uint8_t)(readSensorRegisterValue >> 8);
// 		buf[index++] = (uint8_t)(readSensorRegisterValue);
// 	}

// 	// Read and store Power (0x03)
// 	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_POWER, &readSensorRegisterValue);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		buf[index++] = 0;
// 		buf[index++] = 0;
// 	}
// 	else
// 	{
// 		buf[index++] = (uint8_t)(readSensorRegisterValue >> 8);
// 		buf[index++] = (uint8_t)(readSensorRegisterValue);
// 	}

// 	return index;
// }
