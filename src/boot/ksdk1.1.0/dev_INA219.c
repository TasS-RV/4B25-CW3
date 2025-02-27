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



void
init_INA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	device_INA219State.i2cAddress					= i2cAddress;
	device_INA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegister_INA219(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
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
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		1,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



// Passing in 16 bit values, as compared to the previously function from the MMA8451Q, these are 16 bit fields.
WarpStatus configureSensorINA219(uint16_t configPayload, uint16_t calibrationPayload)  
{
	WarpStatus i2cWriteStatus1, i2cWriteStatus2;

	// Set operating voltage for INA219
	warpScaleSupplyVoltage(device_INA219State.operatingVoltageMillivolts);

	// Write to Configuration Register (0x00)
	i2cWriteStatus1 = writeSensorRegisterINA219(
		kWarpSensorConfigurationRegister_INA219_SETUP, // 0x00
		configPayload // Payload: Operating mode, gain, and resolution
	);

	// Write to Calibration Register (0x05)
	i2cWriteStatus2 = writeSensorRegisterINA219(
		kWarpSensorConfigurationRegister_INA219_CTRL, // 0x05
		calibrationPayload // Payload: Calibration value based on shunt resistor
	);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}




WarpStatus
readSensorRegister_INA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
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
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)device_INA219State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}





void printSensorData_INA219(bool hexModeFlag)
{
	uint16_t readSensorRegisterValue;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(device_INA219State.operatingVoltageMillivolts);

	// Read Shunt Voltage (Register 0x01)
	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_SHUNT_VOLTAGE, &readSensorRegisterValue);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%04x,", readSensorRegisterValue);
		}
		else
		{
			warpPrint(" %d,", (int16_t)readSensorRegisterValue);
		}
	}

	// Read Bus Voltage (Register 0x02)
	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_BUS_VOLTAGE, &readSensorRegisterValue);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		// Bus voltage register has the last 3 bits reserved, so we shift right by 3
		readSensorRegisterValue >>= 3;

		if (hexModeFlag)
		{
			warpPrint(" 0x%04x,", readSensorRegisterValue);
		}
		else
		{
			warpPrint(" %d mV,", readSensorRegisterValue * 4); // LSB = 4mV
		}
	}

	// Read Current (Register 0x04)
	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_CURRENT, &readSensorRegisterValue);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%04x,", readSensorRegisterValue);
		}
		else
		{
			warpPrint(" %d mA,", (int16_t)readSensorRegisterValue);
		}
	}

	// Read Power (Register 0x03)
	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_POWER, &readSensorRegisterValue);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%04x", readSensorRegisterValue);
		}
		else
		{
			warpPrint(" %d mW", readSensorRegisterValue);
		}
	}
}




/*
The above is necessary to print and display the data in the first place - the following function stores the INA219 data in a buffer.
*/
uint8_t appendSensorData_INA219(uint8_t* buf)
{
	uint8_t index = 0;
	uint16_t readSensorRegisterValue;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(device_INA219State.operatingVoltageMillivolts);

	// Read and store Shunt Voltage (0x01)
	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_SHUNT_VOLTAGE, &readSensorRegisterValue);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index++] = 0;
		buf[index++] = 0;
	}
	else
	{
		buf[index++] = (uint8_t)(readSensorRegisterValue >> 8);
		buf[index++] = (uint8_t)(readSensorRegisterValue);
	}

	// Read and store Bus Voltage (0x02)
	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_BUS_VOLTAGE, &readSensorRegisterValue);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index++] = 0;
		buf[index++] = 0;
	}
	else
	{
		readSensorRegisterValue >>= 3; // Remove last 3 bits

		buf[index++] = (uint8_t)(readSensorRegisterValue >> 8);
		buf[index++] = (uint8_t)(readSensorRegisterValue);
	}

	// Read and store Current (0x04)
	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_CURRENT, &readSensorRegisterValue);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index++] = 0;
		buf[index++] = 0;
	}
	else
	{
		buf[index++] = (uint8_t)(readSensorRegisterValue >> 8);
		buf[index++] = (uint8_t)(readSensorRegisterValue);
	}

	// Read and store Power (0x03)
	i2cReadStatus = readSensorRegister_INA219(kWarpSensorOutputRegisterINA219_POWER, &readSensorRegisterValue);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index++] = 0;
		buf[index++] = 0;
	}
	else
	{
		buf[index++] = (uint8_t)(readSensorRegisterValue >> 8);
		buf[index++] = (uint8_t)(readSensorRegisterValue);
	}

	return index;
}
