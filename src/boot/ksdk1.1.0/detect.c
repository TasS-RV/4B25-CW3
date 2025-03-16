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
#include "detect.h"
#include "devMMA8451Q.h"




int32_t get_sqrt(uint32_t magntiude){
    if (magntiude == 0) return 0;  // Avoid division by zero

    uint32_t x =  magntiude / 2;  // Initial guess - larger values will want a higher iterations and greater initial division
    for (int i = 0; i < 6; i++) {  // Higher iterations will give higher accuracy
        x = (x + magntiude / x)/2;
    }
    return x;
}

int32_t convertAcceleration(int16_t number){ // Convert the acceleration from multiples of (1/1024)g to mms^-2. 
    // Acceleration is given in multiples of (1/1024)g with the chosen +/- 8g range and 14-bit resolution of the MMA8451Q readings.
    // Hence, multiply by 9810 and then divide by 1024 to convert to mms^-2. Therefore, there is an implicit scaling factor of 1,000.
    int32_t result = ((int32_t)(number) * 9810) / 1024;
    // warpPrint("convertAcceleration(): %d * (9810/1024) = %d.\n", number, result);
    return result;
  }
  



void update_buffers(uint32_t acc_mag, uint16_t time_diff){
    accel_magnitude_buffer[buffer_index] = acc_mag;
    time_steps_buffer[buffer_index] = 10;    
    // Update buffer index (circular - reset using modulo) - print debug to check if being reset or updated
    warpPrint("Buffer_index: %d \n", buffer_index);
    buffer_index = (buffer_index + 1) % BUFF_SIZE;
}


uint32_t byte_to_state_conversion(){
    uint16_t x_LSB, y_LSB, z_LSB; // Least significant byte of each acceleration measurement.
    uint16_t x_MSB, y_MSB, z_MSB; // Most significant byte of each acceleration measurement.
    int32_t XAcceleration, YAcceleration, ZAcceleration; // Actual acceleration values for checking their accuracy.
    WarpStatus i2cReadStatus;

    timeBefore = OSA_TimeGetMsec(); // Start timing before polling registers and calculating numerical root

    i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6 /* numberOfBytes */); // Read 6 bytes consecutively to get 14-bit acceleration measurements from all three axes.
    
    if (i2cReadStatus != kWarpStatusOK){
        warpPrint("\nFailed to read acceleration measurements.\n");
        return; // Return - exit function if the sensor cannot be read from.
    }

    x_MSB = deviceMMA8451QState.i2cBuffer[0];
    x_LSB = deviceMMA8451QState.i2cBuffer[1];
    XCombined = ((x_MSB & 0xFF) << 6) | (x_LSB >> 2);
    XCombined = (XCombined ^ (1 << 13)) - (1 << 13);
    //warpPrint("x_MSB: %d, x_MSB: %d, XCombined - Decimal: %d, Hexadecimal: %x.\n", x_MSB, x_LSB, XCombined, XCombined);
    XAcceleration = convertAcceleration(XCombined);
    warpPrint("XAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n", XAcceleration, XAcceleration);

    // XVariance *= (totalSamples - 1); // Undo the division by n when the variance was calculated last.
    // XVariance += (XAcceleration*XAcceleration);
    // XVariance /= totalSamples; // Implement the new division by n for this sample.
    // warpPrint("XVariance = %d.\n", XVariance);
        
    y_MSB = deviceMMA8451QState.i2cBuffer[2];
    y_LSB = deviceMMA8451QState.i2cBuffer[3];
    YCombined = ((y_MSB & 0xFF) << 6) | (y_LSB >> 2);
    YCombined = (YCombined ^ (1 << 13)) - (1 << 13);
    //warpPrint("y_MSB: %d, y_MSB: %d, YCombined - Decimal: %d, Hexadecimal: %x.\n", y_MSB, y_LSB, YCombined, YCombined);
    YAcceleration = convertAcceleration(YCombined);
    warpPrint("YAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n", YAcceleration, YAcceleration);

    // YVariance *= (totalSamples - 1); // Undo the division by n when the variance was calculated last.
    // YVariance += (YAcceleration*YAcceleration);
    // YVariance /= totalSamples; // Implement the new division by n for this sample.
    // warpPrint("YVariance = %d.\n", YVariance);
        
    z_MSB = deviceMMA8451QState.i2cBuffer[4];
    z_LSB = deviceMMA8451QState.i2cBuffer[5];
    ZCombined = ((z_MSB & 0xFF) << 6) | (z_LSB >> 2);
    ZCombined = (ZCombined ^ (1 << 13)) - (1 << 13);
    //warpPrint("z_MSB: %d, z_MSB: %d, ZCombined - Decimal: %d, Hexadecimal: %x.\n", z_MSB, z_LSB, ZCombined, ZCombined);
    ZAcceleration = convertAcceleration(ZCombined);
    warpPrint("ZAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n\n", ZAcceleration, ZAcceleration);

    
    uint32_t acc_magntiude = get_sqrt((uint32_t)(ZAcceleration*ZAcceleration) + (uint32_t)(YAcceleration*YAcceleration) + (uint32_t)(XAcceleration*XAcceleration));
    // Testing with known values
    //uint32_t acc_magntiude = get_sqrt((uint32_t)2500);
    warpPrint("Magnitude of acceleration: %d \n", acc_magntiude);
    
    timeAft = OSA_TimeGetMsec();

    //update_buffers(acc_magntiude, (uint16_t)((timeBefore - timeAft) + 100));

    return acc_magntiude;
}


