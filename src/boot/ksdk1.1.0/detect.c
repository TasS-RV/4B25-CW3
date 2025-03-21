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


int64_t Acc_mag_Variance = 0; // Instead, storing instantaenous acceleration magntiude as a global variance
uint16_t timediff_poll[3] = {0};


// int64_t Prev_Covars_Y[NUM_FREQS] = {0};
// int64_t Covars_Y[NUM_FREQS] = {0};
// int64_t Prev_Y_Vars[NUM_FREQS] = {0};
// int64_t Y_Vars[NUM_FREQS] = {0};


// Precomputed coefficients (scaled by 1000 as we cannot handle floats) - currently 3-7Hz inclusive 
const int32_t coeffs[NUM_FREQS] = {
    1902,  // 2 * cos(2π * 2 / 40) * 1000
    1782,  // 2 * cos(2π * 3 / 40) * 1000
    1618,  // 2 * cos(2π * 4 / 40) * 1000
    1414,  // 2 * cos(2π * 5 / 40) * 1000
    1175,  // 2 * cos(2π * 6 / 40) * 1000
    907,   // 2 * cos(2π * 7 / 40) * 1000
    618,   // 2 * cos(2π * 8 / 40) * 1000
    313,   // 2 * cos(2π * 9 / 40) * 1000
    0,     // 2 * cos(2π * 10 / 40) * 1000
    -313,  // 2 * cos(2π * 11 / 40) * 1000
    -618   // 2 * cos(2π * 12 / 40) * 1000
    -907   // 2 * cos(2π * 13 / 40) * 1000
};


int32_t get_sqrt(uint32_t magntiude){
    if (magntiude == 0) return 0;  // Avoid division by zero

    uint32_t x =  magntiude / 2;  // Initial guess - larger values will want a higher iterations and greater initial division
    for (int i = 0; i < 12; i++) {  // Higher iterations will give higher accuracy
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
  

uint32_t compute_goertzel_power()
{   
    uint32_t power[NUM_FREQS] = {0}; //Power is also size of frequency bins, and forms a rolling buffer that is over-written

    int max_pwr_index = 0; // Index for tracking the  frequency bin with peak power amplitude
    uint32_t max_power = 0; // Need to locally store max power for max frequency bin finding function below - keeping bit resolution consistent with power array.

    for (int i = 0; i < NUM_FREQS; i++) {
        // Get precomputed coefficient (scaled ×1000)
        int32_t coeff = coeffs[i];

        // Correct power computation based on the last 2 Y_N and Y_N-1 values - supplemental input val;ue of X_N = 0 assumed.
        power[i] =  (y_values[i][1] * y_values[i][1]) 
                       + (y_values[i][0] * y_values[i][0])
                       - ((coeff * y_values[i][1] * y_values[i][0]) / 1000);

        // Track index of frequency bin whcih is the max power
        if (power[i] > max_power) {
            max_pwr_index = i;
            max_power = power[i];
        }

        if (MMA8451Q_RAW_DATA_COLLECT == 0){warpPrint("\nPower at %d Hz is: %u\n", target_freqs[i], power[i]);} //%u - modifier prints power in unsigned format - expected to be positive
    }
    if (MMA8451Q_RAW_DATA_COLLECT == 0){warpPrint("\n--> Next time window.\n");}
    
    // Call Baysean probability calculation function:
    calculate_baysean(max_pwr_index, power); // Pass in the power spectrm - we may decide to do on the fly Gaussian or other proability modelling and computations with the instantaenous  

    //return P_obs_normalised(target_freq, power);      <-- Use this to get nromalised power when implementing tinot final proability function      
    return power;
}


// This is wrong - I am using the power spectrumm, and not a probaiblity counts when running the test multiple times to get counts of frequencies
uint32_t P_obs_normalised(int target_freq, uint32_t spectrum[NUM_FREQS]){
    uint32_t full_power = 0;
    for (int i = 0; i < NUM_FREQS; i++) {
        full_power = full_power + spectrum[i]; 
    }
    return spectrum[target_freq-2]/full_power; //Frequencies range from 2-13Hz - indices range from 0-11 (12 total bins)
}


/*
Goertzel Update Function - pass in x_n = acc_mag in update_buffers() function in devMMA8451Q. Stores y_n-2 and y_n-2 to computer y_n 
Then performs bitshift to update y_n-2 with y_n-1 and y_n-1 gets replaced with y_n.
*/
void update_goertzel(uint32_t x_n) {
    // Iteraate over each frequecy to computer the Y_n for that frequency - maintaining fized asample rate
    for (int i = 0; i < NUM_FREQS; i++) {
        // Get precomputed coefficient (scaled ×1000)
        int32_t coeff = coeffs[i];

        // Compute y_N - 2*1000*cos() * Y_n-1/ 1000 scaled for integer math
        int32_t y_N = ((2*coeff * y_values[i][1]) / 1000) - y_values[i][0] + x_n;

        //warpPrint("\ny_N values: %d \n\n", y_N); // For print debugging - if we ever get zero powers
        // Shift values: Move y[N-1] → y[N-2], and store y_N in y[N-1]
        y_values[i][0] = y_values[i][1];  // y[N-2] = old y[N-1]
        y_values[i][1] = y_N;           // y[N-1] = new y[N]

          
        if (MMA8451Q_RAW_VarError_PROP){ // Flag for computing covariance - may require lower sampling frequency to observe, favours hgihest possible sampling rate for higher accuracy
            int64_t Var_Y_Nsub2 = Prev_Y_Vars[i];  // Var(y[N-2])
            int64_t Var_Y_Nsub1 = Y_Vars[i];       // Var(y[N-1])
            int64_t Cov_Y_Nsub2 = Prev_Covars_Y[i]; // Cov(y[N-2], y[N-3])
            
            // Compute new covariance: Cov(y[N-1], y[N-2]) = a * Var(y[N-2]) - Cov(y[N-2], y[N-3])
            int64_t Cov_Y_Nsub1 = (int64_t)coeff * Var_Y_Nsub2 / 1000 - Cov_Y_Nsub2;
            
            // Compute new variance using updated covariance
            int64_t Var_Y_N = Acc_mag_Variance 
                                    + (int64_t)coeff * (int64_t)coeff * Var_Y_Nsub1 / 1000000
                                    + Var_Y_Nsub2
                                    + 2 * (int64_t)coeff * Cov_Y_Nsub1 / 1000;
            
            // Store the new computed values - shift to new indices in registers
            Prev_Y_Vars[i] = Var_Y_Nsub1;   // Shift Var(y[N-1]) → Var(y[N-2])
            Y_Vars[i] = Var_Y_N;            // Store new Var(y[N])
            Prev_Covars_Y[i] = Cov_Y_Nsub1; // Shift Cov(y[N-1], y[N-2]) → Cov(y[N-2], y[N-3])
            Covars_Y[i] = Cov_Y_Nsub1;      // Store new Cov(y[N], y[N-1])

            warpPrint("\nY_N variance: %d for frequency: %d Hz.", Var_Y_N, i);}
      
    }
    return;
}



uint32_t byte_to_state_conversion(uint16_t sampling_time_delta){
    uint16_t x_LSB, y_LSB, z_LSB; // Least significant byte of each acceleration measurement.
    uint16_t x_MSB, y_MSB, z_MSB; // Most significant byte of each acceleration measurement.
    int32_t XAcceleration, YAcceleration, ZAcceleration; // Actual acceleration values for checking their accuracy.
    WarpStatus i2cReadStatus;

    //timeBefore = OSA_TimeGetMsec(); // Start timing before polling registers and calculating numerical root
    i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6 /* numberOfBytes */); // Read 6 bytes consecutively to get 14-bit acceleration measurements from all three axes.
    
    if (i2cReadStatus != kWarpStatusOK){
        warpPrint("\nFailed to read acceleration measurements.\n");
        return; // Return - exit function if the sensor cannot be read from.
    }

    timeBefore_poll = OSA_TimeGetMsec(); 

    x_MSB = deviceMMA8451QState.i2cBuffer[0];
    x_LSB = deviceMMA8451QState.i2cBuffer[1];
    XCombined = ((x_MSB & 0xFF) << 6) | (x_LSB >> 2);
    XCombined = (XCombined ^ (1 << 13)) - (1 << 13);
    //warpPrint("x_MSB: %d, x_MSB: %d, XCombined - Decimal: %d, Hexadecimal: %x.\n", x_MSB, x_LSB, XCombined, XCombined);
    XAcceleration = convertAcceleration(XCombined);
    if (MMA8451Q_RAW_DATA_COLLECT == 1){warpPrint("XAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n", XAcceleration, XAcceleration);}

    // XVariance *= (totalSamples - 1); // Undo the division by n when the variance was calculated last.
    // XVariance += (XAcceleration*XAcceleration);
    // XVariance /= totalSamples; // Implement the new division by n for this sample.
    // warpPrint("XVariance = %d.\n", XVariance);
    
    // timediff_poll[0] = OSA_TimeGetMsec() - timeBefore_poll;
    // timeBefore_poll = OSA_TimeGetMsec(); // Variance calculations will be part of the program - therefore keep them within the time measurement for polling + variance calculation delay
    

    y_MSB = deviceMMA8451QState.i2cBuffer[2];
    y_LSB = deviceMMA8451QState.i2cBuffer[3];
    YCombined = ((y_MSB & 0xFF) << 6) | (y_LSB >> 2);
    YCombined = (YCombined ^ (1 << 13)) - (1 << 13);
    //warpPrint("y_MSB: %d, y_MSB: %d, YCombined - Decimal: %d, Hexadecimal: %x.\n", y_MSB, y_LSB, YCombined, YCombined);
    YAcceleration = convertAcceleration(YCombined);
    
    if (MMA8451Q_RAW_DATA_COLLECT == 1){warpPrint("YAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n", YAcceleration, YAcceleration);}
    
    // YVariance *= (totalSamples - 1); // Undo the division by n when the variance was calculated last.
    // YVariance += (YAcceleration*YAcceleration);
    // YVariance /= totalSamples; // Implement the new division by n for this sample.
    // warpPrint("YVariance = %d.\n", YVariance);
    
    // timediff_poll[1] = OSA_TimeGetMsec() - timeBefore_poll;
    // timeBefore_poll = OSA_TimeGetMsec();
    
    z_MSB = deviceMMA8451QState.i2cBuffer[4];
    z_LSB = deviceMMA8451QState.i2cBuffer[5];
    ZCombined = ((z_MSB & 0xFF) << 6) | (z_LSB >> 2);
    ZCombined = (ZCombined ^ (1 << 13)) - (1 << 13);
    //warpPrint("z_MSB: %d, z_MSB: %d, ZCombined - Decimal: %d, Hexadecimal: %x.\n", z_MSB, z_LSB, ZCombined, ZCombined);
    ZAcceleration = convertAcceleration(ZCombined);
    if (MMA8451Q_RAW_DATA_COLLECT == 1){ warpPrint("ZAcceleration (mms^-2) - Decimal: %d, Hexadecimal: %x.\n\n", ZAcceleration, ZAcceleration);}

    // ZVariance *= (totalSamples - 1); // Undo the division by n when the variance was calculated last.
    // ZVariance += (ZAcceleration*ZAcceleration);
    // ZVariance /= totalSamples; // Implement the new division by n for this sample.
    // warpPrint("ZVariance = %d.\n", ZVariance);
    
    timediff_poll[2] = OSA_TimeGetMsec() - timeBefore_poll;
    
    // Testing with known values
    //uint32_t acc_magntiude = get_sqrt((uint32_t)2500);
    uint32_t acc_magntiude = get_sqrt((uint32_t)(ZAcceleration*ZAcceleration) + (uint32_t)(YAcceleration*YAcceleration) + (uint32_t)(XAcceleration*XAcceleration));
    Acc_mag_Variance = (int64_t)acc_magntiude;

    // Update buffer index (circular) - adding both time delay between function call and time difference for polling registers 
        
    update_buffers(acc_magntiude, sampling_time_delta); 
	uint64_t CoVar_XYZ = propagate_std_dev((uint64_t)(XAcceleration*XAcceleration), (uint64_t)(YAcceleration*YAcceleration), (uint64_t)(ZAcceleration*ZAcceleration),  
        X_SD, Y_SD, Z_SD);
    
    if (MMA8451Q_RAW_DATA_COLLECT == 1){
        warpPrint("Magnitude of acceleration: %d \n", acc_magntiude);
        //warpPrint("Mean polling delay: %d us \n", ((timediff_poll[0] + timediff_poll[1] + timediff_poll[2]) * 1000) / 3); //Scaling up to get values after the decimal point - into warpPrint
    
        warpPrint("Total polling delay: %d us \n", timediff_poll[2] * 1000); // uncomment out, delete and remove the ); / 3); //Scaling up to get values after the decimal point - into warpPrint
        warpPrint("Instantaneous Co-Variance: %llu", (uint32_t)(CoVar_XYZ/1000));
    }
    
    return acc_magntiude;
}


uint32_t f_peak(int target_freq, uint32_t spectrum[NUM_FREQS]){
    uint32_t full_power = 0;
    for (int i = 0; i < NUM_FREQS; i++) {
        full_power = full_power + spectrum[i]; 
    }
    return spectrum[target_freq-2]/full_power; //Frequencies range from 2-13Hz - indices range from 0-11 (12 total bins)
}



uint32_t calculate_baysean(int max_pwr_index, uint32_t power_dist[NUM_FREQS]){
    uint32_t P_of_f_given_H1; uint32_t P_of_f_given_H0;

    P_of_f_given_H1 = PDF_parkinsonian[max_pwr_index];
    P_of_f_given_H0 = PDF_non_parkinsonian[max_pwr_index];
    
    // 64 bit unsigned typecasting to get any reaonable probability values at very low confidence levels
    uint64_t numerator = (uint64_t) P_H1 * (uint64_t) P_of_f_given_H1;
    uint64_t denominator = (uint64_t) P_H1 * (uint64_t) P_of_f_given_H1 + 
                       (uint64_t) P_H0 * (uint64_t) P_of_f_given_H0;

    // printouts for debugging values: 
    // warpPrint("\nNumerator: %u\n", P_of_f_given_H1 * P_H1);
    // warpPrint("\nDenominator: %u \n", (P_H1 * P_of_f_given_H1 + P_H0 * P_of_f_given_H0));
    uint64_t final_P = 0;
    if (denominator > 0) {
        final_P = (1000*numerator / denominator); // Safe division after scaling
    } else {
        warpPrint("\nERROR: Denominator is ZERO!");
        final_P = 0;
    }               
    //warpPrint("\nP_H1_given_f: {%u}\n", P_H1_given_f);
    warpPrint("\nFrequency bin with peak power: %d Hz. \n P_H1_given_f: {%u}/1000 \n", target_freqs[max_pwr_index], final_P); //Rescaled output probability by 1000 to accomodate for decent resolution of proability with warpPrint.
    return;
}