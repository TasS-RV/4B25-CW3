/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/

void		initMMA8451Q(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);

// the remainder will all need to be renamed and recconfigured as per the datasheet
WarpStatus	readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterMMA8451Q(uint8_t deviceRegister, uint8_t payloadBtye);
WarpStatus 	configureSensorMMA8451Q(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1, uint8_t payloadHP_FILTER_CUTOFF, uint8_t payloadXYZ_DATA_CFG);
void		printSensorDataMMA8451Q(bool hexModeFlag);
uint8_t		appendSensorDataMMA8451Q(uint8_t* buf);
void update_buffers(uint32_t acc_mag, uint16_t time_diff);

extern volatile WarpI2CDeviceState	deviceMMA8451QState;


const uint8_t bytesPerMeasurementMMA8451Q            = 6;
const uint8_t bytesPerReadingMMA8451Q                = 2;
const uint8_t numberOfReadingsPerMeasurementMMA8451Q = 3;


// Standard deviations (*10) modelling x, y, z accelerations as additive white Gaussian noise from Python fitting - remainding decimal points must be omitted.
uint64_t X_SD = 194;// .30;
uint64_t Y_SD = 329;// .40;
uint64_t Z_SD = 204;// .83;


// Stores the current power calculation, computed at the end of the rolling window
uint64_t window_power = 0;

uint64_t propagate_std_dev(uint64_t x, uint64_t y, uint64_t z, uint64_t sigma_x, uint64_t sigma_y, uint64_t sigma_z);
