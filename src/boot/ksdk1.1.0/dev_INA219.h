
void		init_INA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);

// The remaining lines will need to be reconfigured as per the data - to set the appropriate register buffers
WarpStatus	readSensorRegister_INA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegister_INA219(uint8_t deviceRegister, uint8_t payloadBtye);
WarpStatus 	configureSensor_INA219(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1);
void		printSensorData_INA219(bool hexModeFlag);
uint8_t		appendSensorData_INA219(uint8_t* buf);

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
const uint8_t bytesPerMeasurement_INA219            = 6; // Will need tyo change this
const uint8_t bytesPerReading_INA219                = 2;
const uint8_t numberOfReadingsPerMeasurement_INA219 = 3;