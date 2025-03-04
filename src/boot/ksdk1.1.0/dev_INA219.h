
void		init_INA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);

// The remaining lines will need to be reconfigured as per the data - to set the appropriate register buffers 
WarpStatus	readSensorRegister_INA219(uint8_t deviceRegister,  int numberOfBytes); // All registers are actually 8 bit - 16 bit is the payload
WarpStatus	writeSensorRegister_INA219(uint8_t deviceRegister, uint16_t payloadBtye);
WarpStatus 	configureSensor_INA219(uint16_t configPayload, uint16_t calibrationPayload);
const char* formatINA219Register(uint8_t regAddress, uint8_t msb, uint8_t lsb);
void		printSensorData_INA219(bool hexModeFlag);
uint8_t		appendSensorData_INA219(uint16_t* buf);

/*
WarpStatus	readSensorRegister_INA219(uint8_t deviceRegister,  uint16_t *readValue); // All registers are actually 8 bit - 16 bit is the payload


Might want to remove the pointer and go back to int numb of byutes --> We just want to loop forr sensorr data.

*/

const uint8_t bytesPerMeasurement_INA219              = 2; // Will need to change this - maybe not, as we are still just reading voltage, current and power - each comprising of 2 bytes each 
const uint8_t bytesPerReading_INA219                = 2;
const uint8_t numberOfReadingsPerMeasurement_INA219 = 3;