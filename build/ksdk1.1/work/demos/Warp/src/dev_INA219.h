
void		init_INA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);


WarpStatus	readSensorRegister_INA219(uint8_t deviceRegister,  int numberOfBytes); // All registers are actually 8 bit - 16 bit is the payload
WarpStatus	writeSensorRegister_INA219(uint8_t deviceRegister, uint16_t payloadBtye);
WarpStatus 	configureSensor_INA219(uint16_t configPayload, uint16_t calibrationPayload);
void		printSensorData_INA219(bool hexModeFlag);
uint8_t		appendSensorData_INA219(uint16_t* buf);
void parseINA219Register(uint8_t regAddress, uint16_t rawValue, int CALIB_VALUE); 


const uint8_t bytesPerMeasurement_INA219              = 2; // Payload for each register of the INA219 is actually 16 bits - 2 bytes as indicated below. 
const uint8_t bytesPerReading_INA219                = 2;
const uint8_t numberOfReadingsPerMeasurement_INA219 = 3;