
void byte_to_state_conversion();
int32_t convertAcceleration(int16_t number);
int32_t get_sqrt(uint32_t magntiude); //<-- Magntiude will be an unsigned int, so uint32 - want large integer size for the calculation

uint16_t XCombined;
uint16_t YCombined;
uint16_t ZCombined;

#define BUFF_SIZE 40
uint32_t AccelerationBuffer[BUFF_SIZE] = {0}; // Initialised to 0.
uint32_t LPFBuffer[BUFF_SIZE] = {0}; // Initialised to 0.