
uint32_t byte_to_state_conversion();
void update_buffers(uint32_t acc_mag, uint16_t time_diff);

int32_t convertAcceleration(int16_t number);
int32_t get_sqrt(uint32_t magntiude); //<-- Magntiude will be an unsigned int, so uint32 - want large integer size for the calculation

uint16_t XCombined;
uint16_t YCombined;
uint16_t ZCombined;

#define BUFF_SIZE 40
uint32_t AccelerationBuffer[BUFF_SIZE] = {0}; // Initialised to 0.
uint32_t LPFBuffer[BUFF_SIZE] = {0}; // Initialised to 0.

/* 
Variables for updating a rolling buffer for storing acceleration magntiudes and times  - thes eneed to be in the header, as the script file and internal functions 
may be called multiple times. this may overwrite and reset the circular buffer and associated variables each time.
*/

uint16_t timeBefore = 0;
uint16_t timeAft = 0;

//int delay_ms = 500; --> Removing as this will take more symbol space
