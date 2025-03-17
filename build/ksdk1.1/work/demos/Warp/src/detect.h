
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

volatile int buffer_index = 0;
static uint32_t accel_magnitude_buffer[BUFF_SIZE]; //<-- SHould be BUFF_size
static uint16_t time_steps_buffer[BUFF_SIZE];

// For computing thhe added time delay resulting in a reduction in effective polling frequency
uint16_t timeBefore = 0;
uint16_t timeAft = 0;

// Function for updating Gopertzel array of values - instead of storing the whole BUFF_SIZE of Y_n values, it only stores the last 2 and current one. 
void update_goertzel(uint32_t x_n);

#define NUM_FREQS 5
// Y_values for NUM_FREQ number of frequency bins for Goertzel FFT
uint32_t y_values[NUM_FREQS][2] = {0};

