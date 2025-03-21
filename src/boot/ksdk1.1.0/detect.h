
uint32_t byte_to_state_conversion(uint16_t sampling_time_delta);

int32_t convertAcceleration(int16_t number);
int32_t get_sqrt(uint32_t magntiude); //<-- Magntiude will be an unsigned int, so uint32 - want large integer size for the calculation

uint16_t XCombined;
uint16_t YCombined;
uint16_t ZCombined;


#define BUFF_SIZE 40 //At 40Hz, set to update power every 1s
// uint32_t AccelerationBuffer[BUFF_SIZE] = {0}; // Initialised to 0.
// uint32_t LPFBuffer[BUFF_SIZE] = {0}; // Initialised to 0.

/* 
Variables for updating a rolling buffer for storing acceleration magntiudes and times  - thes eneed to be in the header, as the script file and internal functions 
may be called multiple times. this may overwrite and reset the circular buffer and associated variables each time.
*/

volatile int buffer_index = 0;
static uint32_t accel_magnitude_buffer[BUFF_SIZE]; //<-- SHould be BUFF_size
static uint16_t time_steps_buffer[BUFF_SIZE];

// For computing thhe added time delay resulting in a reduction in effective polling frequency
uint16_t timeBefore_poll = 0;
uint16_t timeAft_poll = 0;

// Number of discrete frequency bins
#define NUM_FREQS 12


int32_t bayes_freq_probability(int16_t number); //<-- MAy need to pass in an array



// Function for updating Gopertzel array of values - instead of storing the whole BUFF_SIZE of Y_n values, it only stores the last 2 and current one. 
void update_goertzel(uint32_t x_n, uint64_t Acc_mag_Variance);
uint32_t compute_goertzel_power();


const uint32_t target_freqs[NUM_FREQS] = {2, 3, 4, 5, 6, 7, 8, 9 ,10, 11, 12, 13};  // Hz - same bit field size for math
// Y_values for NUM_FREQ number of frequency bins for Goertzel FFT
int32_t y_values[NUM_FREQS][2] = {0};

//float Y_Vars[NUM_FREQS][2] = {0};  // variances of yN-2, yN-1, and yN-3
//float Covars_Y[NUM_FREQS][2] = {0};  // variances of yN-2, yN-1, and yN-3
uint64_t Var_Y_N = 0;
int64_t Y_Vars[NUM_FREQS][2] = {0};  // variances of yN-2, yN-1, and yN-3
int64_t Covars_Y[NUM_FREQS][2] = {0};  // variances of yN-2, yN-1, and yN-3



// Section associated with Bayseian probability variables - based on training data: will use standard deviation instead of variance, as to get SD from var requires rooting. Squaring to go the other way is easier.
uint16_t mu_known[NUM_FREQS] = {0};
uint16_t sigma_known[NUM_FREQS] = {0};

uint32_t calculate_baysean(int max_pwr_index, uint32_t power_dist[NUM_FREQS]);
// All frequencies x100000s for integer math
uint32_t PDF_parkinsonian[NUM_FREQS] =      {72, 780, 29844, 33404, 28674, 3930, 1414, 877, 121, 99, 360, 425};  // PDF(H1) - probability spectrum for true hypothesis. Non_gaussian - instead concentration of probaabilities around 4-6Hz Parkinsonian frequencies 
uint32_t PDF_non_parkinsonian[NUM_FREQS] = {6654, 7865, 13294, 15042, 13299, 9772, 15092, 7222, 4482, 1793, 2791, 2694}; // PDF(H0) - probability distribution for false hypothesis  

uint16_t P_H1 = 40000;  // Arbitrary 3% of population with Parkinson's.
uint16_t P_H0 = 60000; // 97% probability/ proportion of populaiton without Parkinson's