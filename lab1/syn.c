/**
 *  V. Hunter Adams (vha3@cornell.edu)
 
    This is an experiment with the multicore capabilities on the
    RP2040. The program instantiates a timer interrupt on each core.
    Each of these timer interrupts writes to a separate channel
    of the SPI DAC and does DDS of two sine waves of two different
    frequencies. These sine waves are amplitude-modulated to "beeps."

    No spinlock is required to mediate the SPI writes because of the
    SPI buffer on the RP2040. Spinlocks are used in the main program
    running on each core to lock the other out from an incrementing
    global variable. These are "under the hood" of the PT_SEM_SAFE_x
    macros. Two threads ping-pong using these semaphores.

    Note that globals are visible from both cores. Note also that GPIO
    pin mappings performed on core 0 can be utilized from core 1.
    Creation of an alarm pool is required to force a timer interrupt to
    take place on core 1 rather than core 0.

 */

// Include necessary libraries
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

// Include VGA graphics library
#include "vga_graphics.h"
// Include standard libraries
#include <math.h>
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"



// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

/////////////////////////// ADC configuration ////////////////////////////////
// ADC Channel and pin
#define ADC_CHAN 0
#define ADC_PIN 26
// Number of samples per FFT
#define NUM_SAMPLES 1024
// Number of samples per FFT, minus 1
#define NUM_SAMPLES_M_1 1023
// Length of short (16 bits) minus log2 number of samples (10)
#define SHIFT_AMOUNT 6
// Log2 number of samples
#define LOG2_NUM_SAMPLES 10
// Sample rate (Hz)
#define Fs 10000.0
// ADC clock rate (unmutable!)
#define ADCCLK 48000000.0

// DMA channels for sampling ADC (VGA driver uses 0 and 1)
int sample_chan = 2 ;
int control_chan = 3 ;

// Max and min macros
#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)

// detect chirp
bool HIGH = 0;

// 0.4 in fixed point (used for alpha max plus beta min)
fix15 zero_point_4 = float2fix15(0.4) ;

// Here's where we'll have the DMA channel put ADC samples
uint8_t sample_array[NUM_SAMPLES] ;
// And here's where we'll copy those samples for FFT calculation
fix15 fr[NUM_SAMPLES] ;
fix15 fi[NUM_SAMPLES] ;

// Sine table for the FFT calculation
fix15 Sinewave[NUM_SAMPLES]; 
// Hann window table for FFT calculation
fix15 window[NUM_SAMPLES]; 

// Pointer to address of start of sample buffer
uint8_t * sample_address_pointer = &sample_array[0] ;

// Peforms an in-place FFT. For more information about how this
// algorithm works, please see https://vanhunteradams.com/FFT/FFT.html
void FFTfix(fix15 fr[], fix15 fi[]) {
    
    unsigned short m;   // one of the indices being swapped
    unsigned short mr ; // the other index being swapped (r for reversed)
    fix15 tr, ti ; // for temporary storage while swapping, and during iteration
    
    int i, j ; // indices being combined in Danielson-Lanczos part of the algorithm
    int L ;    // length of the FFT's being combined
    int k ;    // used for looking up trig values from sine table
    
    int istep ; // length of the FFT which results from combining two FFT's
    
    fix15 wr, wi ; // trigonometric values from lookup table
    fix15 qr, qi ; // temporary variables used during DL part of the algorithm
    
    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// BIT REVERSAL //////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Bit reversal code below based on that found here: 
    // https://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
    for (m=1; m<NUM_SAMPLES_M_1; m++) {
        // swap odd and even bits
        mr = ((m >> 1) & 0x5555) | ((m & 0x5555) << 1);
        // swap consecutive pairs
        mr = ((mr >> 2) & 0x3333) | ((mr & 0x3333) << 2);
        // swap nibbles ... 
        mr = ((mr >> 4) & 0x0F0F) | ((mr & 0x0F0F) << 4);
        // swap bytes
        mr = ((mr >> 8) & 0x00FF) | ((mr & 0x00FF) << 8);
        // shift down mr
        mr >>= SHIFT_AMOUNT ;
        // don't swap that which has already been swapped
        if (mr<=m) continue ;
        // swap the bit-reveresed indices
        tr = fr[m] ;
        fr[m] = fr[mr] ;
        fr[mr] = tr ;
        ti = fi[m] ;
        fi[m] = fi[mr] ;
        fi[mr] = ti ;
    }
    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// Danielson-Lanczos //////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Adapted from code by:
    // Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
    // Length of the FFT's being combined (starts at 1)
    L = 1 ;
    // Log2 of number of samples, minus 1
    k = LOG2_NUM_SAMPLES - 1 ;
    // While the length of the FFT's being combined is less than the number 
    // of gathered samples . . .
    while (L < NUM_SAMPLES) {
        // Determine the length of the FFT which will result from combining two FFT's
        istep = L<<1 ;
        // For each element in the FFT's that are being combined . . .
        for (m=0; m<L; ++m) { 
            // Lookup the trig values for that element
            j = m << k ;                         // index of the sine table
            wr =  Sinewave[j + NUM_SAMPLES/4] ; // cos(2pi m/N)
            wi = -Sinewave[j] ;                 // sin(2pi m/N)
            wr >>= 1 ;                          // divide by two
            wi >>= 1 ;                          // divide by two
            // i gets the index of one of the FFT elements being combined
            for (i=m; i<NUM_SAMPLES; i+=istep) {
                // j gets the index of the FFT element being combined with i
                j = i + L ;
                // compute the trig terms (bottom half of the above matrix)
                tr = multfix15(wr, fr[j]) - multfix15(wi, fi[j]) ;
                ti = multfix15(wr, fi[j]) + multfix15(wi, fr[j]) ;
                // divide ith index elements by two (top half of above matrix)
                qr = fr[i]>>1 ;
                qi = fi[i]>>1 ;
                // compute the new values at each index
                fr[j] = qr - tr ;
                fi[j] = qi - ti ;
                fr[i] = qr + tr ;
                fi[i] = qi + ti ;
            }    
        }
        --k ;
        L = istep ;
    }
}

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs_0 40000            // sample rate
#define epsilon 10 // increment value

// the DDS units - core 1
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_1;                  
volatile unsigned int phase_incr_main_1 = (2300.0*two32)/Fs_0 ;
// the DDS units - core 2
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (2300.0*two32)/Fs_0 ;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             240
#define DECAY_TIME              240
#define SUSTAIN_TIME            200
#define BEEP_DURATION           680
#define BEEP_REPEAT_INTERVAL    80
#define CHIRP_REPEAT_DURATION   6080
#define CHIRP_REPEAT_INTERVAL   10320

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int STATE_0_cycle = 0 ;
volatile unsigned int count_0 = 0 ;
volatile unsigned int count_0_c = 0 ;
volatile unsigned int STATE_1 = 0 ;
volatile unsigned int STATE_1_cycle = 0 ;
volatile unsigned int count_1 = 0 ;
volatile unsigned int count_1_c = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define button0  14
#define button1  15
#define SPI_PORT spi0

// Two variables to store core number
volatile int corenum_0  ;
volatile int corenum_1  ;

// Global counter for spinlock experimenting
volatile int global_counter = 0 ;

// Semaphore
struct pt_sem core_1_go, core_0_go ;

// state and count adjustment for sync
void sync(core_num) {
    // sync core 0
    if(!core_num) {
        if(STATE_0_cycle == 1 && count_0_c >= pow(sqrt(16400) - epsilon, 2) - 6080) {
            STATE_0_cycle = 0;
            STATE_0 = 0;
            count_0 = 0;
            count_0_c = 0;
        } else if (STATE_0_cycle == 1 && count_0_c < pow(sqrt(16400) - epsilon, 2) - 6080) {
            STATE_0_cycle = 1;
            STATE_0 = 0;
            count_0 = 0;
            count_0_c = pow(sqrt(count_0_c + 6080) + epsilon, 2) - 6080;
        } else if (STATE_0_cycle == 0 && count_0_c >= pow(sqrt(6080) - epsilon, 2)) {
            STATE_0_cycle = 1;
            STATE_0 = 0;
            count_0 = 0;
            count_0_c = pow(sqrt(count_0_c) + epsilon, 2) - 6080;
        } else if (STATE_0_cycle == 0 && count_0_c < pow(sqrt(6080) - epsilon, 2)) {
            count_0_c = pow(sqrt(count_0_c) + epsilon, 2);
            STATE_0_cycle = 0;
            volatile unsigned int rem = count_0_c % 760;
            if(rem <= 680) {
                STATE_0 = 0;
            } else {
                STATE_0 = 1;
            }
            if (rem <= 240) {
                count_0 = rem;
            } else if (rem <= 440) {
                count_0 = rem - 240;
            } else if (rem <= 680) {
                count_0 = rem - 440;
            } else {
                count_0 = rem - 680;
            }  
        }    
    // sync core 1
    } else {
        if(STATE_1_cycle == 1 && count_1_c >= pow(sqrt(16400) - epsilon, 2) - 6080) {
            STATE_1_cycle = 0;
            STATE_1 = 0;
            count_1 = 0;
            count_1_c = 0;
        } else if (STATE_1_cycle == 1 && count_1_c < pow(sqrt(16400) - epsilon, 2) - 6080) {
            STATE_1_cycle = 1;
            STATE_1 = 0;
            count_1 = 0;
            count_1_c = pow(sqrt(count_1_c + 6080) + epsilon, 2) - 6080;
        } else if (STATE_1_cycle == 0 && count_1_c >= pow(sqrt(6080) - epsilon, 2)) {
            STATE_1_cycle = 1;
            STATE_1 = 0;
            count_1 = 0;
            count_1_c = pow(sqrt(count_1_c) + epsilon, 2) - 6080;
        } else if (STATE_1_cycle == 0 && count_1_c < pow(sqrt(6080) - epsilon, 2)) {
            count_1_c = pow(sqrt(count_1_c) + epsilon, 2);
            STATE_1_cycle = 0;
            volatile unsigned int rem = count_1_c % 760;
            if(rem <= 680) {
                STATE_1 = 0;
            } else {
                STATE_1 = 1;
            }
            if (rem <= 240) {
                count_1 = rem;
            } else if (rem <= 440) {
                count_1 = rem - 240;
            } else if (rem <= 680) {
                count_1 = rem - 440;
            } else {
                count_1 = rem - 680;
            }  
    }
}

// This timer ISR is called on core 1
bool repeating_timer_callback_core_1(struct repeating_timer *t) {
    if(gpio_get(button1) == 1) {
        STATE_1_cycle = 0;
        count_1 = 0;
        count_1_c = 0;
    } else { 
    if(STATE_1_cycle == 0) {
        if (STATE_1 == 0) {
            // DDS phase and sine table lookup
            phase_accum_main_1 += phase_incr_main_1  ;
            DAC_output_1 = fix2int15(multfix15(current_amplitude_1,
                sin_table[phase_accum_main_1>>24])) + 2048 ;

            // Ramp up amplitude
            if (count_1 < ATTACK_TIME) {
                current_amplitude_1 = (current_amplitude_1 + attack_inc) ;
            }
            // Ramp down amplitude
            else if (count_1 > BEEP_DURATION - DECAY_TIME) {
                current_amplitude_1 = (current_amplitude_1 - decay_inc) ;
            }

            // Mask with DAC control bits
            DAC_data_1 = (DAC_config_chan_A | (DAC_output_1 & 0xffff))  ;

            // SPI write (no spinlock b/c of SPI buffer)
            spi_write16_blocking(SPI_PORT, &DAC_data_1, 1) ;

            // Increment the counter
            count_1 += 1 ;
            count_1_c += 1 ;

            // State transition?
            if (count_1 == BEEP_DURATION) {
                STATE_1 = 1 ;
                count_1 = 0 ;
            }
        }
        // State transition?
        else {
            count_1 += 1 ;
            count_1_c += 1 ;
            if (count_1 == BEEP_REPEAT_INTERVAL) {
                current_amplitude_1 = 0 ;
                STATE_1 = 0 ;
                count_1 = 0 ;
            }
        }
        if (count_1_c == CHIRP_REPEAT_DURATION) {
            STATE_1_cycle = 1 ;
            count_1_c = 0 ;
        }       
    
    } else {
        if(count_1_c == CHIRP_REPEAT_INTERVAL) {
            STATE_1_cycle = 0 ;
            count_1_c = 0 ;
        } else {
            current_amplitude_1 = 0 ;
            count_1_c += 1;
        }   
    }
    }
  // retrieve core number of execution
    corenum_1 = get_core_num() ;

    return true;
}

// This timer ISR is called on core 0
bool repeating_timer_callback_core_0(struct repeating_timer *t) {
    if(gpio_get(button0) == 1) {
        STATE_0_cycle = 0;
        count_0 = 0;
        count_0_c = 0;
    }    else {

    if(STATE_0_cycle == 0) {
        if (STATE_0 == 0) {
            // DDS phase and sine table lookup
            phase_accum_main_0 += phase_incr_main_0  ;
            DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
                sin_table[phase_accum_main_0>>24])) + 2048 ;

            // Ramp up amplitude
            if (count_0 < ATTACK_TIME) {
                current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
            }
            // Ramp down amplitude
            else if (count_0 > BEEP_DURATION - DECAY_TIME) {
                current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
            }

            // Mask with DAC control bits
            DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

            // SPI write (no spinlock b/c of SPI buffer)
            spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

            // Increment the counter
            count_0 += 1 ;
            count_0_c += 1 ;

            // State transition?
            if (count_0 == BEEP_DURATION) {
                STATE_0 = 1 ;
                count_0 = 0 ;
            }
        }

        // State transition?
        else {
            count_0 += 1 ;
            count_0_c += 1 ;
            if (count_0 == BEEP_REPEAT_INTERVAL) {
                current_amplitude_0 = 0 ;
                STATE_0 = 0 ;
                count_0 = 0 ;
            }
        }
        if (count_0_c == CHIRP_REPEAT_DURATION) {
            STATE_0_cycle = 1 ;
            count_0_c = 0 ;
        }  

    } else {
        if(count_0_c == CHIRP_REPEAT_INTERVAL) {
            STATE_0_cycle = 0 ;
            count_0_c = 0 ;
        } else {
            current_amplitude_0 = 0 ;
            count_0_c += 1;
        }   
    }
    }
    // retrieve core number of execution
    corenum_0 = get_core_num() ;

    return true;
}

// This thread runs on core 1
static PT_THREAD (protothread_core_1(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;
    while(1) {
        // Wait for signal
        PT_SEM_SAFE_WAIT(pt, &core_1_go) ;
        // Turn off LED
        gpio_put(LED, 0) ;
        // Increment global counter variable
        for (int i=0; i<10; i++) {
            global_counter += 1 ;
            sleep_ms(250) ;
            printf("Core 1: %d, ISR core: %d\n", global_counter, corenum_1) ;
        }
        printf("\n\n") ;
        // signal other core
        PT_SEM_SAFE_SIGNAL(pt, &core_0_go) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;
    while(1) {
        // Wait for signal
        PT_SEM_SAFE_WAIT(pt, &core_0_go) ;
        // Turn on LED
        gpio_put(LED, 1) ;
        // Increment global counter variable
        for (int i=0; i<10; i++) {
            global_counter += 1 ;
            sleep_ms(250) ;
            printf("Core 0: %d, ISR core: %d\n", global_counter, corenum_0) ;
        }
        printf("\n\n") ;
        // signal other core
        PT_SEM_SAFE_SIGNAL(pt, &core_1_go) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

// Runs on core 0
static PT_THREAD (protothread_fft(struct pt *pt))
{
    // Indicate beginning of thread
    PT_BEGIN(pt) ;
    printf("Starting capture\n") ;
    // Start the ADC channel
    dma_start_channel_mask((1u << sample_chan)) ;
    // Start the ADC
    adc_run(true) ;

    // Declare some static variables
    static int height ;             // for scaling display
    static float max_freqency ;     // holds max frequency
    static int i ;                  // incrementing loop variable

    static fix15 max_fr ;           // temporary variable for max freq calculation
    static int max_fr_dex ;         // index of max frequency

    // Write some text to VGA
    setTextColor(WHITE) ;
    setCursor(65, 0) ;
    setTextSize(1) ;
    writeString("Raspberry Pi Pico") ;
    setCursor(65, 10) ;
    writeString("FFT demo") ;
    setCursor(65, 20) ;
    writeString("Hunter Adams") ;
    setCursor(65, 30) ;
    writeString("vha3@cornell.edu") ;
    setCursor(250, 0) ;
    setTextSize(2) ;
    writeString("Max freqency:") ;

    // Will be used to write dynamic text to screen
    static char freqtext[40];



    while(1) {
        // Wait for NUM_SAMPLES samples to be gathered
        // Measure wait time with timer. THIS IS BLOCKING
        dma_channel_wait_for_finish_blocking(sample_chan);

        // Copy/window elements into a fixed-point array
        for (i=0; i<NUM_SAMPLES; i++) {
            fr[i] = multfix15(int2fix15((int)sample_array[i]), window[i]) ;
            fi[i] = (fix15) 0 ;
        }

        // Zero max frequency and max frequency index
        max_fr = 0 ;
        max_fr_dex = 0 ;

        // Restart the sample channel, now that we have our copy of the samples
        dma_channel_start(control_chan) ;

        // Compute the FFT
        FFTfix(fr, fi) ;

        // Find the magnitudes (alpha max plus beta min)
        for (int i = 0; i < (NUM_SAMPLES>>1); i++) {  
            // get the approx magnitude
            fr[i] = abs(fr[i]); 
            fi[i] = abs(fi[i]);
            // reuse fr to hold magnitude
            fr[i] = max(fr[i], fi[i]) + 
                    multfix15(min(fr[i], fi[i]), zero_point_4); 

            // Keep track of maximum
            if (fr[i] > max_fr && i>4) {
                max_fr = fr[i] ;
                max_fr_dex = i ;
            }
        }
        // Compute max frequency in Hz
        max_freqency = max_fr_dex * (Fs/NUM_SAMPLES) ;
        if(max_freqency > 2200 && max_freqency < 2400) {
            if(!HIGH){
                HIGH = true;
                if (STATE_0_cycle == 1) {
                    printf("core 0 detected a chirp\n");
                    sync(0);
                }

                if(STATE_1_cycle == 1) {
                    printf("core 1 detected a chirp\n");
                    sync(1);
                }
            } else {
            HIGH = false;
            }
        }


        // Display on VGA
        fillRect(250, 20, 176, 30, BLACK); // red box
        sprintf(freqtext, "%d", (int)max_freqency) ;
        setCursor(250, 20) ;
        setTextSize(2) ;
        writeString(freqtext) ;

        // Update the FFT display
        for (int i=5; i<(NUM_SAMPLES>>1); i++) {
            drawVLine(59+i, 50, 429, BLACK);
            height = fix2int15(multfix15(fr[i], int2fix15(36))) ;
            drawVLine(59+i, 479-height, height, WHITE);
        }

    }
    PT_END(pt) ;
}

static PT_THREAD (protothread_blink(struct pt *pt))
{
    // Indicate beginning of thread
    PT_BEGIN(pt) ;
    while (1) {
        // Toggle LED, then wait half a second
        gpio_put(LED, !gpio_get(LED)) ;
        PT_YIELD_usec(500000) ;
    }
    PT_END(pt) ;
}




// This is the core 1 entry point. Essentially main() for core 1
void core1_entry() {
    //gpio initialization
    gpio_init(button1);
    gpio_set_dir(button1, GPIO_IN);

    // create an alarm pool on core 1
    alarm_pool_t *core1pool ;
    core1pool = alarm_pool_create(2, 16) ;

    // Create a repeating timer that calls repeating_timer_callback.
    struct repeating_timer timer_core_1;

    // Negative delay so means we will call repeating_timer_callback, and call it
    // again 25us (40kHz) later regardless of how long the callback took to execute
    alarm_pool_add_repeating_timer_us(core1pool, -25, 
        repeating_timer_callback_core_1, NULL, &timer_core_1);

    // Add thread to core 1
    pt_add_thread(protothread_core_1) ;
    // Add and schedule threads
    pt_add_thread(protothread_blink) ;

    // Start scheduler on core 1
    pt_schedule_start ;

}


// Core 0 entry point
int main() {
    // Initialize stdio/uart (printf won't work unless you do this!)
    stdio_init_all();
    printf("Hello, friends!\n");
    // //gpio initialization
    gpio_init(button0);
    gpio_set_dir(button0, GPIO_IN);

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Initialize the intercore semaphores
    PT_SEM_SAFE_INIT(&core_0_go, 1) ;
    PT_SEM_SAFE_INIT(&core_1_go, 0) ;

    // Initialize the VGA screen
    initVGA() ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    ///////////////////////////////////////////////////////////////////////////////
    // ============================== ADC CONFIGURATION ==========================
    //////////////////////////////////////////////////////////////////////////////
    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(ADC_PIN);

    // Initialize the ADC harware
    // (resets it, enables the clock, spins until the hardware is ready)
    adc_init() ;

    // Select analog mux input (0...3 are GPIO 26, 27, 28, 29; 4 is temp sensor)
    adc_select_input(ADC_CHAN) ;

    // Setup the FIFO
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock. This is setup
    // to grab a sample at 10kHz (48Mhz/10kHz - 1)
    adc_set_clkdiv(ADCCLK/Fs);


    // Populate the sine table and Hann window table
    int ii_1;
    for (ii_1 = 0; ii_1 < NUM_SAMPLES; ii_1++) {
        Sinewave[ii_1] = float2fix15(sin(6.283 * ((float) ii_1) / (float)NUM_SAMPLES));
        window[ii_1] = float2fix15(0.5 * (1.0 - cos(6.283 * ((float) ii_1) / ((float)NUM_SAMPLES))));
    }

    /////////////////////////////////////////////////////////////////////////////////
    // ============================== ADC DMA CONFIGURATION =========================
    /////////////////////////////////////////////////////////////////////////////////

    // Channel configurations
    dma_channel_config c2 = dma_channel_get_default_config(sample_chan);
    dma_channel_config c3 = dma_channel_get_default_config(control_chan);


    // ADC SAMPLE CHANNEL
    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);
    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&c2, DREQ_ADC);
    // Configure the channel
    dma_channel_configure(sample_chan,
        &c2,            // channel config
        sample_array,   // dst
        &adc_hw->fifo,  // src
        NUM_SAMPLES,    // transfer count
        false            // don't start immediately
    );

    // CONTROL CHANNEL
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);      // 32-bit txfers
    channel_config_set_read_increment(&c3, false);                // no read incrementing
    channel_config_set_write_increment(&c3, false);               // no write incrementing
    channel_config_set_chain_to(&c3, sample_chan);                // chain to sample chan

    dma_channel_configure(
        control_chan,                         // Channel to be configured
        &c3,                                // The configuration we just created
        &dma_hw->ch[sample_chan].write_addr,  // Write address (channel 0 read address)
        &sample_address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers, in this case each is 4 byte
        false                               // Don't start immediately.
    );

    // Launch core 1
    multicore_launch_core1(core1_entry);

    // Desynchronize the beeps
    sleep_ms(500) ;

    // Create a repeating timer that calls 
    // repeating_timer_callback (defaults core 0)
    struct repeating_timer timer_core_0;

    // Negative delay so means we will call repeating_timer_callback, and call it
    // again 25us (40kHz) later regardless of how long the callback took to execute
    add_repeating_timer_us(-25, 
        repeating_timer_callback_core_0, NULL, &timer_core_0);


    // Add core 0 threads
    pt_add_thread(protothread_core_0) ;
    pt_add_thread(protothread_fft) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}