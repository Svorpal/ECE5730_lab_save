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

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 40000            // sample rate

// the DDS units - core 1
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_1;                  
volatile unsigned int phase_incr_main_1 = (2300.0*two32)/Fs ;
// the DDS units - core 2
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (2300.0*two32)/Fs ;

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



// This timer ISR is called on core 1
bool repeating_timer_callback_core_1(struct repeating_timer *t) {    
    if(gpio_get(button1) == 1) {
        STATE_1_cycle = 1;
        count_1 = 0;
        count_1_c = 0;
    }    
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
    
    } else if (STATE_1_cycle == 1){
        if(count_1_c == CHIRP_REPEAT_INTERVAL) {
            STATE_1_cycle = 0 ;
            count_1_c = 0 ;
        } else {
            current_amplitude_1 = 0 ;
            count_1_c += 1;
        }   
    } else if (STATE_1_cycle == 2) {
        current_amplitude_1 = 0 ;
        count_1_c = 0;
        count_1 = 0;
    }
  // retrieve core number of execution
    corenum_1 = get_core_num() ;

    return true;
}

// This timer ISR is called on core 0
bool repeating_timer_callback_core_0(struct repeating_timer *t) {    
    if(gpio_get(button0) == 1) {
        STATE_0_cycle = 1;
        count_0 = 0;
        count_0_c = 0;
    }
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

    } else if (STATE_0_cycle == 1){
        if(count_0_c == CHIRP_REPEAT_INTERVAL) {
            STATE_0_cycle = 0 ;
            count_0_c = 0 ;
        } else {
            current_amplitude_0 = 0 ;
            count_0_c += 1;
        }   
    } else if (STATE_0_cycle == 2) {
        current_amplitude_0 = 0 ;
        count_0_c = 0;
        count_0 = 0;
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

    // Start scheduling core 0 threads
    pt_schedule_start ;

}