/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries
#include "vga_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1.h"


// PWM wrap value and clock divide value
// For a CPU rate of 125 MHz, this gives
// a PWM frequency of 1 kHz.
#define WRAPVAL 5000
#define CLKDIV 25.0f
#define max_duty_cycle 5000;

// Variable to hold PWM slice number
uint slice_num  = 0;

// PWM duty cycle
volatile int control ;
volatile int old_control ;


// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];
fix15 complementary_angle = int2fix15(0);
fix15 accel_angle = int2fix15(0);
fix15 gyro_angle_delta = int2fix15(0);
volatile int pwm_on_time = 0;
volatile int motor_disp = 0;
volatile int Para_P = 200;
volatile int Para_I ;
volatile int Para_D ;
fix15 error_accumulation = 0;
fix15 Imax = 1500;
fix15 desired_angle = 0;
fix15 angle_increment = 0.0001;
fix15 prev_error = 0;

// character array
char screentext[40];

// draw speed
int threshold = 10 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// PWM interrupt service routine
void on_pwm_wrap() {


    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    
    // Compute the error
    // error = (desired_angle - complementary_angle) ;
    fix15 error = complementary_angle - desired_angle;

    // Start with angle_increment = 0.0001
    if (error < int2fix15(0)) {
        desired_angle -= angle_increment ;
    }
    else {
        desired_angle += angle_increment ;
    }

    // Integrate the error
    error_accumulation += error ;
    // Clamp the integrated error (start with Imax = max_duty_cycle/2)
    if (error_accumulation > Imax) {
        error_accumulation = Imax ;
    }
    if (error_accumulation < multfix15(-1,Imax)) {
        error_accumulation = multfix15(-1,Imax) ;
    } 

    // Approximate the rate of change of the error
    fix15 error_deriv = (error - prev_error) ;

    // Update duty cycle
    control = fix2int15((multfix15(int2fix15(Para_P) , error)) + multfix15(int2fix15(Para_I) , error_accumulation) + multfix15(int2fix15(Para_D) , error_deriv)) ;
    prev_error = error ;
    
    if (control!=old_control) {
        old_control = control ;
        if (control >= 0) {
            pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
            pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
            pwm_on_time = control;
        } else {
            pwm_set_chan_level(slice_num, PWM_CHAN_B, (-1)*control);
            pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
            pwm_on_time = control;
        }
    }

    // Gather measurements
    mpu6050_read_raw(acceleration, gyro);

    // Accelerometer angle (degrees - 15.16 fixed point)
    accel_angle = multfix15(divfix(acceleration[0], acceleration[1]), float2fix15(180.0/3.1415926)) ;

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    gyro_angle_delta = multfix15(gyro[2], float2fix15(0.001)) ;

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, float2fix15(0.999)) + multfix15(accel_angle, float2fix15(0.001));

    motor_disp = motor_disp + ((pwm_on_time - motor_disp)>>6) ;


            

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// User input thread
static PT_THREAD (protothread_serial(struct pt *pt))
{
    static int test_in;
    PT_BEGIN(pt) ;

    while(1) {
        // printf("desired angle is set to %d.\n\r", fix2int15(desired_angle));
        sprintf(pt_serial_out_buffer, "input desired angle \n\r");
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        if (test_in == fix2int15(desired_angle)) {

        } else {
            desired_angle = int2fix15(test_in);
            printf("desired angle is set to %d.\n\r", fix2int15(desired_angle));
        }

        sprintf(pt_serial_out_buffer, "input parameter P (-1 for no change) \n\r");
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        if (test_in == -1) {

        } else {
            Para_P = test_in;
            printf("Parameter P is set to %d.\n\r", Para_P);
        }

        sprintf(pt_serial_out_buffer, "input parameter I (-1 for no change) \n\r");
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        if (test_in == -1) {

        } else {
        Para_I = test_in;
        printf("Parameter I is set to %d.\n\r",Para_I);
        }

        sprintf(pt_serial_out_buffer, "input parameter D (-1 for no change)\n\r");
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        if (test_in == -1) {

        } else {
        Para_D = test_in;
        printf("Parameter D is set to %d.\n\r",Para_D);
        }
    }
    PT_END(pt) ;
}





// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 500. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = -250. ;
    static float OldMax = 250. ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);



    //   setCursor(500,10);
    //   writeString("Kp: ");
    //   setCursor(500,20);
    //   writeString("Ki: ");
    //   setCursor(500,30);
    //   writeString("Kd: ");

    //   static char kp_text[40];
    //   static char ki_text[40];
    //   static char kd_text[40];    

    //   sprintf(kp_text,"%d", (int)Para_P);
    //   setCursor(600,10);
    //   writeString(kp_text);
    //   sprintf( ki_text,"%d",(int)Para_I);
    //   setCursor(600,20);
    //   writeString( ki_text);
    //   sprintf( kd_text,"%d",(int)Para_D);
    //   setCursor(600,30);
    //   writeString( kd_text);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "+25") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "-25") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+5.00") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-5.00") ;
    setCursor(45, 225) ;
    writeString(screentext) ;
    

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            // Erase a column
           drawVLine(xcoord, 0, 480, BLACK) ;


            // Draw bottom plot 
            drawPixel(xcoord, 430 - (225 - 75) / 2 - fix2int15(complementary_angle) * 3, RED) ;

            // Draw top plot
            drawPixel(xcoord, 230 - (225 - 75) / 2 - motor_disp / 63, GREEN) ;
            
            // Draw desired angle
            setCursor(30,255);
            writeString("angle: ");
            fillRect(75,255, 15,15,BLACK);
            static char angle_text[40];
            sprintf(angle_text,"%d",fix2int15(desired_angle));
            setCursor(75,255);
            writeString(angle_text);

            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}


// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
/////////////////////// 1st pwm 

    // Tell GPIO 5 that it is allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 3125);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    //////////////////////// 2nd pwm 

    // Tell GPIO 5 that it is allocated to the PWM
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 6 (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(4);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 3125);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
