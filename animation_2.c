
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// uS per frame
#define FRAME_RATE 1000000/30 //33000
#define PI 3.1415927

// global variables

//define parameters for void algorithm 
fix15 turnfactor = float2fix15(0.2);
fix15 visualRange = float2fix15(40);
fix15 protectedRange = float2fix15(8);
fix15 centeringfactor = float2fix15(0.0005);
fix15 avoidfactor = float2fix15(0.05);
fix15 matchingfactor = float2fix15(0.05);
fix15 maxspeed = float2fix15(6);
fix15 minspeed = float2fix15(3);
fix15 maxbias = float2fix15(0.01);
fix15 bias_increment = float2fix15(0.00004);
fix15 biasval = float2fix15(0.001);

// boid instance 
typedef struct Boid
{
   fix15 pos_x 
   fix15 pos_y;
   fix15 vx; 
   fix15 vy ;
} boid_list[10];

void boid_initialize(Boid b) {
    b.pos_x = 0;
    b.pos_y = 0;
    b.vx = 0;
    b.vy = 0;
}

// the color of the boid
char color = WHITE ;

// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  srand((unsigned) time_us_32());
  // Start in frame with random location
  *x = int2fix15(rand() % (540-100) + 100);
  *y = int2fix15(rand() % (380-100) + 100);

  // random total speed
  float random_speed = (float)rand()/(float)(RAND_MAX) * (6-3) + 3;
  float random_angle = (float)rand()/(float)(RAND_MAX) * (2*PI);

  *vx = float2fix15(random_speed *  (float) cos ((double) random_angle));
  *vy = float2fix15(random_speed *  (float) sin ((double) random_angle));

}

// Draw the boundaries
void drawArena() {
  drawVLine(100, 100, 280, WHITE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}

// Detect wallstrikes, update velocity and position
void boid_update(int cur)
{
    fix15* x = boid_list[cur].pos_x;
    fix15* y = boid_list[cur].pos_y;
    fix15* vx = boid_list[cur].vx;
    fix15* vy = boid_list[cur].vy;
    
    fix15 xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0,  close_dx = 0, close_dy = 0
    int neighboring_boids = 0;
    for (int i = 0; i < 10; i++) {
        if(i == cur) {
            continue;
        }
    fix15* x_o = boid_list[i].pos_x;
    fix15* y_o = boid_list[i].pos_y;
    fix15* vx_o = boid_list[i].vx;
    fix15* vy_o = boid_list[i].vy;
    // Compute differences in x and y coordinates
    fix15 dx = x - x_o;
    fix15 dy = y - y_o;

        // Are both those differences less than the visual range?
        if (abs(dx) < visualRange && abs(dy) < visualRange) {
            // If so, calculate the squared distance
            fix15 squared_distance = dx*dx + dy*dy;

            // Is squared distance less than the protected range?
            if (squared_distance < protectedRange * protectedRange) { 

                // If so, calculate difference in x/y-coordinates to nearfield boid
                close_dx += dx;
                close_dy += dy;
            }

            // If not in protected range, is the boid in the visual range?
            else if (squared_distance < visualRange * visualRange) {

                // Add other boid's x/y-coord and x/y vel to accumulator variables
                xpos_avg += x_o;
                ypos_avg += y_o;
                xvel_avg += vx_o;
                yvel_avg += vy_o;

                // Increment number of boids within visual range
                neighboring_boids += 1;
            }
        }    
    }

    // If there were any boids in the visual range . . .            
    if (neighboring_boids > 0) {

        // Divide accumulator variables by number of boids in visual range
        xpos_avg = xpos_avg/neighboring_boids;
        ypos_avg = ypos_avg/neighboring_boids;
        xvel_avg = xvel_avg/neighboring_boids;
        yvel_avg = yvel_avg/neighboring_boids;

        // Add the centering/matching contributions to velocity
        vx = (vx + (xpos_avg - x) * centeringfactor +  (xvel_avg - vx) * matchingfactor);

        vy = (vy + (ypos_avg - y) * centeringfactor + (yvel_avg - vy) * matchingfactor);
    } 

    // Add the avoidance contribution to velocity
    vx = vx + (close_dx * avoidfactor);
    vy = vy + (close_dy * avoidfactor);
  

    // If the boid is near an edge, make it turn by turnfactor
    // (this describes a box, will vary based on boundary conditions)
    if (hitTop(y)) {
        vy = vy + turnfactor ;
    }
    if (hitBottom(y)) {
        vy = vy - turnfactor ;
    } 
    if (hitRight(x)) {
        vx = vx - turnfactor;
    }
    if (hitLeft(x)) {
        vx = vx + turnfactor;
    } 

    // If the boid has a bias, bias it!
    // biased to right of screen
    if (cur >= 0 && cur < 5) {
        vx = (1 - biasval) * vx + (biasval * 1);
    }
        
    // biased to left of screen
    else if (cur >= 5 && cur < 10) {
        vx = (1 - biasval) * vx + (biasval * (-1));
    }
       

    // Calculate the boid's speed
    // Slow step! Lookup the "alpha max plus beta min" algorithm
    fix15 speed = sqrt(vx * vx + vy * vy);

    // Enforce min and max speeds
    if (speed < minspeed) {
        vx = (vx/speed) * minspeed;
        vy = (vy/speed) * minspeed;
    }

    if (speed > maxspeed) {
        vx = (vx/speed) * maxspeed;
        vy = (vy/speed) * maxspeed;
    }


    // Update position using velocity
    x = x + vx ;
    y = y + vy ;
    
}
 

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-7: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // update boid color
        if ((user_input > 0) && (user_input < 8)) {
          color = (char)user_input ;
        }
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    for(int i = 0; i < 10; i++) {
        boid_initialize(&boid_list[i].pos_x, &boid_list[i].pos_y, &boid_list[i].vx, &boid_list[i].vy);
        spawnBoid(&boid_list[i].pos_x, &boid_list[i].pos_y, &boid_list[i].vx, &boid_list[i].vy);
    }

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
    
     for(int i = 0; i < 10; i++) {
        // erase boid
        drawRect(fix2int15(boid_list[i].pos_x), fix2int15(boid_list[i].pos_y), 2, 2, BLACK);
        // update boid's position and velocity
        boid_update(i) ;
        }

      // draw the boid at its new position
        for(int i = 0; i < 10; i++) {
            drawRect(fix2int15(boid_list[i].pos_x), fix2int15(boid_list[i].pos_y), 2, 2, color); 
        }
    
      // draw the boundaries
      drawArena() ;
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;


      printf("number of boids: 1 \n");
      printf("spare time: %d us \n ", spare_time);
      printf("time elapsd: %d s\n", time_us_32()/1000000);

      // Display on VGA
      fillRect(250, 20, 176, 30, BLACK); // red box
      writeString("number of boids: 1 \n");
      setCursor(250, 20) ;
      setTextSize(2) ;

      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

  PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 