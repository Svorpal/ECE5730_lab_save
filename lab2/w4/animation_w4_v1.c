
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
#define sqrtfix(a) (float2fix15(sqrt(fix2float15(a))))

// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// uS per frame
#define FRAME_RATE 1000000/30 //33000
#define PI 3.1415927

// global variables
int boid_total_num = 360;
int boid_set1_num = 30;
bool boid_set1_bias_changed = false;
bool boid_set2_bias_changed = false;
int boid_set2_num = 30;
int boid_dynamic_bias_enable = 0;
fix15 boid_set1_bias = float2fix15(0.01);
fix15 boid_set2_bias = float2fix15(0.01);

//define parameters for void algorithm 
fix15 turnfactor = float2fix15(0.2);
fix15 visualRange = float2fix15(40);
fix15 protectedRange = float2fix15(8);
fix15 centeringfactor = float2fix15(0.0005);
fix15 avoidfactor = float2fix15(0.05);
fix15 matchingfactor = float2fix15(0.05);
fix15 maxspeed = float2fix15(6);
fix15 minspeed = float2fix15(3);
fix15 maxbias = float2fix15(0.99);
fix15 bias_increment = float2fix15(0.004);
fix15 biasval = float2fix15(0.01);

// min and max functions

static inline fix15 max(fix15 num1, fix15 num2) {
  return (num1 > num2 ) ? num1 : num2;
}

static inline fix15 min(fix15 num1, fix15 num2) {
  return (num1 > num2 ) ? num2 : num1;
}


// boid instance 
typedef struct list {
   fix15 pos_x;
   fix15 pos_y;
   fix15 vx; 
   fix15 vy ;
   char color; 
   fix15 bias;
} Boid;

Boid boid_list[1000];

static inline void boid_initialize(Boid* b) {
    b->pos_x = 0;
    b->pos_y = 0;
    b->vx = 0;
    b->vy = 0;
    b->bias = biasval;
    b->color = WHITE;
}


// bound type 
fix15 old_bound_type = 0;
fix15 bound_type = 0;

// Create a boid
static inline void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy)
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
static inline void drawArena() {
  if(old_bound_type != bound_type) {
    if(old_bound_type == 0) {
      drawVLine(100, 100, 280, BLACK) ;
      drawVLine(540, 100, 280, BLACK) ;
      drawHLine(100, 100, 440, BLACK) ;
      drawHLine(100, 380, 440, BLACK) ;
    } else if (old_bound_type > 0) {
      fix15 half = old_bound_type >> 1;
      fix15 left = int2fix15(320) - half;
      fix15 right = int2fix15(320) + half;
      drawVLine(fix2int15(left), 0, 480, BLACK) ;
      drawVLine(fix2int15(right), 0, 480,BLACK) ;
    } else if (bound_type < 0) {

    }
  }
  if(bound_type == 0) {
    drawVLine(100, 100, 280, WHITE) ;
    drawVLine(540, 100, 280, WHITE) ;
    drawHLine(100, 100, 440, WHITE) ;
    drawHLine(100, 380, 440, WHITE) ;
  } else if (bound_type > 0) {
    fix15 half = bound_type >> 1;
    fix15 left = int2fix15(320) - half;
    fix15 right = int2fix15(320) + half;
    drawVLine(fix2int15(left), 0, 480, WHITE) ;
    drawVLine(fix2int15(right), 0, 480, WHITE) ;
  } else if (bound_type < 0) {

  }
  old_bound_type = bound_type;
}

// check boundaries
// 0 for rectangle, 1 for top/bot, 2 for left/right
static inline void boundary_check(int boid_num) {
  fix15* x = &boid_list[boid_num].pos_x;
  fix15* y = &boid_list[boid_num].pos_y;
  fix15* vx = &boid_list[boid_num].vx;
  fix15* vy = &boid_list[boid_num].vy;
  
  if(bound_type == 0) {
  // If the boid is near an edge, make it turn by turnfactorWW
  if (hitTop(*y)) {
      *vy = *vy + turnfactor ;
  }
  if (hitBottom(*y)) {
      *vy = *vy - turnfactor ;
  } 
  if (hitRight(*x)) {
      *vx = *vx - turnfactor;
  }
  if (hitLeft(*x)) {
      *vx = *vx + turnfactor;
  } 

  } else if (bound_type > 0) {
    fix15 half = bound_type >> 1;
    fix15 left = int2fix15(320) - half;
    fix15 right = int2fix15(320) + half;
    if (*x > right) {
      *vx = *vx - turnfactor;
    }
    if (*x < left) {
      *vx = *vx + turnfactor;
    } 
  }
}

// Detect wallstrikes, update velocity and position
static inline void boid_update(int cur)
{
    fix15* x = &boid_list[cur].pos_x;
    fix15* y = &boid_list[cur].pos_y;
    fix15* vx = &boid_list[cur].vx;
    fix15* vy = &boid_list[cur].vy;
    fix15* bias = &boid_list[cur].bias;
    
    fix15 xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0,  close_dx = 0, close_dy = 0;
    fix15 neighboring_boids = 0;
    for (int i = 0; i < boid_total_num; i++) {
      if(i != cur) {
      fix15* x_o = &boid_list[i].pos_x;
      fix15* y_o = &boid_list[i].pos_y;
      fix15* vx_o = &boid_list[i].vx;
      fix15* vy_o = &boid_list[i].vy;
      // Compute differences in x and y coordinates
      fix15 dx = *x - *x_o;
      if(dx > int2fix15(320)) {dx = dx - int2fix15(640);}
      if(dx < int2fix15(-320)) {dx = dx + int2fix15(640);}
      fix15 dy = *y - *y_o;
      if(dy > int2fix15(240)) {dy = dy - int2fix15(480);}
      if(dy < int2fix15(-240)) {dy = dy + int2fix15(480);}

      // Are both those differences less than the visual range?
      if (dx < visualRange && dy < visualRange && dx > -visualRange && dy > -visualRange) {

          // Is squared distance less than the protected range? 
          if (dx < protectedRange && dy < protectedRange && dx > -protectedRange && dy > -protectedRange) { 

              // If so, calculate difference in x/y-coordinates to nearfield boid
              close_dx += dx;
              close_dy += dy;
          }

          // If not in protected range, is the boid in the visual range?
          else {

              // Add other boid's x/y-coord and x/y vel to accumulator variables
              xpos_avg += *x_o;
              ypos_avg += *y_o;
              xvel_avg += *vx_o;
              yvel_avg += *vy_o;

              // Increment number of boids within visual range
              neighboring_boids += int2fix15(1);
          }
        }
      }    
    }

    // If there were any boids in the visual range . . .            
    if (neighboring_boids > 0) {

        // Divide accumulator variables by number of boids in visual range
        xpos_avg = divfix(xpos_avg,neighboring_boids);
        ypos_avg = divfix(ypos_avg,neighboring_boids);
        xvel_avg = divfix(xvel_avg,neighboring_boids);
        yvel_avg = divfix(yvel_avg,neighboring_boids);

        // Add the centering/matching contributions to velocity
        *vx = *vx + multfix15((xpos_avg - *x),centeringfactor) +  multfix15((xvel_avg - *vx), matchingfactor);                      

        *vy = *vy + multfix15((ypos_avg - *y),centeringfactor) +  multfix15((yvel_avg - *vy), matchingfactor);
    } 

    // Add the avoidance contribution to velocity
    *vx = *vx + multfix15(close_dx,avoidfactor);
    *vy = *vy + multfix15(close_dy,avoidfactor);
  
    boundary_check(cur);

    // update bias values
    // If the boid has a bias, bias it!
    // biased to right of screen
  
    if (cur >= 0 && cur < boid_set1_num) {
      if (boid_dynamic_bias_enable == 1) {
        if (*vx > 0) { 
          *bias = min(maxbias, *bias + bias_increment);
        }
        else {
          *bias = max(bias_increment, *bias - bias_increment);
        }
      }
        *vx = multfix15((int2fix15(1) - *bias),(*vx)) + *bias;
    }
        
    // biased to left of screen
    else if (cur >= boid_total_num - boid_set2_num && cur <  boid_total_num) {
      if (boid_dynamic_bias_enable == 1) {
        if (*vx < 0){
          *bias = min(maxbias, *bias + bias_increment);
        }
        else{
        	*bias = max(bias_increment, *bias - bias_increment);
        }
      }
        *vx = multfix15((int2fix15(1) - *bias),(*vx)) - *bias;
    } 
       

    // // Calculate the boid's speed
    // // Slow step! Lookup the "alpha max plus beta min" algorithm
    fix15 temp_add = multfix15(*vx,*vx) + multfix15(*vy,*vy);
    fix15 speed = sqrtfix( temp_add );

    // // Enforce min and max speeds
    if (speed < minspeed) {
         *vx = multfix15(divfix(*vx,speed),minspeed);
         *vy = multfix15(divfix(*vy,speed),minspeed);
    }

    if (speed > maxspeed) {
         *vx = multfix15(divfix(*vx,speed),maxspeed);
         *vy = multfix15(divfix(*vy,speed),maxspeed);
    }


    // Update position using velocity
    *x = *x + *vx ;
    if(*x > int2fix15(640)) {*x = *x - int2fix15(640);}
    if(*x < int2fix15(0)) {*x = *x + int2fix15(640);}
    *y = *y + *vy ;
    if(*y > int2fix15(480)) {*y = *y - int2fix15(480);}
    if(*y < int2fix15(0)) {*y = *y + int2fix15(480);}
    
}
 

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input_int;
    static float user_input_float;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input frame type and width: (0 for rect, pos for top/bot, neg for all))");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input_int) ;

        if ((user_input_int > -1000) && (user_input_int < 1000)) {
         bound_type =  int2fix15(user_input_int) ;
        }


    // print prompt
        sprintf(pt_serial_out_buffer, "input set1 boids: (-1 for no change)");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input_int) ;
        if (user_input_int >= 0 && user_input_int < boid_total_num - boid_set2_num) {
          boid_set1_num =  user_input_int;
        } else if (user_input_int == -1) {

        } else {
          sprintf(pt_serial_out_buffer, "invalid input or exceeding maximum \n\r");
          serial_write ;
        }

        // print prompt
        sprintf(pt_serial_out_buffer, "input set2 boids: (-1 for no change)");
        
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input_int) ;
        if (user_input_int >= 0 && user_input_int < boid_total_num - boid_set1_num) {
          boid_set2_num =  user_input_int;
        } else if (user_input_int == -1) {

        } else {
          sprintf(pt_serial_out_buffer, "invalid input or exceeding maximum \n\r");
          serial_write ;
        }

        // print prompt
        sprintf(pt_serial_out_buffer, "input new bias strength for set1: (-1 for no change, 0 <= x <= 0.99)");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%f", &user_input_float) ;
        if (user_input_float > 0 && user_input_float <= 10000) {
          boid_set1_bias = float2fix15(user_input_float);
          boid_set1_bias_changed = true;
        } else if (user_input_float == -1) {
    
        } else {
          sprintf(pt_serial_out_buffer, "invalid input or exceeding maximum \n\r");
          serial_write ;
        }


        // print prompt
        sprintf(pt_serial_out_buffer, "input new bias strength for set2: (-1 for no change, 0 <= x <= 0.99)");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%f", &user_input_float) ;
        if (user_input_float > 0 && user_input_float <= 10000) {
          boid_set2_bias = float2fix15(user_input_float);
          boid_set2_bias_changed = true;
        } else if (user_input_float == -1) {

        } else {
          sprintf(pt_serial_out_buffer, "invalid input or exceeding maximum \n\r");
          serial_write ;
        }


        // print prompt
        sprintf(pt_serial_out_buffer, "Enable Dynamic Bias? (1 for yes, 0 for no, -1 for no change)");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input_int) ;
        if (user_input_int == 0) {
          boid_dynamic_bias_enable =  0;
        } else if (user_input_int == 1) {
          boid_dynamic_bias_enable =  1;
        } else if (user_input_int == -1) {

        } else {
          sprintf(pt_serial_out_buffer, "invalid input or exceeding maximum \n\r");
          serial_write ;
        }
      }

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

    for(int i = 0; i < boid_total_num; i++) {
        boid_initialize(&boid_list[i]);
        if (i >= 0 && i < boid_set1_num) {
          boid_list[i].color = WHITE;
          boid_list[i].bias = boid_set1_bias;
        }
        else if (i >= boid_total_num - boid_set2_num && i < boid_total_num) {
          boid_list[i].color = RED;
          boid_list[i].bias = boid_set2_bias;
        } else {
          boid_list[i].color = GREEN;
          boid_list[i].bias = biasval;
        }
        spawnBoid(&boid_list[i].pos_x, &boid_list[i].pos_y, &boid_list[i].vx, &boid_list[i].vy);
    }

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ; 
  
    
     for(int i = 0; i < boid_total_num; i++) {
        // erase boid
        drawRect(fix2int15(boid_list[i].pos_x), fix2int15(boid_list[i].pos_y), 2, 2, BLACK);
        // update boid's position and velocity
        boid_update(i) ;
        if (i >= 0 && i < boid_set1_num) {
          boid_list[i].color = WHITE;
          boid_list[i].bias = boid_set1_bias;
          boid_set1_bias_changed = false;
        }
        else if (i >= boid_total_num - boid_set2_num && i < boid_total_num) {
          boid_list[i].color = RED;
          boid_list[i].bias = boid_set2_bias;
          boid_set2_bias_changed = false;
        } else {
          boid_list[i].color = GREEN;
        }
        drawRect(fix2int15(boid_list[i].pos_x), fix2int15(boid_list[i].pos_y), 2, 2, boid_list[i].color); 
        }

      // draw the boundaries
      drawArena(bound_type) ;
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;

      setTextColor(WHITE);
      setCursor(32,0);
      setTextSize(1);
      writeString("ECE 5730 Students:");
      setCursor(32,10);
      writeString("Michael Wu (yw2464)");
      setCursor(32,20);
      writeString("Yuqian Cao (yc2443)");
      setCursor(500,0);
      writeString("number of boids: ");
      setCursor(500,10);
      writeString("spare time (us): ");
      setCursor(500,20);
      writeString("time elapsed (s):");
      fillRect(600,0,50,30,BLACK);
      static char numtext[40];
      static char stmtext[40];
      static char teltext[40];
      sprintf(numtext,"%d",(int)boid_total_num);
      setCursor(600,0);
      writeString(numtext);
      sprintf(stmtext,"%d",(int)spare_time);
      setCursor(600,10);
      writeString(stmtext);
      sprintf(teltext,"%d",(int)(time_us_32()/1000000));
      setCursor(600,20);
      writeString(teltext);


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
  const uint32_t sys_clock = 250000;
  set_sys_clock_khz(sys_clock, true);

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