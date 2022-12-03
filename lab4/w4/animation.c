
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
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/gpio.h"
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
#define hitBottom(b) (b>int2fix15(350))
#define hitTop(b) (b<int2fix15(130))
#define hitLeft(a) (a<int2fix15(130))
#define hitRight(a) (a>int2fix15(510))

// uS per frame
#define FRAME_RATE 1000000/30 //33000
#define PI 3.1415927

//buttons
#define button_w  10
#define button_a  11
#define button_s  12
#define button_d  13

#define button_w1  2
#define button_a1  3
#define button_s1  4
#define button_d1  5

// global variables
int boid_total_num = 200;
int boid_alive_num = 200;
int boid_safe_num = 0;
int boid_dead_num = 0;

//define parameters for void algorithm 
fix15 turnfactor = float2fix15(0.2);
fix15 visualRange = float2fix15(40);
fix15 protectedRange = float2fix15(8);
fix15 sheepDogRange = float2fix15(35);
fix15 sheepwolfRange = float2fix15(5);
fix15 centeringfactor = float2fix15(0.0005);
fix15 avoidfactor = float2fix15(0.05);
fix15 matchingfactor = float2fix15(0.05);
fix15 maxspeed = float2fix15(3);
fix15 minspeed = float2fix15(0);
fix15 maxdogspeed = float2fix15(10);
fix15 sheep_shift_factor = float2fix15(0.05);



// min and max functions

fix15 max(fix15 num1, fix15 num2) {
  return (num1 > num2 ) ? num1 : num2;
}

fix15 min(fix15 num1, fix15 num2) {
  return (num1 > num2 ) ? num2 : num1;
}


// boid instance 
typedef struct list {
   fix15 pos_x;
   fix15 pos_y;
   fix15 vx; 
   fix15 vy ;
   char color; 
   bool inCage;
   bool alive;
} Boid;

Boid boid_list[1000];

// dog instance 
typedef struct list1 {
   fix15 pos_x;
   fix15 pos_y;
   fix15 vx; 
   fix15 vy ;
   char color; 
} Dog;

Dog shep;

// wolf instance 
typedef struct list2 {
   fix15 pos_x;
   fix15 pos_y;
   fix15 vx; 
   fix15 vy ;
   char color; 
} Wolf;

Wolf wolf;


void boid_initialize(Boid* b) {
    b->pos_x = 0;
    b->pos_y = 0;
    b->vx = 0;
    b->vy = 0;
    b->color = WHITE;
    b->inCage = false;
    b->alive = true;
}

void dog_initialize(Dog* d) {
    d->pos_x = int2fix15(300);
    d->pos_y = int2fix15(240);
    d->vx = 0;
    d->vy = 0;
    d->color = RED;
}

void wolf_initialize(Wolf* w) {
    w->pos_x = int2fix15(330);
    w->pos_y = int2fix15(240);
    w->vx = 0;
    w->vy = 0;
    w->color = GREEN;
}

// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  srand((unsigned) time_us_32());
  // Start in frame with random location
  *x = int2fix15(rand() % (540-100) + 100);
  *y = int2fix15(rand() % (380-100) + 100);

  // random total speed
  float random_speed = (float)rand()/(float)(RAND_MAX) * (3-0) + 0;
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

  drawHLine(300, 380, 40,  BLACK) ;
  drawVLine(300, 380, 10,  WHITE) ;
  drawVLine(340, 380, 10,  WHITE) ;

  drawRect(270, 390, 100,50, WHITE);
  drawHLine(300, 390, 40,  BLACK) ;
}

bool inHome(fix15 x, fix15 y) {
  if(x >= int2fix15(270) && x <= int2fix15(370) && y >= int2fix15(390) && y <= int2fix15(440)) {
    return true;
  } else {
    return false;
  }
}

// check boundaries
// 0 for rectangle, 1 for top/bot, 2 for left/right
void boundary_check(int boid_num) {
  fix15* x = &boid_list[boid_num].pos_x;
  fix15* y = &boid_list[boid_num].pos_y;
  fix15* vx = &boid_list[boid_num].vx;
  fix15* vy = &boid_list[boid_num].vy;
  bool* inRect = &boid_list[boid_num].inCage;
  // If the boid is near an edge, make it turn by turnfactorWW
  if (hitTop(*y)) {
      *vy = *vy + turnfactor ;
  }
  // if (hitBottom(*y)) {
  //     *vy = *vy - turnfactor ;
  // } 
  if (hitRight(*x)) {
      *vx = *vx - turnfactor;
  }
  if (hitLeft(*x)) {
      *vx = *vx + turnfactor;
  } 
  if (inHome(*x, *y)) {
    if(*inRect == false) {
      *inRect = true; 
      boid_safe_num++;
    }
    
  }
}

// Detect wallstrikes, update velocity and position
void boid_update(int cur)
{
    
    fix15* x = &boid_list[cur].pos_x;
    fix15* y = &boid_list[cur].pos_y;
    fix15* vx = &boid_list[cur].vx;
    fix15* vy = &boid_list[cur].vy;
    bool* atHome = &boid_list[cur].inCage;
    bool* isalive = &boid_list[cur].alive;

    fix15* x_wolf = &wolf.pos_x;
    fix15* y_wolf = &wolf.pos_y;
    fix15* vx_wolf = &wolf.vx;
    fix15* vy_wolf = &wolf.vy;
    fix15 close_dx_sw = 0;
    fix15 close_dy_sw = 0;

    fix15 sheep_wolf_dx = *x - *x_wolf;
    fix15 sheep_wolf_dy = *y - *y_wolf;
    fix15 squared_distance_sw = multfix15(sheep_wolf_dx,sheep_wolf_dx) + multfix15(sheep_wolf_dy,sheep_wolf_dy);
    if (squared_distance_sw < multfix15(sheepwolfRange,sheepwolfRange) && sheep_wolf_dx < sheepwolfRange && sheep_wolf_dy < sheepwolfRange && sheep_wolf_dx > multfix15(int2fix15(-1),sheepwolfRange) && sheep_wolf_dy > multfix15(int2fix15(-1),sheepwolfRange)) { 
        if(!(*atHome)){
          *isalive = false;
          drawRect(fix2int15(boid_list[cur].pos_x), fix2int15(boid_list[cur].pos_y), 2, 2, BLACK);
          boid_alive_num--;
          boid_dead_num++;
          return;
        }
    }
    if(!(*isalive)) {
      return;
    }

    fix15* x_dog = &shep.pos_x;
    fix15* y_dog = &shep.pos_y;
    fix15* vx_dog = &shep.vx;
    fix15* vy_dog = &shep.vy;
    fix15 close_dx_sd = 0;
    fix15 close_dy_sd = 0;

    fix15 sheep_dog_dx = *x - *x_dog;
    fix15 sheep_dog_dy = *y - *y_dog;
    fix15 squared_distance_sd = multfix15(sheep_dog_dx,sheep_dog_dx) + multfix15(sheep_dog_dy,sheep_dog_dy);
    if(!(*atHome)) {
      if (squared_distance_sd < multfix15(sheepDogRange,sheepDogRange)&& sheep_dog_dx < sheepDogRange && sheep_dog_dy < sheepDogRange && sheep_dog_dx > multfix15(int2fix15(-1),sheepDogRange) && sheep_dog_dy > multfix15(int2fix15(-1),sheepDogRange)) { 
          close_dx_sd += sheep_dog_dx;
          close_dy_sd += sheep_dog_dy;
          *vx = *vx + multfix15(close_dx_sd,sheep_shift_factor);
          *vy = *vy + multfix15(close_dy_sd,sheep_shift_factor);

      } else {
          fix15 xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0,  close_dx = 0, close_dy = 0;
          fix15 neighboring_boids = 0;
          for (int i = 0; i < boid_total_num; i++) {
            if(i != cur && boid_list[i].alive && !boid_list[i].inCage) {
              fix15* x_o = &boid_list[i].pos_x;
              fix15* y_o = &boid_list[i].pos_y;
              fix15* vx_o = &boid_list[i].vx;
              fix15* vy_o = &boid_list[i].vy;
              bool* atHome_o = &boid_list[i].inCage;
              bool* isalive_o = &boid_list[i].alive;
              // if(!(isalive_o)) {
              //   continue;
              // }
              // Compute differences in x and y coordinates
              fix15 dx = *x - *x_o;
              fix15 dy = *y - *y_o;
              // Are both those differences less than the visual range?
              if (dx < visualRange && dy < visualRange && dx > -visualRange && dy > -visualRange) {
                  // If so, calculate the squared distance
                  fix15 squared_distance = multfix15(dx,dx) + multfix15(dy,dy);

                  // Is squared distance less than the protected range?
                  if (squared_distance < multfix15(protectedRange,protectedRange)) { 

                      // If so, calculate difference in x/y-coordinates to nearfield boid
                      close_dx += dx;
                      close_dy += dy;
                  }

                  // If not in protected range, is the boid in the visual range?
                  else if (squared_distance < multfix15(visualRange,visualRange)) {

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
      }
    } else {
      float home_speed = 1;
      float random_angle = (float)rand()/(float)(RAND_MAX) * (2*PI);
      *vx = float2fix15(home_speed *  (float) cos ((double) random_angle));
      *vy = float2fix15(home_speed *  (float) sin ((double) random_angle));
    }

    boundary_check(cur);

    // // Calculate the boid's speed
    // // Slow step! Lookup the "alpha max plus beta min" algorithm
    fix15 temp_add = multfix15(*vx,*vx) + multfix15(*vy,*vy);
    fix15 speed = sqrtfix( temp_add );

    // Enforce min and max speeds
    if (speed < minspeed && !(*atHome)) {
         *vx = multfix15(divfix(*vx,speed),minspeed);
         *vy = multfix15(divfix(*vy,speed),minspeed);
    }

    if (speed > maxspeed) {
         *vx = multfix15(divfix(*vx,speed),maxspeed);
         *vy = multfix15(divfix(*vy,speed),maxspeed);
    }



    // Update position using velocity
    *x = *x + *vx ;
    *y = *y + *vy ;

    if(*atHome) {
      if(*y <= int2fix15(390)) {
        *y = int2fix15(395);
        *vy = 1;
      }         
      else if(*y >= int2fix15(440)) {
        *y = int2fix15(435);
        *vy = -1;
      }
      else if(*x <= int2fix15(270)) {
        *x = int2fix15(275);
        *vx = 1;
      }
      else if(*y >= int2fix15(390) && *y <= int2fix15(440) && *x >= int2fix15(370)) {
        *x = int2fix15(365);
        *vx = -1;
      }
    }
    else{
      if(*x >= int2fix15(540)) {
      *x = int2fix15(535);
      *vx = -1;
      } 
      if(*x <= int2fix15(100)) {
        *x = int2fix15(105); 
        *vx = 1;
      }
      if(*x >= int2fix15(100) && *x <= int2fix15(300) && *y >= int2fix15(380) && *y <= int2fix15(390)) {
        *y = int2fix15(380);
        *vy = -1;
      }
      if(*x >= int2fix15(340) && *x <= int2fix15(540) && *y >= int2fix15(380) && *y <= int2fix15(390)) {
        *y = int2fix15(380);
        *vy = -1;
      }
      if(*y <= int2fix15(100)) {
        *y = int2fix15(105);
        *vy = 1;
      }
      if(*y >= int2fix15(440)) {
        *y = int2fix15(435);
        *vy = -1;
      }
    }
    
}

void dog_update() {
    fix15* x = &shep.pos_x;
    fix15* y = &shep.pos_y;
    fix15* vx = &shep.vx;
    fix15* vy = &shep.vy;
    
    if(gpio_get(button_w) == 1 && gpio_get(button_s) == 0) {
      *vy = int2fix15(-5);
    }
    else if(gpio_get(button_w) == 0 && gpio_get(button_s) == 1) {
      *vy = int2fix15(5);
    }
    else *vy = int2fix15(0);

    if(gpio_get(button_a) == 1 && gpio_get(button_d) == 0) {
      *vx = int2fix15(-5);
    }
    else if(gpio_get(button_a) == 0 && gpio_get(button_d) == 1) {
      *vx = int2fix15(5);
    }
    else *vx = int2fix15(0);

    *x = *x + *vx ;
    *y = *y + *vy ;

    if(*x >= int2fix15(540)) {
      *x = int2fix15(535);
    } 
    if(*x <= int2fix15(100)) {
      *x = int2fix15(105); 
    }
    if(*y <= int2fix15(100)) {
      *y = int2fix15(105);
    }
    if(*y >= int2fix15(380)) {
      *y = int2fix15(375);
    }
}
 
void wolf_update() {
    fix15* x = &wolf.pos_x;
    fix15* y = &wolf.pos_y;
    fix15* vx = &wolf.vx;
    fix15* vy = &wolf.vy;
    
    if(gpio_get(button_w1) == 1 && gpio_get(button_s1) == 0) {
      *vy = int2fix15(-5);
    }
    else if(gpio_get(button_w1) == 0 && gpio_get(button_s1) == 1) {
      *vy = int2fix15(5);
    }
    else *vy = int2fix15(0);

    if(gpio_get(button_a1) == 1 && gpio_get(button_d1) == 0) {
      *vx = int2fix15(-5);
    }
    else if(gpio_get(button_a1) == 0 && gpio_get(button_d1) == 1) {
      *vx = int2fix15(5);
    }
    else *vx = int2fix15(0);

    *x = *x + *vx ;
    *y = *y + *vy ;

    if(*x >= int2fix15(540)) {
      *x = int2fix15(535);
      } 
    if(*x <= int2fix15(100)) {
      *x = int2fix15(105); 
    }
    if(*y <= int2fix15(100)) {
      *y = int2fix15(105);
    }
    if(*y >= int2fix15(380)) {
      *y = int2fix15(375);
    }
}
// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static char user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input shepherd dog direction ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%c", &user_input) ;
        // if(user_input == 'w')
        // shep.vx = shep.vx - int2fix15(1);
        // else if(user_input == 'a')
        // shep.vy = shep.vy - int2fix15(1);
        // else if(user_input == 's')
        // shep.vx = shep.vx + int2fix15(1);
        // else if(user_input == 'd')
        // shep.vx = shep.vx + int2fix15(1);
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
    dog_initialize(&shep);
    wolf_initialize(&wolf);
    for(int i = 0; i < boid_total_num; i++) {
        boid_initialize(&boid_list[i]);
        spawnBoid(&boid_list[i].pos_x, &boid_list[i].pos_y, &boid_list[i].vx, &boid_list[i].vy);
    }

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      drawRect(fix2int15(shep.pos_x) - 2, fix2int15(shep.pos_y) - 2, 5, 5, BLACK);
      drawRect(fix2int15(wolf.pos_x) - 2, fix2int15(wolf.pos_y) - 2, 5, 5, BLACK);  
      for(int i = 0; i < boid_total_num; i++) {
        if(boid_list[i].alive == false) {
          continue;
        }
        // erase boids
        drawRect(fix2int15(boid_list[i].pos_x), fix2int15(boid_list[i].pos_y), 2, 2, BLACK);
        // update boid's position and velocity
        boid_update(i) ;
        if(boid_list[i].alive) {
          drawRect(fix2int15(boid_list[i].pos_x), fix2int15(boid_list[i].pos_y), 2, 2, boid_list[i].color); 
        }

      }
      dog_update();
      drawRect(fix2int15(shep.pos_x) - 2, fix2int15(shep.pos_y) - 2, 5, 5, RED);
      wolf_update();
      drawRect(fix2int15(wolf.pos_x) - 2, fix2int15(wolf.pos_y) - 2, 5, 5, GREEN);
      // draw the boundaries
      drawArena() ;
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
      writeString("sheep alive: ");
      setCursor(500,10);
      writeString("sheep saved: ");
      setCursor(500,20);
      writeString("sheep dead: ");
      setCursor(500,30);
      writeString("spare time (us): ");
      setCursor(500,40);
      writeString("time elapsed (s):");

      fillRect(600,0,50,50,BLACK);
      static char numtext[40];
      static char livetext[40];
      static char deadtext[40];
      static char stmtext[40];
      static char teltext[40];
      sprintf(numtext,"%d",(int)boid_alive_num);
      setCursor(600,0);
      writeString(numtext);
      sprintf(livetext,"%d",(int)boid_safe_num);
      setCursor(600,10);
      writeString(livetext);
      sprintf(deadtext,"%d",(int)boid_dead_num);
      setCursor(600,20);
      writeString(deadtext);
      sprintf(stmtext,"%d",(int)spare_time);
      setCursor(600,30);
      writeString(stmtext);
      sprintf(teltext,"%d",(int)(time_us_32()/1000000));
      setCursor(600,40);
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
  gpio_init(button_w);
  gpio_set_dir(button_w, GPIO_IN);
  gpio_init(button_a);
  gpio_set_dir(button_a, GPIO_IN);
  gpio_init(button_s);
  gpio_set_dir(button_s, GPIO_IN);
  gpio_init(button_d);
  gpio_set_dir(button_d, GPIO_IN);


  gpio_init(button_w1);
  gpio_set_dir(button_w1, GPIO_IN);
  gpio_init(button_a1);
  gpio_set_dir(button_a1, GPIO_IN);
  gpio_init(button_s1);
  gpio_set_dir(button_s1, GPIO_IN);
  gpio_init(button_d1);
  gpio_set_dir(button_d1, GPIO_IN);


  adc_init();
  adc_gpio_init(26);
  adc_select_input(0);
  uint16_t result = adc_read();
  float voltage = result * (3.3/4095.0);
  printf(" ADC Raw Value = %d Voltage = %f \r\n", result, voltage);
  
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
