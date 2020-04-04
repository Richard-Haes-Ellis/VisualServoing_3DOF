#include <ArduinoJson.h>

#define SPR 1600
#define T1_FREQ 656250

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA (2 * 3.14159 / SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA * T1_FREQ * 100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)(T1_FREQ * 0.676))         // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA * 2 * 10000000000)         // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA * 20000)                // ALPHA*20000


#define PIN_X_DIR 55    // PORTA Bit 24
#define PIN_Y_ENABLE 56 // PORTA Bit 23
#define PIN_X_STEP 54   // PORTA Bit 16
#define PIN_Z_MIN 18    // PORTA Bit 11
#define PIN_Z_MAX 19    // PORTA Bit 10
#define PIN_Y_STEP 60   // PORTA Bit 3
#define PIN_Y_DIR 61    // PORTA Bit 2

#define PIN_X_MAX 2     // PORTB Bit 25
#define PIN_Z_ENABLE 62 // PORTB Bit 17

#define PIN_X_MIN 3     // PORTC Bit 28
#define PIN_FAN 9       // PORTC Bit 21
#define PIN_Z_STEP 46   // PORTC Bit 17
#define PIN_Z_DIR 48    // PORTC Bit 15
#define PIN_X_ENABLE 38 // PORTC Bit 6

#define PIN_Y_MAX 15 // PORTD Bit 5
#define PIN_Y_MIN 14 // PORTD Bit 4

// Speed ramp states
#define STOP 0
#define ACCEL 1
#define DECEL 2
#define RUN 3

// Direction of stepper axis1 movement
#define CW -1
#define CCW 1

typedef struct
{
       // Axis specs
       int goal_pos;
       int max_vel;
       int max_acc;
       float goal_exec_time;

       // Hardware declarations
       uint8_t enable_pin;
       uint8_t step_pin;
       uint8_t dir_pin;

       // Motor status at any given time
       volatile uint8_t dir;       //! Direction stepper axis1 should move.
       volatile uint32_t step_position;
       volatile uint8_t running;
       volatile uint32_t speed;
       
       // Interrupt variables
       volatile uint32_t step_count;
       volatile uint32_t step_delay;     //! Peroid of next timer delay. At start this value set the accelration rate c0.
       volatile float step_delay_f;
       volatile uint32_t min_delay;      //! Minimum time delay (max speed)
       volatile uint32_t n;                //! Counter used when accelerateing/decelerateing to calculate step_delay.
       volatile uint32_t rampUpStepCount;
       volatile uint32_t total_steps;
       volatile  int32_t rest;

       // Interrupt handler
       Tc *tc;
       uint32_t channel;
       IRQn_Type irq;
       
} speedRampData;

speedRampData axes[3];

StaticJsonDocument<75> doc;

char data[100];

int timer1=0,prv_timer0,elapsed=0;
int timer2=0;

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq)
{
       NVIC_ClearPendingIRQ(irq);
       NVIC_EnableIRQ(irq);
       TC_Start(tc, channel);
}

void stopTimer(Tc *tc, uint32_t channel, IRQn_Type irq)
{
       NVIC_DisableIRQ(irq);
       TC_Stop(tc, channel);
}

void restartCounter(Tc *tc, uint32_t channel)
{
       // To reset a conter we se the TC_CCR_SWTRG (Software trigger) bit in the TC_CCR
       tc->TC_CHANNEL[channel].TC_CCR |= TC_CCR_SWTRG;
}

void configureTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
       // Unblock thee power managent cotroller
       pmc_set_writeprotect(false);
       pmc_enable_periph_clk((uint32_t)irq);
       // Configure
       TC_Configure(tc,           // Timer
                    channel,      // Channel
                    TC_CMR_WAVE | // Wave form is enabled
                        TC_CMR_WAVSEL_UP_RC |
                        TC_CMR_TCCLKS_TIMER_CLOCK4 // Settings
       );

       uint32_t rc = VARIANT_MCK / 128 / frequency; //128 because we selected TIMER_CLOCK4 above

       tc->TC_CHANNEL[channel].TC_RC = rc; //TC_SetRC(tc, channel, rc);
       // TC_Start(tc, channel);

       // enable timer interrupts on the timer
       tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;  // IER = interrupt enable register // Enables the RC compare register.
       tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS; // IDR = interrupt disable register /// Disables the RC compare register.

       // To reset a conter we se the TC_CCR_SWTRG (Software trigger) bit in the TC_CCR
       tc->TC_CHANNEL[channel].TC_CCR |= TC_CCR_SWTRG;

       /* Enable the interrupt in the nested vector interrupt controller */
       // NVIC_EnableIRQ(irq);
}

int planMovement(speedRampData &axis)
{
       int code = 0;

       // Direction stuff
       axis.dir = axis.goal_pos > 0 ? CCW : CW;                // Save the direction state 
       digitalWrite(axis.dir_pin,axis.goal_pos>0?HIGH:LOW);    // Set the direction depending on the steps

       // Acceleraton stuff
       axis.total_steps = abs(axis.goal_pos);                  // Change the sign of the steps to positive
       if(axis.max_acc == 0){                                  // If we get zero acceleration get out.
              return -1;
       }
       axis.step_delay_f = (float)T1_FREQ_148*sqrt((float)2/axis.max_acc);        // We save the acceleration to the first step delay. (NEEDS CHANGING)
       axis.step_delay = axis.step_delay_f;

       // Velocity stuff
       float minTime = 2*axis.max_vel/axis.max_acc; // Worst case scenario we only accelerate and decelerate. 
       if(axis.max_vel == 0){                                  // If we input zero speed get out.
              return -1;
       }
       if(axis.goal_exec_time<=minTime) // We dont have enough time to do that movement, we will return the actual time we will take.
       {
              axis.goal_exec_time = minTime;
              axis.speed = axis.max_vel;
              // CALCULATE ESTIMATED TIME

              code = 2;
       }
       else // If we have time to do it then we can recalculate the maximum speed
       {
              float _4ac = (4.0*(float)axis.total_steps)/(float)axis.max_acc;
              float _bsq = axis.goal_exec_time*axis.goal_exec_time;
              if(_4ac > _bsq){Serial.println("Failed due to complex root.");return -1;}
              float speed  = (axis.goal_exec_time - sqrt(_bsq-_4ac))*(float)axis.max_acc*0.5;
              axis.speed = speed;
              code = 1;
              
       }

       // CALCULATE SPEED MIN_DELAY
       axis.min_delay = T1_FREQ / axis.speed;               // Set the minumum delay needed for the speed given

       // Inner calcualtion variables
       axis.step_count = 0;                            // Initialize the step count to zero 
       axis.n = 0;                                     // Set the ramp counter to zero.
       axis.rampUpStepCount = 0;                       // Set the ramp counter to zero. Is it not the same as n??
       axis.running = true;                            // Set the axis status to running

       // Register stuff
       TC1->TC_CHANNEL[0].TC_RC = axis.step_delay;     // Set counter ragister to the starting delay (acceleration)

       // Enable motor
       digitalWrite(axis.enable_pin,LOW);                  // Enable stepper axis

       // Start 
       // startTimer(axis.tc,axis.channel,axis.irq);                                    // Start the timer

       /*
       Serial.println("Min delay:"    +String(axis.min_delay));
       Serial.println("Start delay:"  +String(axis.step_delay_f));
       Serial.println("Total steps:"  +String(axis.total_steps));

       Serial.println("Desired position:" +String(axis.goal_pos));
       Serial.println("Acceleration:"     +String(axis.max_acc));
       Serial.println("Max velocity:"     +String(axis.max_vel));
       */
       return code; // Returns eather a 1 or 2 if time constraint is plauseble or not respectively
       // If 2 is returned global planner must adjust all other times to coordinate properly
}

void TC3_Handler()
{
       TC_GetStatus(axes[0].tc, axes[0].channel);               // Timer 1 channel 0 ----> TC3 it also clear the flag
       if (axes[0].step_count <= axes[0].total_steps)
       {
              digitalWrite(axes[0].step_pin, HIGH);
              digitalWrite(axes[0].step_pin, LOW);
              axes[0].step_count++;
              axes[0].step_position += axes[0].dir;
       }

       if(axes[0].step_count > axes[0].total_steps) // If we step more that the total it means we have already finished
       {
              axes[0].running = false;
              digitalWrite(axes[0].enable_pin,HIGH);
              stopTimer(axes[0].tc,axes[0].channel,axes[0].irq);
       }

       if (axes[0].rampUpStepCount == 0) // If we are ramping up 
       { 
              // Calculate next delays 
              axes[0].n++;    
              axes[0].step_delay_f = axes[0].step_delay_f - (2 * axes[0].step_delay_f /*+ axes[0].rest */) / (4 * axes[0].n + 1);

              // If we reach max speed by checkig if the calculated delay is smaller that the one we calculated with the max speed
              if (axes[0].step_delay_f <=  (float)axes[0].min_delay)
              { 
                     axes[0].step_delay_f = axes[0].min_delay;       // We saturate it to that minimum delay 
                     axes[0].rampUpStepCount = axes[0].step_count; // We save the number of steps it took to reach this speed
              }
              // If istead we manage to reach half way without reaching full speed we have to start decelerating 
              if (axes[0].step_count >= (axes[0].total_steps / 2))
              { 
                     axes[0].rampUpStepCount = axes[0].step_count; // So we save the number of steps it took to accelerate to this speed 
              }
       }
       // If we dont have to ramp down yet (we shoudl be in eather run or acelerating)
       else if (axes[0].step_count >= axes[0].total_steps - axes[0].rampUpStepCount)
       { 
              if(axes[0].n!=0)
              {
                     axes[0].step_delay_f = ( axes[0].step_delay_f * (4 * axes[0].n + 1)) / (4 * axes[0].n + 1 - 2); // THis is the same as the other equation but inverted
                     
              }
              axes[0].n--;
       }
       axes[0].step_delay = axes[0].step_delay_f;
       axes[0].tc->TC_CHANNEL[axes[0].channel].TC_RC = axes[0].step_delay;
}

void TC4_Handler()
{
       TC_GetStatus(axes[1].tc, axes[1].channel);               // Timer 1 channel 0 ----> TC3 it also clear the flag
       if (axes[1].step_count <= axes[1].total_steps)
       {
              digitalWrite(axes[1].step_pin, HIGH);
              digitalWrite(axes[1].step_pin, LOW);
              axes[1].step_count++;
              axes[1].step_position += axes[1].dir;
       }

       if(axes[1].step_count > axes[1].total_steps) // If we step more that the total it means we have already finished
       {
              axes[1].running = false;
              digitalWrite(axes[1].enable_pin,HIGH);
              stopTimer(axes[1].tc,axes[1].channel,axes[1].irq);
       }

       if (axes[1].rampUpStepCount == 0) // If we are ramping up 
       { 
              // Calculate next delays 
              axes[1].n++;    
              axes[1].step_delay_f = axes[1].step_delay_f - (2 * axes[1].step_delay_f /*+ axes[1].rest */) / (4 * axes[1].n + 1);

              // If we reach max speed by checkig if the calculated delay is smaller that the one we calculated with the max speed
              if (axes[1].step_delay_f <=  (float)axes[1].min_delay)
              { 
                     axes[1].step_delay_f = axes[1].min_delay;       // We saturate it to that minimum delay 
                     axes[1].rampUpStepCount = axes[1].step_count; // We save the number of steps it took to reach this speed
              }
              // If istead we manage to reach half way without reaching full speed we have to start decelerating 
              if (axes[1].step_count >= (axes[1].total_steps / 2))
              { 
                     axes[1].rampUpStepCount = axes[1].step_count; // So we save the number of steps it took to accelerate to this speed 
              }
       }
       // If we dont have to ramp down yet (we shoudl be in eather run or acelerating)
       else if (axes[1].step_count >= axes[1].total_steps - axes[1].rampUpStepCount)
       { 
              if(axes[1].n!=0)
              {
                     axes[1].step_delay_f = ( axes[1].step_delay_f * (4 * axes[1].n + 1)) / (4 * axes[1].n + 1 - 2); // THis is the same as the other equation but inverted
                     
              }
              axes[1].n--;
       }
       axes[1].step_delay = axes[1].step_delay_f;
       axes[1].tc->TC_CHANNEL[axes[1].channel].TC_RC = axes[1].step_delay;
}

void TC5_Handler()
{
       TC_GetStatus(axes[2].tc, axes[2].channel);               // Timer 1 channel 0 ----> TC3 it also clear the flag
       if (axes[2].step_count <= axes[2].total_steps)
       {
              digitalWrite(axes[2].step_pin, HIGH);
              digitalWrite(axes[2].step_pin, LOW);
              axes[2].step_count++;
              axes[2].step_position += axes[2].dir;
       }

       if(axes[2].step_count > axes[2].total_steps) // If we step more that the total it means we have already finished
       {
              axes[2].running = false;
              digitalWrite(axes[2].enable_pin,HIGH);
              stopTimer(axes[2].tc,axes[2].channel,axes[2].irq);
       }

       if (axes[2].rampUpStepCount == 0) // If we are ramping up 
       { 
              // Calculate next delays 
              axes[2].n++;    
              axes[2].step_delay_f = axes[2].step_delay_f - (2 * axes[2].step_delay_f /*+ axes[2].rest */) / (4 * axes[2].n + 1);

              // If we reach max speed by checkig if the calculated delay is smaller that the one we calculated with the max speed
              if (axes[2].step_delay_f <=  (float)axes[2].min_delay)
              { 
                     axes[2].step_delay_f = axes[2].min_delay;       // We saturate it to that minimum delay 
                     axes[2].rampUpStepCount = axes[2].step_count; // We save the number of steps it took to reach this speed
              }
              // If istead we manage to reach half way without reaching full speed we have to start decelerating 
              if (axes[2].step_count >= (axes[2].total_steps / 2))
              { 
                     axes[2].rampUpStepCount = axes[2].step_count; // So we save the number of steps it took to accelerate to this speed 
              }
       }
       // If we dont have to ramp down yet (we shoudl be in eather run or acelerating)
       else if (axes[2].step_count >= axes[2].total_steps - axes[2].rampUpStepCount)
       { 
              if(axes[2].n!=0)
              {
                     axes[2].step_delay_f = ( axes[2].step_delay_f * (4 * axes[2].n + 1)) / (4 * axes[2].n + 1 - 2); // THis is the same as the other equation but inverted
                     
              }
              axes[2].n--;
       }
       axes[2].step_delay = axes[2].step_delay_f;
       axes[2].tc->TC_CHANNEL[axes[2].channel].TC_RC = axes[2].step_delay;
}


void setup()
{
       Serial.begin(115200);

       pinMode(PIN_Z_ENABLE, OUTPUT);
       pinMode(PIN_Z_DIR, OUTPUT);
       pinMode(PIN_Z_STEP, OUTPUT);

       pinMode(PIN_Y_ENABLE, OUTPUT);
       pinMode(PIN_Y_DIR, OUTPUT);
       pinMode(PIN_Y_STEP, OUTPUT);

       pinMode(PIN_X_ENABLE, OUTPUT);
       pinMode(PIN_X_DIR, OUTPUT);
       pinMode(PIN_X_STEP, OUTPUT);

       delay(3000);


       Serial.println("Starting timer..");
       configureTimer(/*Timer TC1*/ TC1, /*Channel 0*/ 0, /*TC3 interrupt nested vector controller*/ TC3_IRQn, /*Frequency in hz*/ T1_FREQ);
       configureTimer(/*Timer TC1*/ TC1, /*Channel 0*/ 1, /*TC3 interrupt nested vector controller*/ TC4_IRQn, /*Frequency in hz*/ T1_FREQ);
       configureTimer(/*Timer TC1*/ TC1, /*Channel 0*/ 2, /*TC3 interrupt nested vector controller*/ TC5_IRQn, /*Frequency in hz*/ T1_FREQ);

       // Axis 1
       axes[0].enable_pin     = PIN_Z_ENABLE;
       axes[0].step_pin       = PIN_Z_STEP;
       axes[0].dir_pin        = PIN_Z_DIR;

       axes[0].goal_pos = 80000;
       axes[0].max_acc = 70000; // Up to 70k is fine 
       axes[0].max_vel = 30000; // 30k is the max

       axes[0].tc = TC1;
       axes[0].channel = 0;
       axes[0].irq = TC3_IRQn;

       // Axis 2
       axes[1].enable_pin     = PIN_Y_ENABLE;
       axes[1].step_pin       = PIN_Y_STEP;
       axes[1].dir_pin        = PIN_Y_DIR;

       axes[1].goal_pos = 10000;
       axes[1].max_acc = 70000; // MAX FOR THIS AXIS
       axes[1].max_vel = 30000; // MAX for this axes

       axes[1].tc = TC1;
       axes[1].channel = 1;
       axes[1].irq = TC4_IRQn;

       // Axis 3
       axes[2].enable_pin     = PIN_X_ENABLE;
       axes[2].step_pin       = PIN_X_STEP;
       axes[2].dir_pin        = PIN_X_DIR;

       axes[2].goal_pos = 5000;
       axes[2].max_acc = 70000; // MAX FOR THIS AXIS
       axes[2].max_vel = 30000; // MAX for this axes

       axes[2].tc = TC1;
       axes[2].channel = 2;
       axes[2].irq = TC5_IRQn;
}

void startMovement(speedRampData axis[],uint numAxis)
{
       for(uint8_t i=0; i< numAxis;i++){
              startTimer(axis[i].tc,axis[i].channel,axis[i].irq); 
       }
}

void loop()
{
       axes[0].goal_exec_time = 3;
       axes[1].goal_exec_time = 3;
       axes[2].goal_exec_time = 3;
      
       planMovement(axes[0]);
       planMovement(axes[1]);
       planMovement(axes[2]);
       startMovement(axes,3);
       while(axes[0].running);

       axes[0].goal_pos=-axes[0].goal_pos;
       axes[0].goal_exec_time = 3;

       axes[1].goal_pos=-axes[1].goal_pos;
       axes[1].goal_exec_time = 3;

       axes[2].goal_pos=-axes[2].goal_pos;
       axes[2].goal_exec_time = 3;
       
       planMovement(axes[0]);
       planMovement(axes[1]);
       planMovement(axes[2]);
       startMovement(axes,3);
       while(axes[0].running);

       
       // speed_cntr_Move(axis2);

       // do
       // {
       //        Serial.println(" n:" + String(axis1.n) +
       //                       " step:" + String(axis1.step_count) +
       //                       " running:" + String(axis1.running) +
       //                       " step_pos:" + String(axis1.step_position) +
       //                       " elapsed:" + String(elapsed) +
       //                       " step_delay:" + String(axis1.step_delay));
       // } while (axis1.running == true);
       
       while(true);
}

/* ARCHIVE

#define PIN_X_DIR      (0x1u << 24) // PORTA Bit 24 
#define PIN_Y_ENABLE   (0x1u << 23) // PORTA Bit 23
#define PIN_X_STEP     (0x1u << 16) // PORTA Bit 16
#define PIN_Z_MIN      (0x1u << 11) // PORTA Bit 11
#define PIN_Z_MAX      (0x1u << 10) // PORTA Bit 10
#define PIN_Y_STEP     (0x1u << 3)  // PORTA Bit 3
#define PIN_Y_DIR      (0x1u << 2)  // PORTA Bit 2


#define PIN_X_MAX      (0x1u << 25) // PORTB Bit 25
#define PIN_Z_ENABLE   (0x1u << 17) // PORTB Bit 17

#define PIN_X_MIN      (0x1u << 28) // PORTC Bit 28
#define PIN_FAN        (0x1u << 21) // PORTC Bit 21
#define PIN_Z_STEP     (0x1u << 17) // PORTC Bit 17
#define PIN_Z_DIR      (0x1u << 15) // PORTC Bit 15
#define PIN_X_ENABLE   (0x1u << 6)  // PORTC Bit 6

#define PIN_Y_MAX      (0x1u << 5)  // PORTD Bit 5
#define PIN_Y_MIN      (0x1u << 4)  // PORTD Bit 4

#define LED_PIN        (0x1u << 27) // PORTB Bit 27

#define PINHIGH PIOC->PIO_SODR |= LED_PIN; // SODR: Set   Output Data Register
#define PINLOW  PIOC->PIO_CODR |= LED_PIN; // CODR: Clear Output Data Register


ISR/IRQ    TC             Channel	   Due pins
TC0	    TC0	        0	    2, 13
TC1	    TC0	        1	    60, 61
TC2	    TC0	        2	    58
TC3	    TC1	        0	    none  <- this line in the example above
TC4	    TC1	        1	    none
TC5	    TC1	        2	    none
TC6	    TC2	        0	    4, 5
TC7	    TC2	        1	    3, 10
TC8	    TC2	        2	    11, 12

TC      Chan      NVIC "irq"   IRQ handler function   PMC id
TC0	   0	     TC0_IRQn        TC0_Handler  	     ID_TC0
TC0	   1	     TC1_IRQn        TC1_Handler  	     ID_TC1
TC0	   2	     TC2_IRQn        TC2_Handler  	     ID_TC2
TC1	   0	     TC3_IRQn        TC3_Handler  	     ID_TC3
TC1	   1	     TC4_IRQn        TC4_Handler  	     ID_TC4
TC1	   2	     TC5_IRQn        TC5_Handler  	     ID_TC5
TC2	   0	     TC6_IRQn        TC6_Handler  	     ID_TC6
TC2	   1	     TC7_IRQn        TC7_Handler  	     ID_TC7
TC2	   2	     TC8_IRQn        TC8_Handler  	     ID_TC8

TIMER_CLOCK1  MCK/2
TIMER_CLOCK2  MCK/8
TIMER_CLOCK3  MCK/32
TIMER_CLOCK4  MCK/128

*/
