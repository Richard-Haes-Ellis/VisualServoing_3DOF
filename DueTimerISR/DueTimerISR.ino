#define SPR 1600
#define T1_FREQ 1000000

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
       int goal_vel;
       int goal_acc;

       // Hardware declarations
       uint8_t enable_pin;
       uint8_t step_pin;
       uint8_t dir_pin;

       // Motor status at any given time
       volatile uint8_t dir;       //! Direction stepper axis1 should move.
       volatile uint32_t step_position;
       volatile uint8_t running;
       
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

speedRampData axis1;
speedRampData axis2;

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
                        TC_CMR_TCCLKS_TIMER_CLOCK1 // Settings
       );

       uint32_t rc = VARIANT_MCK / 2 / frequency; //128 because we selected TIMER_CLOCK4 above

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

int speed_cntr_Move(speedRampData &axis)
{
       axis.dir = axis.goal_pos > 0 ? CCW : CW;                // Save the direction state 
       digitalWrite(axis.dir_pin,axis.goal_pos>0?HIGH:LOW);    // Set the direction depending on the steps
       axis.total_steps = abs(axis.goal_pos);                  // Change the sign of the steps to positive
       if(axis.goal_acc == 0){                                  // If we get zero acceleration get out.
              return false;
       }
       // CALCULATE ACCELERATION C0
       axis.step_delay_f = (float)T1_FREQ_148*sqrt((float)2/axis.goal_acc);        // We save the acceleration to the first step delay. (NEEDS CHANGING)
       axis.step_delay = axis.step_delay_f;
       axis.step_count = 0;                            // Initialize the step count to zero 
       axis.n = 0;                                     // Set the ramp counter to zero.
       if(axis.goal_vel == 0){                                  // If we input zero speed get out.
              return false;
       }
       // CALCULATE SPEED MIN_DELAY
       axis.min_delay = T1_FREQ / axis.goal_vel;               // Set the minumum delay needed for the speed given
       axis.rampUpStepCount = 0;                       // Set the ramp counter to zero. Is it not the same as n??
       axis.running = true;                            // Set the axis status to running
       axis.rest = 0;
       TC1->TC_CHANNEL[0].TC_RC = axis.step_delay;     // Set counter ragister to the starting delay (acceleration)
       digitalWrite(axis.enable_pin,LOW);                  // Enable stepper axis
       startTimer(axis.tc,axis.channel,axis.irq);                                    // Start the timer

       Serial.println("Min delay:"    +String(axis.min_delay));
       Serial.println("Start delay:"  +String(axis.step_delay));
       Serial.println("Total steps:"  +String(axis.total_steps));

       Serial.println("Desired position:" +String(axis.goal_pos));
       Serial.println("Acceleration:"     +String(axis.goal_acc));
       Serial.println("Max velocity:"     +String(axis.goal_vel));
       return 1;
}

void TC3_Handler()
{
       TC_GetStatus(axis1.tc, axis1.channel);               // Timer 1 channel 0 ----> TC3 it also clear the flag
       if (axis1.step_count <= axis1.total_steps)
       {
              digitalWrite(axis1.step_pin, HIGH);
              digitalWrite(axis1.step_pin, LOW);
              axis1.step_count++;
              axis1.step_position += axis1.dir;
       }

       if(axis1.step_count > axis1.total_steps) // If we step more that the total it means we have already finished
       {
              axis1.running = false;
              digitalWrite(axis1.enable_pin,HIGH);
              stopTimer(axis1.tc,axis1.channel,axis1.irq);
       }

       if (axis1.rampUpStepCount == 0) // If we are ramping up 
       { 
              // Calculate next delays 
              axis1.n++;    
              axis1.step_delay_f = axis1.step_delay_f - (2 * axis1.step_delay_f /*+ axis1.rest */) / (4 * axis1.n + 1);

              // If we reach max speed by checkig if the calculated delay is smaller that the one we calculated with the max speed
              if (axis1.step_delay_f <=  (float)axis1.min_delay)
              { 
                     axis1.step_delay_f = axis1.min_delay;       // We saturate it to that minimum delay 
                     axis1.rampUpStepCount = axis1.step_count; // We save the number of steps it took to reach this speed
              }
              // If istead we manage to reach half way without reaching full speed we have to start decelerating 
              if (axis1.step_count >= (axis1.total_steps / 2))
              { 
                     axis1.rampUpStepCount = axis1.step_count; // So we save the number of steps it took to accelerate to this speed 
              }
       }
       // If we dont have to ramp down yet (we shoudl be in eather run or acelerating)
       else if (axis1.step_count >= axis1.total_steps - axis1.rampUpStepCount)
       { 
              if(axis1.n!=0)
              {
                     axis1.step_delay_f = ( axis1.step_delay_f * (4 * axis1.n + 1)) / (4 * axis1.n + 1 - 2); // THis is the same as the other equation but inverted
                     
              }
              axis1.n--;
       }
       axis1.step_delay = axis1.step_delay_f;
       axis1.tc->TC_CHANNEL[axis1.channel].TC_RC = axis1.step_delay;
}

void TC4_Handler()
{
       TC_GetStatus(axis2.tc, axis2.channel);               // Timer 1 channel 0 ----> TC3 it also clear the flag
       if (axis2.step_count <= axis2.total_steps)
       {
              digitalWrite(axis2.step_pin, HIGH);
              digitalWrite(axis2.step_pin, LOW);
              axis2.step_count++;
              axis2.step_position += axis2.dir;
       }

       if(axis2.step_count > axis2.total_steps) // If we step more that the total it means we have already finished
       {
              axis2.running = false;
              digitalWrite(axis2.enable_pin,HIGH);
              stopTimer(axis2.tc,axis2.channel,axis2.irq);
       }

       if (axis2.rampUpStepCount == 0) // If we are ramping up 
       { 
              // Calculate next delays 
              axis2.n++;    
              axis2.step_delay_f = axis2.step_delay_f - (2 * axis2.step_delay_f /*+ axis2.rest */) / (4 * axis2.n + 1);

              // If we reach max speed by checkig if the calculated delay is smaller that the one we calculated with the max speed
              if (axis2.step_delay_f <=  (float)axis2.min_delay)
              { 
                     axis2.step_delay_f = axis2.min_delay;       // We saturate it to that minimum delay 
                     axis2.rampUpStepCount = axis2.step_count; // We save the number of steps it took to reach this speed
              }
              // If istead we manage to reach half way without reaching full speed we have to start decelerating 
              if (axis2.step_count >= (axis2.total_steps / 2))
              { 
                     axis2.rampUpStepCount = axis2.step_count; // So we save the number of steps it took to accelerate to this speed 
              }
       }
       // If we dont have to ramp down yet (we shoudl be in eather run or acelerating)
       else if (axis2.step_count >= axis2.total_steps - axis2.rampUpStepCount)
       { 
              if(axis2.n!=0)
              {
                     axis2.step_delay_f = ( axis2.step_delay_f * (4 * axis2.n + 1)) / (4 * axis2.n + 1 - 2); // THis is the same as the other equation but inverted
                     
              }
              axis2.n--;
       }
       axis2.step_delay = axis2.step_delay_f;
       axis2.tc->TC_CHANNEL[axis2.channel].TC_RC = axis2.step_delay;
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

       delay(3000);


       Serial.println("Starting timer..");
       configureTimer(/*Timer TC1*/ TC1, /*Channel 0*/ 0, /*TC3 interrupt nested vector controller*/ TC3_IRQn, /*Frequency in hz*/ T1_FREQ);
       configureTimer(/*Timer TC1*/ TC1, /*Channel 0*/ 1, /*TC3 interrupt nested vector controller*/ TC4_IRQn, /*Frequency in hz*/ T1_FREQ);

       // Axis 1
       axis1.enable_pin     = PIN_Z_ENABLE;
       axis1.step_pin       = PIN_Z_STEP;
       axis1.dir_pin        = PIN_Z_DIR;

       axis1.goal_pos = 70000;
       axis1.goal_acc = 25; // MAX FOR THIS AXIS
       axis1.goal_vel = 1000; // MAX for this axis

       axis1.tc = TC1;
       axis1.channel = 0;
       axis1.irq = TC3_IRQn;

       // Axis 2
       axis2.enable_pin     = PIN_Y_ENABLE;
       axis2.step_pin       = PIN_Y_STEP;
       axis2.dir_pin        = PIN_Y_DIR;

       axis2.goal_pos = 10000;
       axis2.goal_acc = 25; // MAX FOR THIS AXIS
       axis2.goal_vel = 500; // MAX for this axis

       axis2.tc = TC1;
       axis2.channel = 1;
       axis2.irq = TC4_IRQn;
}

void loop()
{
       // speed_cntr_Move(axis1.goal_pos,axis1.goal_acc, axis1.goal_vel);
       speed_cntr_Move(axis1);
       speed_cntr_Move(axis2);

       do
       {
              Serial.println(" n:"            + String(axis1.n)              +
                             " step:"         + String(axis1.step_count)     +
                             " running:"      + String(axis1.running)        +
                             " step_pos:"     + String(axis1.step_position)  +
                             " step_delay:"   + String(axis1.step_delay));
       }while(axis1.running == true);
       Serial.println("Finished movement");

       // Stay there
       while(1){

       }
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
