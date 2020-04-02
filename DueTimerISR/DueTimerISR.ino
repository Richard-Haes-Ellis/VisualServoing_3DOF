#define SPR 1600
#define T1_FREQ 1000000

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA (2 * 3.14159 / SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA * T1_FREQ * 100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ * 0.676) / 100)) // divided by 100 and scaled by 0.676
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

// Direction of stepper motor movement
#define CW 0
#define CCW 1

#define TRUE 1
#define FALSE 0

typedef struct
{
       unsigned char run_state : 3; //! What part of the speed ramp we are in.
       unsigned char dir : 1;       //! Direction stepper motor should move.
       unsigned int step_delay;     //! Peroid of next timer delay. At start this value set the accelration rate c0.
       unsigned int decel_start;    //! What step_pos to start decelaration
       signed int decel_val;        //! Sets deceleration rate.
       unsigned int min_delay;      //! Minimum time delay (max speed)
       signed int n;                //! Counter used when accelerateing/decelerateing to calculate step_delay.
       unsigned char running : TRUE;
} speedRampData;

speedRampData motor;

int pos = 50000;
int acc = 100;
int vel = 1000;

void startTimer()
{
       NVIC_ClearPendingIRQ(TC3_IRQn);
       NVIC_EnableIRQ(TC3_IRQn);
       TC_Start(TC1, 0);
}

void stopTimer(void)
{
       NVIC_DisableIRQ(TC3_IRQn);
       TC_Stop(TC1, 0);
}

void restartCounter()
{
       // To reset a conter we se the TC_CCR_SWTRG (Software trigger) bit in the TC_CCR
       TC1->TC_CHANNEL[0].TC_CCR |= TC_CCR_SWTRG;
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

void speed_cntr_Move(signed int steps, unsigned int accel, unsigned int speed)
{
       unsigned int steps_2_max_speed; //! Number of steps before we hit max speed.
       unsigned int steps_until_decel; //! Number of steps before we must start deceleration (if accel does not hit max speed).
       unsigned int decel = accel;

       if (steps < 0) // Set direction from sign on steps value.
       {
              motor.dir = CCW;
              steps = -steps; // We make it positive
              digitalWrite(PIN_Z_DIR, HIGH);
       }
       else
       {
              motor.dir = CW;
              digitalWrite(PIN_Z_DIR, LOW);
       }

       if (steps == 1) // If moving only 1 steps.
       {
              motor.n = -1;            // Move one steps...
              motor.run_state = DECEL; // ...in DECEL state.
              motor.step_delay = 1000; // Just a short delay so main() can act on 'running'.
              motor.running = TRUE;

              TC1->TC_CHANNEL[0].TC_RC = 10;
       }

       else if (steps != 0) // Only move if number of steps to move is not zero.
       {

              motor.min_delay = A_T_x100 / speed;                                               // Minimum delay for a given speed
              motor.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel)) / 100;                      // First C0 delay time
              steps_2_max_speed = (long)(speed * speed) / (long)((long)(A_x20000 * accel) / 100); // steps_2_max_speed = speed^2 / (2*alpha*accel)
              Serial.println("Motor minimim step delay:"+String(motor.min_delay));
              Serial.println("Motor initial step delay C0:"+String(motor.step_delay));
              Serial.println("Motor steps to maximum speed:"+String(steps_2_max_speed));

              if (steps_2_max_speed == 0) // We need at least 1 step to get to any speed.
              {
                     steps_2_max_speed = 1;
              }

              steps_until_decel = (steps * 0.5); // n1 = (n1+n2)decel / (accel + decel)

              if (steps_until_decel == 0) // We must accelrate at least 1 steps before we can start deceleration.
              {
                     steps_until_decel = 1;
              }

              if (steps_until_decel <= steps_2_max_speed) // We wont reach desired speed
              {
                     motor.decel_val = steps_until_decel - steps; // n1 - steps  = -n2 ==>> DECEL STEPS
                     Serial.println("Motor cant reach desired speed");
              }
              else // We will reach max speed
              {
                     motor.decel_val = -steps_2_max_speed; // n1*accel1 = n2*accel2 to get to --> n2 = -n1*(accel1/decel2)
                     Serial.println("1:"+String(steps_2_max_speed)+" 2:"+String(accel)+" 3:"+String(decel));
                     Serial.println("Motor can reach desired speed: "+String(motor.decel_val));
              }

              if (motor.decel_val == 0) // We must decelrate at least 1 steps to stop.
              {
                     motor.decel_val = -1;
              }
              Serial.println("Motor deceleration value:"+String(motor.decel_val));

              motor.decel_start = steps + motor.decel_val; // steps - n2 = number of steps befor me start decelerating
              Serial.println("Deceleration starts at step:"+String(motor.decel_start));

              if (motor.step_delay <= motor.min_delay) // If the maximum speed is so low that we dont need to go via accelration state.
              {
                     motor.step_delay = motor.min_delay;
                     motor.run_state = RUN; // Go straght to run state
              }
              else // If step_delay (Initial speed) is larger than minimum delay then we accelerate
              {
                     motor.run_state = ACCEL; // Go to accel state
              }

              motor.n = 0; // Reset acceleration / deceleration counter. Used in the Cn=cn_1-(4*Cn_1+rest)/(4*n+1)
              motor.running = TRUE;
              digitalWrite(PIN_Z_ENABLE, LOW);

              TC1->TC_CHANNEL[0].TC_RC = 10; // Doesent really matter because its the time for the first step, after that it would already be recalculated.
              startTimer();
       }
}

void TC3_Handler()
{
       TC_GetStatus(TC1, 0);               // Timer 1 channel 0 ----> TC3 it also clear the flag
       unsigned int new_step_delay = 0;    // Holds next delay period.
       static int last_accel_delay;        // Remember the last steps delay used when accelrating.
       static unsigned int step_count = 0; // Counting steps when moving.
       static unsigned int rest = 0;       // Keep track of remainder from new_step-delay calculation to incrase accurancy

       TC1->TC_CHANNEL[0].TC_RC = motor.step_delay; // Set counter limit in the timer counter register

       switch (motor.run_state)
       {
       case STOP:
              step_count = 0;
              rest = 0;
              stopTimer();
              motor.running = FALSE;
              digitalWrite(PIN_Z_ENABLE, HIGH);
              break;

       case ACCEL:
              digitalWrite(PIN_Z_STEP, HIGH);
              digitalWrite(PIN_Z_STEP, LOW);
              step_count++;
              motor.n++; // This is the n in the equations


              new_step_delay = motor.step_delay - (((2 * (long)motor.step_delay) + rest) / (4 * motor.n + 1));
              rest = ((2 * (long)motor.step_delay) + rest) % (4 * motor.n + 1);

       
              
              if (step_count >= motor.decel_start)             // Chech if we should start decelration.
              {
                     motor.n = motor.decel_val;                // Is it not the same already??? NO, decel_val is negative.
                     motor.run_state = DECEL;
              } 
              else if (new_step_delay <= motor.min_delay)      // Chech if we hitted max speed.
              {
                     last_accel_delay = new_step_delay;
                     new_step_delay = motor.min_delay;
                     rest = 0;
                     motor.run_state = RUN;
              }
              break;

       case RUN:
              digitalWrite(PIN_Z_STEP, HIGH);
              digitalWrite(PIN_Z_STEP, LOW);
              step_count++;
              new_step_delay = motor.min_delay; // I think its not neccesary
              // Chech if we should start decelration.
              if (step_count >= motor.decel_start)
              {
                     motor.n = motor.decel_val;
                     // Start decelration with same delay as accel ended with.
                     new_step_delay = last_accel_delay;
                     motor.run_state = DECEL;
              }
              break;

       case DECEL:
              digitalWrite(PIN_Z_STEP, HIGH);
              digitalWrite(PIN_Z_STEP, LOW);
              step_count++;
              motor.n++; // Starts out negative and slowarly goes positive.
              new_step_delay = motor.step_delay - (((2 * (long)motor.step_delay) + rest) / (4 * motor.n + 1));
              rest = ((2 * (long)motor.step_delay) + rest) % (4 * motor.n + 1);
              // Check if we at last steps
              if (motor.n >= 0)
              {
                     motor.run_state = STOP;
              }
              break;
       }
       motor.step_delay = new_step_delay;
}

void setup()
{
       Serial.begin(115200);
       pinMode(PIN_Z_ENABLE, OUTPUT);
       pinMode(PIN_Z_DIR, OUTPUT);
       pinMode(PIN_Z_STEP, OUTPUT);
       delay(3000);
       Serial.println("Starting timer..");
       configureTimer(/*Timer TC1*/ TC1, /*Channel 0*/ 0, /*TC3 interrupt nested vector controller*/ TC3_IRQn, /*Frequency in hz*/ T1_FREQ);
}

void loop()
{
       Serial.println("A_T_x100:"         +String(A_T_x100));
       Serial.println("ALPHA:"            +String(ALPHA));
       Serial.println("T1_FREQ_148:"      +String(T1_FREQ_148));
       Serial.println("A_SQ:"             +String(A_SQ));
       Serial.println("A_x20000:"         +String(A_x20000));

       Serial.println("Desired position:" +String(pos));
       Serial.println("Acceleration:"     +String(acc));
       Serial.println("Max velocity:"     +String(vel));

       speed_cntr_Move(pos, acc, vel);
       while (motor.running)
       {
              Serial.println(" n:"            + String(motor.n)           +
                             " run_state:"    + String(motor.run_state)   +
                             " running:"      + String(motor.running)     +
                             " step_delay:"   + String(motor.step_delay));
       }
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
