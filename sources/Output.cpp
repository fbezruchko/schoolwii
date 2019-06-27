#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

#if defined(SERVO)
void initializeServo();
#endif

/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
uint8_t PWM_PIN[8] = {9, 10, 11, 8, 6, 5, A2, 12};
#endif

/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/

#if defined(SERVO)
volatile uint8_t atomicServo[8] = {125, 125, 125, 125, 125, 125, 125, 5};
#endif

/**************************************************************************************/
/***************       Calculate first and last used servos        ********************/
/**************************************************************************************/
#if defined(SERVO)
#if defined(PRI_SERVO_FROM)
#define SERVO_START PRI_SERVO_FROM
#endif
#if defined(PRI_SERVO_TO)
#define SERVO_END PRI_SERVO_TO
#endif
#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {
#if defined(SERVO)
#if defined(PRI_SERVO_FROM)   // write primary servos
  for (uint8_t i = (PRI_SERVO_FROM - 1); i < PRI_SERVO_TO; i++) {
    atomicServo[i] = (servo[i] - 1000) >> 2;
  }
#endif
#endif
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]

  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
#if defined(PROMINI)
#if (NUMBER_MOTOR > 0)
#ifdef EXT_MOTOR_RANGE            // 490Hz
  OCR1A = ((motor[0] >> 2) - 250);
#elif defined(EXT_MOTOR_32KHZ)
  OCR1A = (motor[0] - 1000) >> 2; //  pin 9
#elif defined(EXT_MOTOR_4KHZ)
  OCR1A = (motor[0] - 1000) << 1;
#elif defined(EXT_MOTOR_1KHZ)
  OCR1A = (motor[0] - 1000) << 3;
#else
  OCR1A = motor[0] >> 3; //  pin 9
#endif
#endif
#if (NUMBER_MOTOR > 1)
#ifdef EXT_MOTOR_RANGE            // 490Hz
  OCR1B = ((motor[1] >> 2) - 250);
#elif defined(EXT_MOTOR_32KHZ)
  OCR1B = (motor[1] - 1000) >> 2; //  pin 10
#elif defined(EXT_MOTOR_4KHZ)
  OCR1B = (motor[1] - 1000) << 1;
#elif defined(EXT_MOTOR_1KHZ)
  OCR1B = (motor[1] - 1000) << 3;
#else
  OCR1B = motor[1] >> 3; //  pin 10
#endif
#endif
#endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i = 0; i < NUMBER_MOTOR; i++) {
    motor[i] = mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for (uint8_t i = 0; i < NUMBER_MOTOR; i++) {
    pinMode(PWM_PIN[i], OUTPUT);
  }

  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
#if defined(PROMINI)
#if defined(EXT_MOTOR_32KHZ)
  TCCR1A = (1 << WGM11); // phase correct mode & no prescaler
  TCCR1B = (1 << WGM13) | (1 << CS10);
  ICR1   = 0x00FF; // TOP to 255;
  TCCR2B =  (1 << CS20);
#elif defined(EXT_MOTOR_4KHZ)
  TCCR1A = (1 << WGM11); // phase correct mode & no prescaler
  TCCR1B = (1 << WGM13) | (1 << CS10);
  ICR1   = 0x07F8; // TOP to 1023;
  TCCR2B =  (1 << CS21);
#elif defined(EXT_MOTOR_1KHZ)
  TCCR1A = (1 << WGM11); // phase correct mode & no prescaler
  TCCR1B = (1 << WGM13) | (1 << CS10);
  ICR1   = 0x1FE0; // TOP to 8184;
  TCCR2B =  (1 << CS20) | (1 << CS21);
#endif

#if (NUMBER_MOTOR > 0)
  TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
#endif
#if (NUMBER_MOTOR > 1)
  TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
#endif
#endif

  writeAllMotors(MINCOMMAND);
  delay(300);
#if defined(SERVO)
  initializeServo();
#endif
}


#if defined(SERVO)
/**************************************************************************************/
/************                Initialize the PWM Servos               ******************/
/**************************************************************************************/
void initializeServo() {
  // do pins init
#if (PRI_SERVO_FROM == 3)
  SERVO_3_PINMODE;
#endif
  SERVO_4_PINMODE;
  SERVO_5_PINMODE;
  SERVO_6_PINMODE;
  SERVO_7_PINMODE;
  SERVO_8_PINMODE;

#if defined(SERVO_1_HIGH)
#if defined(PROMINI) // uses timer 0 Comperator A (8 bit)
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt
#define SERVO_1K_US 250
#endif
#endif
}

/**************************************************************************************/
/************              Servo software PWM generation             ******************/
/**************************************************************************************/

// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us


#if defined(SERVO_1_HIGH)

ISR(TIMER0_COMPA_vect) {
  static uint8_t servo_state = 0; // indicates the current state of the chain
  if (servo_state == 0) {
    SERVO_1_HIGH; // set servo 1's pin high
    OCR0A += SERVO_1K_US; // wait 1000us
    servo_state++; // count up the state
  } else if (servo_state == 1) {
    OCR0A += atomicServo[SERVO_1_ARR_POS]; // load the servo's value (0-1000us)
    servo_state++; // count up the state

  } else if (servo_state == 2) {
    SERVO_1_LOW;
    SERVO_2_HIGH;
    OCR0A += SERVO_1K_US;
    servo_state++;
  } else if (servo_state == 2 + 1) {
    OCR0A += atomicServo[SERVO_2_ARR_POS];
    servo_state++;

  } else if (servo_state == 4) {
    SERVO_2_LOW;
    SERVO_3_HIGH;
    OCR0A += SERVO_1K_US;
    servo_state++;
  } else if (servo_state == 4 + 1) {
    OCR0A += atomicServo[SERVO_3_ARR_POS];
    servo_state++;

  } else if (servo_state == 6) {
    SERVO_3_LOW;
    SERVO_4_HIGH;
    OCR0A += SERVO_1K_US;
    servo_state++;
  } else if (servo_state == 6 + 1) {
    OCR0A += atomicServo[SERVO_4_ARR_POS];
    servo_state++;

  } else if (servo_state == 8) {
    SERVO_4_LOW;
    SERVO_5_HIGH;
    OCR0A += SERVO_1K_US;
    servo_state++;
  } else if (servo_state == 8 + 1) {
    OCR0A += atomicServo[SERVO_5_ARR_POS];
    servo_state++;

#if defined (SERVO_6_HIGH)
  } else if (servo_state == 10) {
    SERVO_5_LOW;
    SERVO_6_HIGH;
    OCR0A += SERVO_1K_US;
    servo_state++;
  } else if (servo_state == 10 + 1) {
    OCR0A += atomicServo[SERVO_6_ARR_POS];
    servo_state++;
#endif
  } else {
    LAST_LOW;
    OCR0A += SERVO_1K_US;
    if (servo_state < 30) {
      servo_state += 2;
    } else {
      servo_state = 0;
    }
  }
}
#endif
#endif

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/

// get servo middle point from Config or from RC-Data
int16_t get_middle(uint8_t nr) {
  return (conf.servoConf[nr].middle < RC_CHANS) ? rcData[conf.servoConf[nr].middle] : conf.servoConf[nr].middle;
}

void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  /****************                   main Mix Table                ******************/
  /*****************************               AIRPLANE                **************************************/
  // servo[7] is programmed with safty features to avoid motorstarts when ardu reset..
  // All other servos go to center at reset..  Half throttle can be dangerus
  // Only use servo[7] as motorcontrol if motor is used in the setup            */
  if (!f.ARMED) {
    servo[7] = MINCOMMAND; // Kill throttle when disarmed
  } else {
    servo[7] = constrain(rcCommand[THROTTLE], conf.minthrottle, MAXTHROTTLE);
  }
  motor[0] = servo[7];

  if (f.PASSTHRU_MODE) { // Direct passthru from RX
    servo[3] = rcCommand[ROLL];     //   Wing 1
    servo[4] = rcCommand[ROLL];     //   Wing 2
    servo[5] = rcCommand[YAW];                      //   Rudder
    servo[6] = rcCommand[PITCH];                    //   Elevator
  } else {
    // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
    servo[3] = axisPID[ROLL];   //   Wing 1
    servo[4] = axisPID[ROLL];   //   Wing 2
    servo[5] = axisPID[YAW];                    //   Rudder
    servo[6] = axisPID[PITCH];                  //   Elevator
  }
  for (i = 3; i < 7; i++) {
    servo[i]  = ((int32_t)conf.servoConf[i].rate * servo[i]) / 100L; // servo rates
    servo[i] += get_middle(i);
  }

  /************************************************************************************************************/
  // add midpoint offset, then scale and limit servo outputs - except SERVO8 used commonly as Moror output
  // don't add offset for camtrig servo (SERVO3)
#if defined(SERVO)
  for (i = SERVO_START - 1; i < SERVO_END; i++) {
    if (i < 2) {
      servo[i] = map(servo[i], 1020, 2000, conf.servoConf[i].min, conf.servoConf[i].max);  // servo travel scaling, only for gimbal servos
    }
    servo[i] = constrain(servo[i], conf.servoConf[i].min, conf.servoConf[i].max); // limit the values
  }
#endif

  /****************                normalize the Motors values                ******************/
  maxMotor = motor[0];
  for (i = 1; i < NUMBER_MOTOR; i++)
    if (motor[i] > maxMotor) maxMotor = motor[i];
  for (i = 0; i < NUMBER_MOTOR; i++) {
    if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
    if ((rcData[THROTTLE] < MINCHECK))
#ifndef MOTOR_STOP
      motor[i] = conf.minthrottle;
#else
      motor[i] = MINCOMMAND;
#endif
    if (!f.ARMED)
      motor[i] = MINCOMMAND;
  }
}
