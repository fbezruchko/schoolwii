#ifndef DEF_H_
#define DEF_H_

/**************************************************************************************/
/***************             Proc specific definitions             ********************/
/**************************************************************************************/
// Proc auto detection
#if defined(__AVR_ATmega328P__)
  #define PROMINI
#endif

/**************************************************************************************/
/***************             motor and servo numbers               ********************/
/**************************************************************************************/
#define SERVO_RATES      {30,30,100,100,100,100,100,100}

#define FIXEDWING
#define COPTER_WITH_SERVO
#define SERVO
  
  #if defined (USE_THROTTLESERVO)
    #define NUMBER_MOTOR   0
    #define PRI_SERVO_TO   8
  #else
    #define NUMBER_MOTOR   1
    #define PRI_SERVO_TO   7
  #endif
  
  #define PRI_SERVO_FROM   3

/**************************   atmega328P (Promini)  ************************************/
#if defined(PROMINI)
  #define LEDPIN_PINMODE             DDRB |= 1<<5;     // Arduino pin 13
  #define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 PORTB &= ~(1<<5);
  #define LEDPIN_ON                  PORTB |= (1<<5);

  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
  #define PPM_PIN_PINMODE            DDRD &= ~(1<<2); // pin 2 input
  #define PPM_PIN_ISR_ATTACH         attachInterrupt(0, rxInt, RISING); //PIN 0
  #define PPM_PIN_ISR_DISABLE        EIMSK &= ~(1 << INT0);
  #define PPM_PIN_ISR_ENABLE         EIMSK |= (1 << INT0); 
  #define RX_SERIAL_PORT             0
    
  #define V_BATPIN                   A3    // Analog PIN 1

  #define SERVO_3_PINMODE            DDRC |= 1<<2; // pin A2  // CAM TRIG  - alt TILT_PITCH
  #define SERVO_3_PIN_HIGH           PORTC |= 1<<2;
  #define SERVO_3_PIN_LOW            PORTC &= ~(1<<2);
  
  #define SERVO_4_PINMODE            DDRB |= 1<<4; // pin 12  // new       - alt TILT_ROLL
  #define SERVO_4_PIN_HIGH           PORTB |= 1<<4;
  #define SERVO_4_PIN_LOW            PORTB &= ~(1<<4);
  
  #define SERVO_5_PINMODE            DDRB |= 1<<3; // pin 11  // BI LEFT
  #define SERVO_5_PIN_HIGH           PORTB |= 1<<3;
  #define SERVO_5_PIN_LOW            PORTB &= ~(1<<3);

  #define SERVO_6_PINMODE            DDRB|= 1<<0; // pin D8
  #define SERVO_6_PIN_HIGH           PORTB|= 1<<0;
  #define SERVO_6_PIN_LOW            PORTB &= ~(1<<0);
  
  #define SERVO_7_PINMODE            DDRB |= 1<<2; // pin 10  // new
  #define SERVO_7_PIN_HIGH           PORTB |= 1<<2;
  #define SERVO_7_PIN_LOW            PORTB &= ~(1<<2);
  
  #define SERVO_8_PINMODE            DDRB |= 1<<1; // pin 9  // new
  #define SERVO_8_PIN_HIGH           PORTB |= 1<<1;
  #define SERVO_8_PIN_LOW            PORTB &= ~(1<<1);


#endif

/**********************   Sort the Servos for the most ideal SW PWM     ************************/
// this define block sorts the above slected servos to be in a simple order from 1 - (count of total servos)
// its pretty fat but its the best way i found to get less compiled code and max speed in the ISR without loosing its flexibility

#if defined(SERVO_8_PIN_LOW)
  #define LAST_LOW SERVO_8_PIN_LOW
#else
  #define LAST_LOW SERVO_7_PIN_LOW
#endif

#if (PRI_SERVO_FROM == 3)
  #define SERVO_1_HIGH SERVO_3_PIN_HIGH
  #define SERVO_1_LOW SERVO_3_PIN_LOW
  #define SERVO_1_ARR_POS 2

  #define SERVO_2_HIGH SERVO_4_PIN_HIGH
  #define SERVO_2_LOW SERVO_4_PIN_LOW
  #define SERVO_2_ARR_POS 3

  #define SERVO_3_HIGH SERVO_5_PIN_HIGH
  #define SERVO_3_LOW SERVO_5_PIN_LOW
  #define SERVO_3_ARR_POS 4

  #define SERVO_4_HIGH SERVO_6_PIN_HIGH
  #define SERVO_4_LOW SERVO_6_PIN_LOW 
  #define SERVO_4_ARR_POS 5

  #define SERVO_5_HIGH SERVO_7_PIN_HIGH
  #define SERVO_5_LOW SERVO_7_PIN_LOW 
  #define SERVO_5_ARR_POS 6

  #if (PRI_SERVO_TO > 7)
    #define SERVO_6_HIGH SERVO_8_PIN_HIGH
    #define SERVO_6_LOW SERVO_8_PIN_LOW 
    #define SERVO_6_ARR_POS 7
  #endif

#endif

/**************************************************************************************/
/***************      IMU Orientations and Sensor definitions      ********************/
/**************************************************************************************/

#define ACC 1
#define MAG 1
#define GYRO 1
#define GPS 1

#if defined(GY_521)
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}
  #undef INTERNAL_I2C_PULLUPS
#endif

#if (MAG)
#define HMC5883
#define MAG_ORIENTATION(X, Y, Z) {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = -Z;}
#define MPU6050_EN_I2C_BYPASS // MAG connected to the AUX I2C bus of MPU6050
#endif

/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/

#if defined(USE_MSP_WP)
  #define NAVCAP 1
#else
  #define NAVCAP 0
#endif

/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/

#define MULTITYPE 14    
#define SERVO_RATES      {30,30,100,100,-100,100,100,100}

/**************************************************************************************/
/***************          Some unsorted "chain" defines            ********************/
/**************************************************************************************/

//#define RC_CHANS 12
#define RC_CHANS 6

/**************************************************************************************/
/***************               Error Checking Section              ********************/
/**************************************************************************************/

#ifndef NUMBER_MOTOR
        #error "NUMBER_MOTOR is not set, most likely you have not defined any type of multicopter"
#endif

#if GPS && !defined(NMEA)
  #error "when using GPS you must specify the protocol NMEA"
#endif

#endif /* DEF_H_ */
