#ifndef CONFIG_H_
#define CONFIG_H_

/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

/* this file consists of several sections
 * to create a working combination you must at least make your choices in section 1.
 * 1 - BASIC SETUP - you must select an option in every block.
 *      this assumes you have 4 channels connected to your board with standard ESCs and servos.
 * 2 - COPTER TYPE SPECIFIC OPTIONS - you likely want to check for options for your copter type
 * 3 - RC SYSTEM SETUP
 * 4 - ALTERNATE CPUs & BOARDS - if you have
 * 5 - ALTERNATE SETUP - select alternate RX (SBUS, PPM, etc.), alternate ESC-range, etc. here
 * 6 - OPTIONAL FEATURES - enable nice to have features here (FlightModes, LCD, telemetry, battery monitor etc.)
 * 7 - TUNING & DEVELOPER - if you know what you are doing; you have been warned
 *     - (ESCs calibration, Dynamic Motor/Prop Balancing, Diagnostics,Memory savings.....)
 */

/* Notes:
 * 1. parameters marked with (*) in the comment are stored in eeprom and can be changed via serial monitor or LCD.
 * 2. parameters marked with (**) in the comment are stored in eeprom and can be changed via the GUI
 */

    #define CLEAR_EEPROM 0
/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************    The type of multicopter    ****************************/
    #define AIRPLANE

  /****************************    Motor minthrottle    *******************************/
    /* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
       This is the minimum value that allow motors to run at a idle speed  */
    //#define MINTHROTTLE 1150 // (*) (**)
    #define MINTHROTTLE 1500 // (*) (**)

  /****************************    Motor maxthrottle    *******************************/
    /* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
    #define MAXTHROTTLE 1600

  /****************************    Mincommand          *******************************/
    /* this is the value for the ESCs when they are not armed
       in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
    #define MINCOMMAND  1000

  /**********************************  I2C speed for old WMP config (useless config for other sensors)  *************/
    #define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP

  /***************************    Internal i2c Pullups   ********************************/
    /* enable internal I2C pull ups (in most cases it is better to use external pullups) */
    //#define INTERNAL_I2C_PULLUPS

  /**********************************  constant loop time  ******************************/
    #define LOOP_TIME 2800

  /**************************************************************************************/
  /*****************          boards and sensor definitions            ******************/
  /**************************************************************************************/

    /***************************    Combined IMU Boards    ********************************/
      /* if you use a specific sensor board:
         please submit any correction to this list.
           Note from Alex: I only own some boards, for other boards, I'm not sure, the info was gathered via rc forums, be cautious */
      #define GY_521          // Chinese 6  DOF with  MPU6050, LLC

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  2 - COPTER TYPE SPECIFIC OPTIONS                               *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

   /********************************    ARM/DISARM    *********************************/
   /* optionally disable stick combinations to arm/disarm the motors. */
    #define ALLOW_ARM_DISARM_VIA_TX_YAW

    /********************************    SERVOS      *********************************/
    /* info on which servos connect where and how to setup can be found here
     * http://www.multiwii.com/wiki/index.php?title=Config.h#Servos_configuration
     */

    /* Do not move servos if copter is unarmed
     * It is a quick hack to overcome feedback tail wigglight when copter has a flexibile
     * landing gear
    */
    //#define DISABLE_SERVOS_WHEN_UNARMED

    /* if you want to preset min/middle/max values for servos right after flashing, because of limited physical
     * room for servo travel, then you must enable and set all three following options */
     //#define SERVO_MIN {1020, 1020, 1020, 1020, 1020, 1020, 1020, 1020}
     //#define SERVO_MAX {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000}
     //#define SERVO_MID {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500} // (*)
     //#define FORCE_SERVO_RATES {30,30,100,100,100,100,100,100} // 0 = normal, 1= reverse

  /***********************          Airplane                       ***********************/
    #define USE_THROTTLESERVO // For use of standard 50Hz servo on throttle.

  /***********************      Common for Heli & Airplane         ***********************/

    /* tail precomp from collective */
    #define YAW_COLL_PRECOMP 10           // (*) proportional factor in 0.1. Higher value -> higher precomp effect. value of 10 equals no/neutral effect
    #define YAW_COLL_PRECOMP_DEADBAND 120 // (*) deadband for collective pitch input signal around 0-pitch input value

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  3 - RC SYSTEM SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

    /****************************    PPM Sum Reciver    ***********************************/
      /* The following lines apply only for specific receiver with only one PPM sum signal, on digital PIN 2
         Select the right line depending on your radio brand. Feel free to modify the order in your PPM order is different */
      //#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Graupner/Spektrum
      //#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Robe/Hitec/Futaba
      //#define SERIAL_SUM_PPM         ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For Multiplex
      //#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //For some Hitec/Sanwa/Others
      //#define SERIAL_SUM_PPM         THROTTLE,YAW,ROLL,PITCH,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //Modelcraft
      #define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2 //For FlySky 6 channels

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  4 - ALTERNATE CPUs & BOARDS                                    *******/
/*****************                                                                 ***************/
/*************************************************************************************************/


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  5 - ALTERNATE SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /******                Serial com speed    *********************************/
    /* This is the speed of the serial interfaces */
    #define SERIAL0_COM_SPEED 115200

  /**************************************************************************************/
  /********                              Gyro filters                ********************/
  /**************************************************************************************/

    /******                Gyro smoothing    **********************************/
      /* GYRO_SMOOTHING. In case you cannot reduce vibrations _and_ _after_ you have tried the low pass filter options, you
         may try this gyro smoothing via averaging. Not suitable for multicopters!
         Good results for helicopter, airplanes and flying wings (foamies) with lots of vibrations.*/
      //#define GYRO_SMOOTHING {20, 20, 3}    // (*) separate averaging ranges for roll, pitch, yaw

    /************************    Moving Average Gyros    **********************************/
      //#define MMGYRO 10                      // (*) Active Moving Average Function for Gyros
      //#define MMGYROVECTORLENGTH 15          // Length of Moving Average Vector (maximum value for tunable MMGYRO

  /************************    Analog Reads              **********************************/
    /* if you want faster analog Reads, enable this. It may result in less accurate results, especially for more than one analog channel */
    //#define FASTER_ANALOG_READS

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  6 - OPTIONAL FEATURES                                          *******/
/*****************                                                                 ***************/
/*************************************************************************************************/


  /********                          Failsafe settings                 ********************/
    /* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels) 
       the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC is avaliable),
       PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THROTTLE value. You must set this value to descending about 1m/s or so
       for best results. This value is depended from your configuration, AUW and some other params.  Next, after FAILSAFE_OFF_DELAY the copter is disarmed, 
       and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */
    #define FAILSAFE                                // uncomment  to activate the failsafe function
    #define FAILSAFE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
    #define FAILSAFE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
    #define FAILSAFE_THROTTLE  (MINTHROTTLE + 200)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case
    
    #define FAILSAFE_DETECT_TRESHOLD  985

  /*************************    INFLIGHT ACC Calibration    *****************************/
    /* This will activate the ACC-Inflight calibration if unchecked */
    //#define INFLIGHT_ACC_CALIBRATION

  /**************************************************************************************/
  /***********************                  TX-related         **************************/
  /**************************************************************************************/

    /* introduce a deadband around the stick center
       Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw */
    //#define DEADBAND 6

  /**************************************************************************************/
  /***********************                  GPS                **************************/
  /**************************************************************************************/
       
    // 38400 is max baudrate
    #define GPS_BAUD   38400       // GPS_BAUD

   /* GPS protocol 
       NMEA  - Standard NMEA protocol GGA, GSA and RMC  sentences are needed */
   #define NMEA

   //Enables the MSP_WP command set , which is used by WinGUI for displaying an setting up navigation
    #define USE_MSP_WP

   // HOME position is reset at every arm, uncomment it to prohibit it (you can set home position with GyroCalibration)    
    //#define DONT_RESET_HOME_AT_ARM

/* GPS navigation can control the heading */

// copter faces toward the navigation point, maghold must be enabled for it
#define NAV_CONTROLS_HEADING       1    //(**)
// true - copter comes in with tail first
#define NAV_TAIL_FIRST             0    //(**)
// true - when copter arrives to home position it rotates it's head to takeoff direction
#define NAV_SET_TAKEOFF_HEADING    1    //(**)

/* Get your magnetic declination from here : http://magnetic-declination.com/
Convert the degree+minutes into decimal degree by ==> degree+minutes*(1/60)
Note the sign on declination it could be negative or positive (WEST or EAST)
Also note, that maqgnetic declination changes with time, so recheck your value every 3-6 months */
#define MAG_DECLINATION  4.02f   //(**)

// Adds a forward predictive filterig to compensate gps lag. Code based on Jason Short's lead filter implementation
#define GPS_LEAD_FILTER               //(**)

// add a 5 element moving average filter to GPS coordinates, helps eliminate gps noise but adds latency comment out to disable
// use it with NMEA gps only 
//#define GPS_FILTERING                 //(**)

// if we are within this distance to a waypoint then we consider it reached (distance is in cm)
#define GPS_WP_RADIUS              100      //(**)

// Safe WP distance, do not start mission if the first wp distance is larger than this number (in meters)
// Also aborts mission if the next waypoint distance is more than this number
#define SAFE_WP_DISTANCE           500      //(**)

//Maximu allowable navigation altitude (in meters) automatic altitude control will not go above this height
#define MAX_NAV_ALTITUDE           100     //(**)

// minimum speed when approach waypoint
#define NAV_SPEED_MIN              100    // cm/sec //(**)
// maximum speed to reach between waypoints
#define NAV_SPEED_MAX              300    // cm/sec //(**)
// Slow down to zero when reaching waypoint (same as NAV_SPEED_MIN = 0)
#define NAV_SLOW_NAV               0      //(**)
// Weight factor of the crosstrack error in navigation calculations (do not touch)
#define CROSSTRACK_GAIN            .4     //(**)
// Maximum allowable banking than navigation outputs
#define NAV_BANK_MAX 3000                 //(**)

//Throttle stick input will be ignored
#define IGNORE_THROTTLE            1         //(**)

//If FENCE DISTANCE is larger than 0 then controller will switch to RTH when it farther from home
//than the defined number in meters
#define FENCE_DISTANCE      80


   //#define ONLY_ALLOW_ARM_WITH_GPS_3DFIX      // Only allow FC arming if GPS has a 3D fix.

  /********************************************************************/
  /****           battery voltage monitoring                       ****/
  /********************************************************************/
    /* for V BAT monitoring
       after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
       with R1=33k and R2=51k
       vbat = [0;1023]*16/VBATSCALE
       must be associated with #define BUZZER ! */
    #define VBAT              // uncomment this line to activate the vbat code
    #define VBATSCALE       131 // (*) (**) change this value if readed Battery voltage is different than real voltage
    #define VBATNOMINAL     84 // 12,6V full battery nominal voltage - only used for lcd.telemetry
    #define VBATLEVEL_WARN1 71 // (*) (**) 7,1V
    #define VBATLEVEL_WARN2  66 // (*) (**) 6,6V
    #define VBATLEVEL_CRIT   62 // (*) (**) 6.2V - critical condition: if vbat ever goes below this value, permanent alarm is triggered
    #define NO_VBAT          16 // Avoid beeping without any battery
    #define VBAT_OFFSET       0 // offset in 0.1Volts, gets added to voltage value  - useful for zener diodes
    #define VBAT_PRESCALER 16 // set this to 8 if vbatscale would exceed 255
    #define VBAT_SMOOTH 16              // len of averaging vector for smoothing the VBAT readings; should be power of 2; set to 1 to disable
    
    #define VBAT_CELLS_NUM 2 // set this to the number of cells

  /********************************************************************/
  /****           board naming                                     ****/
  /********************************************************************/

    /*
     * this name is displayed together with the MultiWii version number
     * upon powerup on the LCD.
     * If you are without a DISPLAYD then You may enable LCD_TTY and
     * use arduino IDE's serial monitor to view the info.
     *
     * You must preserve the format of this string!
     * It must be 16 characters total,
     * The last 4 characters will be overwritten with the version number.
     */
    #define BOARD_NAME "MultiWii   V-.--"
    //                  123456789.123456

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  7 - TUNING & DEVELOPER                                  **************/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************************************************************************/
  /********   special ESC with extended range [0-2000] microseconds  ********************/
  /**************************************************************************************/
    //#define EXT_MOTOR_RANGE // using this with wii-esc requires to change MINCOMMAND to 1008 for promini and mega

  /**************************************************************************************/
  /********  brushed ESC ****************************************************************/
  /**************************************************************************************/
    // for 328p proc
    //#define EXT_MOTOR_32KHZ
    #define EXT_MOTOR_4KHZ
    //#define EXT_MOTOR_1KHZ

  /**************************************************************************************/
  /***********************     motor, servo and other presets     ***********************/
  /**************************************************************************************/
    /* motors will not spin when the throttle command is in low position
       this is an alternative method to stop immediately the motors */
    #define MOTOR_STOP

    /* some radios have not a neutral point centered on 1500. can be changed here */
    #define MIDRC 1500

  /********************************************************************/
  /****           ESCs calibration                                 ****/
  /********************************************************************/

    #define ESC_CALIB_LOW  MINCOMMAND
    #define ESC_CALIB_HIGH 2000

/*************************************************************************************************/
/****           END OF CONFIGURABLE PARAMETERS                                                ****/
/*************************************************************************************************/

#endif /* CONFIG_H_ */
