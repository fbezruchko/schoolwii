/*
  MultiWiiCopter by Alexandre Dubus
  www.multiwii.com
  March  2015     V2.4
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version. see <http://www.gnu.org/licenses/>
*/

#include <avr/io.h>

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"
#include "Output.h"
#include "RX.h"
#include "Sensors.h"
#include "Serial.h"
#include "GPS.h"
#include "Protocol.h"

#include <avr/pgmspace.h>

/*********** RC alias *****************/

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
  ;

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ARM;"
#if MAG
  "MAG;"
#endif
#if GPS
  "GPS HOME;"
  "GPS HOLD;"
#endif
  "PASSTHRU;"
  //#ifdef INFLIGHT_ACC_CALIBRATION
  //  "CALIB;"
  //#endif
#if GPS
  "MISSION;"
  "LAND;"
#endif
  ;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
#if MAG
  5, //"MAG;"
#endif
#if GPS
  10, //"GPS HOME;"
  11, //"GPS HOLD;"
#endif
  12, //"PASSTHRU;"
  //#ifdef INFLIGHT_ACC_CALIBRATION
  //  17, //"CALIB;"
  //#endif
#if GPS
  20, //"MISSION;"
  21, //"LAND;"
#endif
};

uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingG;
int16_t  magHold; // [-180;+180]
uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
uint8_t  rcOptions[CHECKBOXITEMS];

// **************
// gyro+acc IMU
// **************
int16_t gyroZero[3] = {0, 0, 0};

imu_t imu;

analog_t analog;

alt_t alt;

att_t att;

int16_t  debug[4];

flags_struct_t f;

//for log
int16_t  i2c_errors_count = 0;


// **********************
//Automatic ACC Offset Calibration
// **********************
//#if defined(INFLIGHT_ACC_CALIBRATION)
//uint16_t InflightcalibratingA = 0;
//int16_t AccInflightCalibrationArmed;
//uint16_t AccInflightCalibrationMeasurementDone = 0;
//uint16_t AccInflightCalibrationSavetoEEProm = 0;
//uint16_t AccInflightCalibrationActive = 0;
//#endif

uint16_t intPowerTrigger1;

// ******************
// rc functions
// ******************
#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

int16_t rcData[RC_CHANS];    // interval [1000;2000]
int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP
int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data
int16_t lookupPitchRollRC[5];// lookup table for expo & RC rate PITCH+ROLL
uint16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

// *************************
// motor and servo functions
// *************************
int16_t axisPID[3];
int16_t motor[NUMBER_MOTOR + 1];
int16_t servo[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1000};

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[2], dynD8[2];

global_conf_t global_conf;

conf_t conf;

// **********************
// GPS common variables, no need to put them in defines, since compiller will optimize out unused variables
// **********************
#if GPS
gps_conf_struct GPS_conf;
#endif
int16_t  GPS_angle[2] = { 0, 0};                      // the angles that must be applied for GPS correction
int32_t  GPS_coord[2];
int32_t  GPS_home[2];
int32_t  GPS_hold[2];
int32_t  GPS_prev[2];                                 //previous pos
int32_t  GPS_poi[2];
uint8_t  GPS_numSat;
uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
int16_t  GPS_directionToHome;                         // direction to home - unit: degree
int32_t  GPS_directionToPoi;
//uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10

//uint8_t GPS_mode  = GPS_MODE_NONE; // contains the current selected gps flight mode --> moved to the f. structure
uint8_t NAV_state = 0; // NAV_STATE_NONE;  /// State of the nav engine
uint8_t NAV_error = 0; // NAV_ERROR_NONE;
uint8_t prv_gps_modes = 0;              /// GPS_checkbox items packed into 1 byte for checking GPS mode changes
uint32_t nav_timer_stop = 0;            /// common timer used in navigation (contains the desired stop time in millis()
uint16_t nav_hold_time;                 /// time in seconds to hold position
uint8_t NAV_paused_at = 0;              // This contains the mission step where poshold paused the runing mission.

uint8_t next_step = 1;                  /// The mission step which is upcoming it equals with the mission_step stored in EEPROM
int16_t jump_times = -10;
#if GPS
mission_step_struct mission_step;
#endif

// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
int16_t  nav[2];
int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

// The orginal altitude used as base our new altitude during nav
//int32_t original_altitude;
//This is the target what we want to reach
//int32_t target_altitude;
//This is the interim value which is feeded into the althold controller
//int32_t alt_to_hold;

//uint32_t alt_change_timer;
//int8_t alt_change_flag;
//uint32_t alt_change;

uint8_t alarmArray[ALRM_FAC_SIZE];           // array

void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp, tmp2;
  uint8_t axis, prop1, prop2;

  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value (or collective.pitch value for heli)
#define DYN_THR_PID_CHANNEL THROTTLE
  prop2 = 128; // prop2 was 100, is 128 now
  if (rcData[DYN_THR_PID_CHANNEL] > 1500) { // breakpoint is fix: 1500
    if (rcData[DYN_THR_PID_CHANNEL] < 2000) {
      prop2 -=  ((uint16_t)conf.dynThrPID * (rcData[DYN_THR_PID_CHANNEL] - 1500) >> 9); //  /512 instead of /500
    } else {
      prop2 -=  conf.dynThrPID;
    }
  }

  for (axis = 0; axis < 3; axis++) {
    tmp = min(abs(rcData[axis] - MIDRC), 500);
#if defined(DEADBAND)
    if (tmp > DEADBAND) {
      tmp -= DEADBAND;
    }
    else {
      tmp = 0;
    }
#endif
    if (axis != 2) { //ROLL & PITCH
      tmp2 = tmp >> 7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = lookupPitchRollRC[tmp2] + ((tmp - (tmp2 << 7)) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) >> 7);
      prop1 = 128 - ((uint16_t)conf.rollPitchRate * tmp >> 9); // prop1 was 100, is 128 now -- and /512 instead of /500
      prop1 = (uint16_t)prop1 * prop2 >> 7; // prop1: max is 128   prop2: max is 128   result prop1: max is 128
      dynP8[axis] = (uint16_t)conf.pid[axis].P8 * prop1 >> 7; // was /100, is /128 now
      dynD8[axis] = (uint16_t)conf.pid[axis].D8 * prop1 >> 7; // was /100, is /128 now
    } else {      // YAW
      rcCommand[axis] = tmp;
    }
    if (rcData[axis] < MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE], MINCHECK, 2000);
  tmp = (uint32_t)(tmp - MINCHECK) * 2559 / (2000 - MINCHECK); // [MINCHECK;2000] -> [0;2559]
  tmp2 = tmp / 256; // range [0;9]
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 256) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 256; // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]

  // query at most one multiplexed analog channel per MWii cycle
  static uint8_t analogReader = 0;
  switch (analogReader++ % (3 + VBAT_CELLS_NUM)) {
    case 0:
      {
        break;
      }

    case 1:
      {
#if defined(VBAT)
        static uint8_t ind = 0;
        static uint16_t vvec[VBAT_SMOOTH], vsum;
        uint16_t v = analogRead(V_BATPIN);
#if VBAT_SMOOTH == 1
        analog.vbat = (v * VBAT_PRESCALER) / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
#else
        vsum += v;
        vsum -= vvec[ind];
        vvec[ind++] = v;
        ind %= VBAT_SMOOTH;
#if VBAT_SMOOTH == VBAT_PRESCALER
        analog.vbat = vsum / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
#elif VBAT_SMOOTH < VBAT_PRESCALER
        analog.vbat = (vsum * (VBAT_PRESCALER / VBAT_SMOOTH)) / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
#else
        analog.vbat = ((vsum / VBAT_SMOOTH) * VBAT_PRESCALER) / conf.vbatscale + VBAT_OFFSET; // result is Vbatt in 0.1V steps
#endif
#endif
#endif // VBAT
        break;
      }
    case 2:
      {
        break;
      }
    default: // here analogReader >=4, because of ++ in switch()
      {
        break;
      } // end default
  } // end of switch()

  if ( (calibratingA > 0 && ACC ) || (calibratingG > 0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {
      LEDPIN_OFF;
    }
    if (f.ARMED) {
      LEDPIN_ON;
    }
  }

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  //sserial
  //Only one serial port on ProMini.  Skip serial com if SERIAL RX in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
  serialCom();

#if (GPS)
  {
    sserialCom();
    if (GPS_Frame) 
    {
      PPM_PIN_ISR_ENABLE;
      //sei();
    }
  }
#endif
  //sserial

  if (f.ARMED)  {
#if defined(VBAT)
    if ( (analog.vbat > NO_VBAT) && (analog.vbat < vbatMin) ) vbatMin = analog.vbat;
#endif
  }
}

void setup() {
  SerialOpen(0, SERIAL0_COM_SPEED);
  LEDPIN_PINMODE;
  initOutput();

  if (CLEAR_EEPROM) clearEEPROM();

  readGlobalSet();
  global_conf.currentSet = 0;
  readEEPROM();
  readGlobalSet();
  readEEPROM();// load setting data from last used profile

  blinkLED(2, 40, global_conf.currentSet + 1);

#if GPS
  recallGPSconf();                              //Load GPS configuration parameteres
#endif

  configureReceiver();
  initSensors();
#if GPS
  GPS_set_pids();
#endif
  previousTime = micros();
  calibratingG = 512;
  /************************************/
#if GPS
  GPS_SerialInit();
  GPS_conf.max_wp_number = getMaxWPNumber();
#endif

#ifdef FASTER_ANALOG_READS
  ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
#endif
  f.SMALL_ANGLES_25 = 1; // important for gyro only conf
#ifdef DEBUGMSG
  debugmsg_append_str("initialization completed\n");
#endif
  conf.activate[BOXPASSTHRU] = 1; //default passthru mode
}

void go_arm() {
  if (calibratingG == 0
#if defined(FAILSAFE)
      && failsafeCnt < 2
#endif
#if GPS && defined(ONLY_ALLOW_ARM_WITH_GPS_3DFIX)
      && (f.GPS_FIX && GPS_numSat >= 5)
#endif
     ) {
    if (!f.ARMED) { // arm now!
      f.ARMED = 1;
      magHold = att.heading;
      //f.PASSTHRU_MODE = 1;
      //rcOptions[BOXPASSTHRU] = 1;
#if defined(VBAT)
      if (analog.vbat > NO_VBAT) vbatMin = analog.vbat;
#endif
    }
  } else if (!f.ARMED) {
    blinkLED(2, 255, 1);
    SET_ALARM(ALRM_FAC_ACC, ALRM_LVL_ON);
  }
}
void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
  }
}

// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       // this hold sticks position for command combos
  uint8_t axis, i;
  int16_t error, errorAngle;
  int16_t delta;
  int16_t PTerm = 0, ITerm = 0, DTerm, PTermACC, ITermACC;
  static int16_t lastGyro[2] = {0, 0};
  static int16_t errorAngleI[2] = {0, 0};

  static int32_t errorGyroI_YAW;
  static int16_t delta1[2], delta2[2];
  static int16_t errorGyroI[2] = {0, 0};

  static uint16_t rcTime  = 0;
  static int16_t initialThrottleHold;
  int16_t rc;
  int32_t prop = 0;

  if ((int16_t)(currentTime - rcTime) > 0 ) { // 50Hz
    rcTime = currentTime + 20000;
    computeRC();
    // Failsafe routine - added by MIS
#if defined(FAILSAFE)
    if ( failsafeCnt > (5 * FAILSAFE_DELAY) && f.ARMED) {                // Stabilize, and set Throttle to specified level
      for (i = 0; i < 3; i++) rcData[i] = MIDRC;                          // after specified guard time after RC signal is lost (in 0.1sec)
      rcData[THROTTLE] = conf.failsafe_throttle;
      if (failsafeCnt > 5 * (FAILSAFE_DELAY + FAILSAFE_OFF_DELAY)) {      // Turn OFF motors after specified Time (in 0.1sec)
        go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
        f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
      }
      failsafeEvents++;
    }
    if ( failsafeCnt > (5 * FAILSAFE_DELAY) && !f.ARMED) { //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
      go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
      f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
    }
    failsafeCnt++;
#endif
    // end of failsafe routine - next change is made with RcOptions setting

    // ------------------ STICKS COMMAND HANDLER --------------------
    // checking sticks positions
    uint8_t stTmp = 0;
    for (i = 0; i < 4; i++) {
      stTmp >>= 2;
      if (rcData[i] > MINCHECK) stTmp |= 0x80;     // check for MIN
      if (rcData[i] < MAXCHECK) stTmp |= 0x40;     // check for MAX
    }
    if (stTmp == rcSticks) {
      if (rcDelayCommand < 250) rcDelayCommand++;
    } else rcDelayCommand = 0;
    rcSticks = stTmp;

    // perform actions
    if (rcData[THROTTLE] <= MINCHECK) {            // THROTTLE at minimum
#if !defined(FIXEDWING)
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0;
      errorGyroI_YAW = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
#endif
      if (conf.activate[BOXARM] > 0) {             // Arming/Disarming via ARM BOX
        if ( rcOptions[BOXARM] && f.OK_TO_ARM ) go_arm(); else if (f.ARMED) go_disarm();
      }
    }
    if (rcDelayCommand == 20) {
      if (f.ARMED) {                  // actions during armed
#ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) go_disarm();    // Disarm via YAW
#endif
#ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
        if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO) go_disarm();    // Disarm via ROLL
#endif
      } else {                        // actions during not armed
        i = 0;
        if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {    // GYRO calibration
          calibratingG = 512;
#if GPS
          GPS_reset_home_position();
#endif

        }
//#if defined(INFLIGHT_ACC_CALIBRATION)
//        else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI) {    // Inflight ACC calibration START/STOP
//          if (AccInflightCalibrationMeasurementDone) {               // trigger saving into eeprom after landing
//            AccInflightCalibrationMeasurementDone = 0;
//            AccInflightCalibrationSavetoEEProm = 1;
//          } else {
//            AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
//          }
//        }
//#endif
        if (rcSticks == THR_LO + YAW_HI + PIT_HI + ROL_CE) {            // Enter LCD config
          previousTime = micros();
        }
#ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) go_arm();      // Arm via YAW
#endif
#ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
        else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI) go_arm();      // Arm via ROLL
#endif
#if ACC
        else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) calibratingA = 512;   // throttle=max, yaw=left, pitch=min
#endif
#if MAG
        else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) f.CALIBRATE_MAG = 1;  // throttle=max, yaw=right, pitch=min
#endif
        i = 0;
        if      (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
          conf.angleTrim[PITCH] += 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
          conf.angleTrim[PITCH] -= 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
          conf.angleTrim[ROLL] += 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
          conf.angleTrim[ROLL] -= 2;
          i = 1;
        }
        if (i) {
          writeParams(1);
          rcDelayCommand = 0;    // allow autorepetition
        }
      }
    }

//#if defined(INFLIGHT_ACC_CALIBRATION)
//    if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > MINCHECK && !rcOptions[BOXARM] ) { // Copter is airborne and you are turning it off via boxarm : start measurement
//      InflightcalibratingA = 50;
//      AccInflightCalibrationArmed = 0;
//    }
//    if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
//      if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone) {
//        InflightcalibratingA = 50;
//      }
//    } else if (AccInflightCalibrationMeasurementDone && !f.ARMED) {
//      AccInflightCalibrationMeasurementDone = 0;
//      AccInflightCalibrationSavetoEEProm = 1;
//    }
//#endif

    uint16_t auxState = 0;
    for (i = 0; i < 4; i++)
      auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);

    for (i = 0; i < CHECKBOXITEMS; i++)
      rcOptions[i] = (auxState & conf.activate[i]) > 0;

    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAFE_DELAY is always false

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;

    if (rcOptions[BOXMAG]) {
      if (!f.MAG_MODE) {
        f.MAG_MODE = 1;
        magHold = att.heading;
      }
    } else {
      f.MAG_MODE = 0;
    }

#if GPS
    // This handles the three rcOptions boxes
    // unlike other parts of the multiwii code, it looks for changes and not based on flag settings
    // by this method a priority can be established between gps option

    //Generate a packed byte of all four GPS boxes.
    uint8_t gps_modes_check = (rcOptions[BOXLAND]<< 3) + (rcOptions[BOXGPSHOME] << 2) + (rcOptions[BOXGPSHOLD] << 1) + (rcOptions[BOXGPSNAV]);

    if (f.ARMED ) {                       //Check GPS status and armed
      //TODO: implement f.GPS_Trusted flag, idea from Dramida - Check for degraded HDOP and sudden speed jumps
      if (f.GPS_FIX) {
        if (GPS_numSat > 5 ) {
          if (prv_gps_modes != gps_modes_check) {                           //Check for change since last loop
            NAV_error = NAV_ERROR_NONE;
            if (rcOptions[BOXGPSHOME]) {                                    // RTH has the priotity over everything else
              init_RTH();
            } else if (rcOptions[BOXGPSHOLD]) {                             //Position hold has priority over mission execution  //But has less priority than RTH
              if (f.GPS_mode == GPS_MODE_NAV)
                NAV_paused_at = mission_step.number;
              f.GPS_mode = GPS_MODE_HOLD;
              GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON], &GPS_coord[LAT], & GPS_coord[LON]); //hold at the current position
              //set_new_altitude(alt.EstAlt);                                //and current altitude
              NAV_state = NAV_STATE_HOLD_INFINIT;
            } else if (rcOptions[BOXLAND]) {                               //Land now (It has priority over Navigation)
              f.GPS_mode = GPS_MODE_HOLD;
              GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON],&GPS_coord[LAT], & GPS_coord[LON]);
              //set_new_altitude(alt.EstAlt);
              NAV_state = NAV_STATE_LANDED;
            } else if (rcOptions[BOXGPSNAV]) {                             //Start navigation
              f.GPS_mode = GPS_MODE_NAV;                                   //Nav mode start
              GPS_prev[LAT] = GPS_coord[LAT];
              GPS_prev[LON] = GPS_coord[LON];
              if (NAV_paused_at != 0) {
                next_step = NAV_paused_at;
                NAV_paused_at = 0;                                         //Clear paused step
              } else {
                next_step = 1;
                jump_times = -10;                                          //Reset jump counter
              }
              NAV_state = NAV_STATE_PROCESS_NEXT;
            } else {                                                       //None of the GPS Boxes are switched on
              f.GPS_mode = GPS_MODE_NONE;
              f.THROTTLE_IGNORED = false;
              f.LAND_IN_PROGRESS = 0;
              f.THROTTLE_IGNORED = 0;
              NAV_state = NAV_STATE_NONE;
              GPS_reset_nav();
            }
            prv_gps_modes = gps_modes_check;
          }
        } else { //numSat>5
          //numSat dropped below 5 during navigation
          if (f.GPS_mode == GPS_MODE_NAV) {
            NAV_paused_at = mission_step.number;
            f.GPS_mode = GPS_MODE_NONE;
            //set_new_altitude(alt.EstAlt);                                  //and current altitude
            NAV_state = NAV_STATE_NONE;
            NAV_error = NAV_ERROR_SPOILED_GPS;
            prv_gps_modes = 0xff;                                          //invalidates mode check, to allow re evaluate rcOptions when numsats raised again
          }
          if (f.GPS_mode == GPS_MODE_HOLD || f.GPS_mode == GPS_MODE_RTH) {
            f.GPS_mode = GPS_MODE_NONE;
            NAV_state = NAV_STATE_NONE;
            NAV_error = NAV_ERROR_SPOILED_GPS;
            prv_gps_modes = 0xff;                                          //invalidates mode check, to allow re evaluate rcOptions when numsats raised again
          }
          nav[0] = 0; nav[1] = 0;
        }
      } else { //f.GPS_FIX
        // GPS Fix dissapeared, very unlikely that we will be able to regain it, abort mission
        f.GPS_mode = GPS_MODE_NONE;
        NAV_state = NAV_STATE_NONE;
        NAV_paused_at = 0;
        NAV_error = NAV_ERROR_GPS_FIX_LOST;
        GPS_reset_nav();
        prv_gps_modes = 0xff;                                              //Gives a chance to restart mission when regain fix
      }
    } else { //copter is armed
      //copter is disarmed
      f.GPS_mode = GPS_MODE_NONE;
      f.THROTTLE_IGNORED = false;
      NAV_state = NAV_STATE_NONE;
      NAV_paused_at = 0;
      NAV_error = NAV_ERROR_DISARMED;
      GPS_reset_nav();
    }

#endif //GPS

    if (rcOptions[BOXPASSTHRU]) {
      f.PASSTHRU_MODE = 1;
    }
    else {
      f.PASSTHRU_MODE = 0;
    }

  } else { // not in rc loop
    static uint8_t taskOrder = 0; // never call all functions in the same loop, to avoid high delay spikes
    switch (taskOrder) {
      case 0:
        taskOrder++;
#if MAG
        if (Mag_getADC() != 0) break; // 320 µs
#endif
      case 1:
        taskOrder++;
      case 2:
        taskOrder++;
      case 3:
        taskOrder++;
#if GPS
        if (GPS_Compute() != 0) break;  // performs computation on new frame only if present
#if defined(I2C_GPS)
        if (GPS_NewData() != 0) break;  // 160 us with no new data / much more with new data
#endif
#else
        //delayMicroseconds(160);
#endif
      case 4:
        taskOrder = 0;
        break;
    }
  }

  while (1) {
    currentTime = micros();
    cycleTime = currentTime - previousTime;
#if defined(LOOP_TIME)
    if (cycleTime >= LOOP_TIME) break;
#else
    break;
#endif
  }
  previousTime = currentTime;

  computeIMU();

  //***********************************
  // THROTTLE sticks during mission and RTH
#if GPS
  if (GPS_conf.ignore_throttle == 1) {
    if (f.GPS_mode == GPS_MODE_NAV || f.GPS_mode == GPS_MODE_RTH) {
      //rcCommand[ROLL] = 0;
      //rcCommand[PITCH] = 0;
      //rcCommand[YAW] = 0;
      f.THROTTLE_IGNORED = 1;
    } else
      f.THROTTLE_IGNORED = 0;
  }

  //Heading manipulation TODO: Do heading manipulation
#endif

  if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) {
    int16_t dif = att.heading - magHold;
    if (dif <= - 180) dif += 360;
    if (dif >= + 180) dif -= 360;
    if (f.SMALL_ANGLES_25 || (f.GPS_mode != 0)) rcCommand[YAW] -= dif * conf.pid[PIDMAG].P8 >> 5; //Always correct maghold in GPS mode
  } else magHold = att.heading;

#if GPS
  //TODO: split cos_yaw calculations into two phases (X and Y)
  if (( f.GPS_mode != GPS_MODE_NONE ) && f.GPS_FIX_HOME ) {
    float sin_yaw_y = sin(att.heading * 0.0174532925f);
    float cos_yaw_x = cos(att.heading * 0.0174532925f);
    GPS_angle[ROLL]   = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
    GPS_angle[PITCH]  = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
  } else {
    GPS_angle[ROLL]  = 0;
    GPS_angle[PITCH] = 0;
  }
#endif //GPS

  // PITCH & ROLL
  for (axis = 0; axis < 2; axis++) {
    rc = rcCommand[axis] << 1;
    error = rc - imu.gyroData[axis];
    errorGyroI[axis]  = constrain(errorGyroI[axis] + error, -16000, +16000);   // WindUp   16 bits is ok here
    if (abs(imu.gyroData[axis]) > 640) errorGyroI[axis] = 0;

    ITerm = (errorGyroI[axis] >> 7) * conf.pid[axis].I8 >> 6;                  // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000

    PTerm = mul(rc, conf.pid[axis].P8) >> 6;

    PTerm -= mul(imu.gyroData[axis], dynP8[axis]) >> 6; // 32 bits is needed for calculation

    delta          = imu.gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = imu.gyroData[axis];
    DTerm          = delta1[axis] + delta2[axis] + delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;

    DTerm = mul(DTerm, dynD8[axis]) >> 5;     // 32 bits is needed for calculation

    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

  //YAW
#define GYRO_P_MAX 300
#define GYRO_I_MAX 250

  rc = mul(rcCommand[YAW] , (2 * conf.yawRate + 30))  >> 5;

  error = rc - imu.gyroData[YAW];
  errorGyroI_YAW  += mul(error, conf.pid[YAW].I8);
  errorGyroI_YAW  = constrain(errorGyroI_YAW, 2 - ((int32_t)1 << 28), -2 + ((int32_t)1 << 28));
  if (abs(rc) > 50) errorGyroI_YAW = 0;

  PTerm = mul(error, conf.pid[YAW].P8) >> 6;
#ifndef COPTER_WITH_SERVO
  int16_t limit = GYRO_P_MAX - conf.pid[YAW].D8;
  PTerm = constrain(PTerm, -limit, +limit);
#endif

  ITerm = constrain((int16_t)(errorGyroI_YAW >> 13), -GYRO_I_MAX, +GYRO_I_MAX);

  axisPID[YAW] =  PTerm + ITerm;

  mixTable();
  // do not update servos during unarmed calibration of sensors which are sensitive to vibration
  #if defined(DISABLE_SERVOS_WHEN_UNARMED)
    if (f.ARMED) writeServos();
  #else
    if ( (f.ARMED) || ((!calibratingG) && (!calibratingA)) ) writeServos();
  #endif
  writeMotors();
}
