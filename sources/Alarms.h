#ifndef ALARMS_H_
#define ALARMS_H_

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat);
void alarmHandler(void);
void i2CLedRingState(void);
void blinkLedRing(void);

/*
AlarmArray
0: toggle
1: failsafe
2: noGPS
3: beeperOn
4: pMeter
5: runtime
6: vBat
7: confirmation
8: Acc
9: I2C Error
*/
enum alrm_fac {
  ALRM_FAC_TOGGLE = 0,
  ALRM_FAC_FAILSAFE,
  ALRM_FAC_GPS,
  ALRM_FAC_BEEPERON,
  //ALRM_FAC_PMETER,
  ALRM_FAC_RUNTIME,
  ALRM_FAC_VBAT,
  ALRM_FAC_CONFIRM,
  ALRM_FAC_ACC,
  ALRM_FAC_I2CERROR,
  ALRM_FAC_SIZE, // MUST be LAST - used for size of array alarmArray
 };


/*
Resources:
0: onboard LED
*/
enum alrm_res {
  ALRM_RES_LED = 0,
  ALRM_RES_ANY    ,
};

enum alrm_lvl_onoff {
 ALRM_LVL_OFF = 0,
 ALRM_LVL_ON = 1,
};
enum alrm_lvl_failsafe {
 ALRM_LVL_FAILSAFE_FINDME = 1,
 ALRM_LVL_FAILSAFE_PANIC,
 };
enum alrm_lvl_toggle {
 ALRM_LVL_TOGGLE_1  = 1,
 ALRM_LVL_TOGGLE_2    ,
 ALRM_LVL_TOGGLE_ELSE ,
};
#if GPS
  enum alrm_lvl_gps {
   ALRM_LVL_GPS_NOFIX  = 2,
  };
#endif
#ifdef VBAT
  enum alrm_lvl_vbat {
   ALRM_LVL_VBAT_INFO  = 1,
   ALRM_LVL_VBAT_WARN ,
   ALRM_LVL_VBAT_CRIT ,
  };
#endif
enum alrm_lvl_confirm {
 ALRM_LVL_CONFIRM_1 = 1,
 ALRM_LVL_CONFIRM_2    ,
 ALRM_LVL_CONFIRM_ELSE ,
};

#define SET_ALARM(fac, level) alarmArray[fac] = level

#define IS_ALARM_SET(fac, level) ( alarmArray[fac] == level )

#endif /* ALARMS_H_ */
