#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Sensors.h"
#include "Alarms.h"

void alarmPatternComposer();
void patternDecode(uint8_t resource,uint16_t first,uint16_t second,uint16_t third,uint16_t cyclepause, uint16_t endpause);
void setTiming(uint8_t resource, uint16_t pulse, uint16_t pause);
void turnOff(uint8_t resource);
void toggleResource(uint8_t resource, uint8_t activate);
void inline switch_landing_lights(uint8_t on);

static uint8_t cycleDone[5]={0,0,0,0,0}, 
               resourceIsOn[5] = {0,0,0,0,0};
static uint32_t LastToggleTime[5] ={0,0,0,0,0};
static int16_t  i2c_errors_count_old = 0;

static uint8_t SequenceActive[5]={0,0,0,0,0};

/********************************************************************/
/****                      Alarm Handling                        ****/
/********************************************************************/
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
/*
Resources:
0: onboard LED
*/
void alarmHandler(void){
  
  #if defined(RCOPTIONSBEEP)
    static uint8_t i = 0,firstrun = 1, last_rcOptions[CHECKBOXITEMS];
                  
    if (last_rcOptions[i] != rcOptions[i])   alarmArray[ALRM_FAC_TOGGLE] = ALRM_LVL_TOGGLE_1;
      last_rcOptions[i] = rcOptions[i]; 
      i++;
    if(i >= CHECKBOXITEMS)i=0;
    
    if(firstrun == 1 && alarmArray[ALRM_FAC_CONFIRM] == ALRM_LVL_OFF) {
      alarmArray[ALRM_FAC_TOGGLE] = ALRM_LVL_OFF;    //only enable options beep AFTER gyro init
      alarmArray[ALRM_FAC_BEEPERON] = ALRM_LVL_OFF;
    }        
    else firstrun = 0;
  #endif  
     
  #if defined(FAILSAFE)
    if ( failsafeCnt > (5*FAILSAFE_DELAY) && f.ARMED) {
      alarmArray[ALRM_FAC_FAILSAFE] = ALRM_LVL_FAILSAFE_PANIC;                                                                   //set failsafe warning level to 1 while landing
      if (failsafeCnt > 5*(FAILSAFE_DELAY+FAILSAFE_OFF_DELAY)) alarmArray[ALRM_FAC_FAILSAFE] = ALRM_LVL_FAILSAFE_FINDME;          //start "find me" signal after landing
    }
    if ( failsafeCnt > (5*FAILSAFE_DELAY) && !f.ARMED) alarmArray[ALRM_FAC_FAILSAFE] = ALRM_LVL_FAILSAFE_FINDME;                  // tx turned off while motors are off: start "find me" signal
    if ( failsafeCnt == 0) alarmArray[ALRM_FAC_FAILSAFE] = ALRM_LVL_OFF;                                              // turn off alarm if TX is okay
  #endif
  
  #if GPS
    if ((f.GPS_mode != GPS_MODE_NONE) && !f.GPS_FIX) alarmArray[ALRM_FAC_GPS] = ALRM_LVL_GPS_NOFIX;
    else if (!f.GPS_FIX) alarmArray[ALRM_FAC_GPS] = ALRM_LVL_ON;
    else alarmArray[ALRM_FAC_GPS] = ALRM_LVL_OFF;
  #endif
  
  #if defined(VBAT)
    if (vbatMin < conf.vbatlevel_crit) alarmArray[ALRM_FAC_VBAT] = ALRM_LVL_VBAT_CRIT;
    else if ( (analog.vbat > conf.vbatlevel_warn1)  || (NO_VBAT > analog.vbat)) alarmArray[ALRM_FAC_VBAT] = ALRM_LVL_OFF;
    else if (analog.vbat > conf.vbatlevel_warn2) alarmArray[ALRM_FAC_VBAT] = ALRM_LVL_VBAT_INFO;
    else if (analog.vbat > conf.vbatlevel_crit) alarmArray[ALRM_FAC_VBAT] = ALRM_LVL_VBAT_WARN;
    //else alarmArray[6] = 4;
  #endif
  
  if (i2c_errors_count > i2c_errors_count_old+100 || i2c_errors_count < -1) alarmArray[ALRM_FAC_I2CERROR] = ALRM_LVL_ON;
  else alarmArray[ALRM_FAC_I2CERROR] = ALRM_LVL_OFF;
  alarmPatternComposer();
}

void alarmPatternComposer(){ 
  //static char resource = 0;
  // patternDecode(length1,length2,length3,beeppause,endpause,loop)
}

void patternDecode(uint8_t resource,uint16_t first,uint16_t second,uint16_t third,uint16_t cyclepause, uint16_t endpause){
  static uint16_t pattern[5][5];
  static uint8_t icnt[5] = {0,0,0,0,0};
  
  if(SequenceActive[resource] == 0){
    SequenceActive[resource] = 1; 
    pattern[resource][0] = first; 
    pattern[resource][1] = second;
    pattern[resource][2] = third;
    pattern[resource][3] = endpause;
    pattern[resource][4] = cyclepause;
  }
  if(icnt[resource] <3 ){
    if (pattern[resource][icnt[resource]] != 0){
      setTiming(resource,pattern[resource][icnt[resource]],pattern[resource][4]);
     }
  }
  else if (LastToggleTime[resource] < (millis()-pattern[resource][3]))  {  //sequence is over: reset everything
    icnt[resource]=0;
    SequenceActive[resource] = 0;                               //sequence is now done, cycleDone sequence may begin
    alarmArray[ALRM_FAC_TOGGLE] = ALRM_LVL_OFF;                                //reset toggle bit
    alarmArray[ALRM_FAC_CONFIRM] = ALRM_LVL_OFF;                                //reset confirmation bit
    turnOff(resource);   
    return;
  }
  if (cycleDone[resource] == 1 || pattern[resource][icnt[resource]] == 0) {            //single on off cycle is done
    if (icnt[resource] < 3) {
      icnt[resource]++;
    }
    cycleDone[resource] = 0;
    turnOff(resource);    
  }  
}

void turnOff(uint8_t resource){
  if (resource == 1) {
    if (resourceIsOn[1]) {
      resourceIsOn[1] = 0;
    }
  }else if (resource == 0) {
    if (resourceIsOn[0]) {
      resourceIsOn[0] = 0;
      LEDPIN_OFF;
    }
  }else if (resource == 2) {
    if (resourceIsOn[2]) {
      resourceIsOn[2] = 0;
    }
  }else if (resource == 3) {
    if (resourceIsOn[3]) {
      resourceIsOn[3] = 0;
    }
  }else if (resource == 4) {
    if (resourceIsOn[4]) {
      resourceIsOn[4] = 0;
    }
  }
}

/********************************************************************/
/****                         LED Handling                       ****/
/********************************************************************/
//Beware!! Is working with delays, do not use inflight!

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      LEDPIN_TOGGLE; // switch LEDPIN state
      delay(ontime);
    }
    delay(60); //wait 60 ms
  }
}

/********************************************************************/
/****                   Global Resource Handling                 ****/
/********************************************************************/

  void setTiming(uint8_t resource, uint16_t pulse, uint16_t pause){
    if (!resourceIsOn[resource] && (millis() >= (LastToggleTime[resource] + pause))&& pulse != 0) {
      resourceIsOn[resource] = 1;      
      toggleResource(resource,1);
      LastToggleTime[resource]=millis();      
    } else if ( (resourceIsOn[resource] && (millis() >= LastToggleTime[resource] + pulse) ) || (pulse==0 && resourceIsOn[resource]) ) {
      resourceIsOn[resource] = 0;
      toggleResource(resource,0);
      LastToggleTime[resource]=millis();
      cycleDone[resource] = 1;     
    } 
  } 
 
  void toggleResource(uint8_t resource, uint8_t activate){
     switch(resource) {
        case 0:
        default:
          if (activate == 1) {LEDPIN_ON;}
          else LEDPIN_OFF;
          break;
      }
      return;
  }
