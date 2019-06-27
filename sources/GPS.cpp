#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "GPS.h"
#include "Serial.h"
#include "MwiiSSerial.h"
#include "Sensors.h"
#include "MultiWii.h"
#include "EEPROM.h"
#include <math.h>

#if GPS

//Function prototypes for other GPS functions
//These perhaps could go to the gps.h file, however these are local to the gps.cpp  
static void GPS_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* bearing);
static void GPS_distance_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist);
static void GPS_calc_velocity(void);
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng );
static void GPS_calc_poshold(void);
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow);
static void GPS_calc_nav_rate(uint16_t max_speed);
int32_t wrap_18000(int32_t ang);
static bool check_missed_wp(void);
void GPS_calc_longitude_scaling(int32_t lat);
static void GPS_update_crosstrack(void);
int32_t wrap_36000(int32_t ang);


// Leadig filter - TODO: rewrite to normal C instead of C++

// Set up gps lag
#define GPS_LAG 1.0f                          //We assumes that GPS has a 1 sec lag 

static int32_t  GPS_coord_lead[2];              // Lead filtered gps coordinates

class LeadFilter {
public:
    LeadFilter() :
        _last_velocity(0) {
    }

    // setup min and max radio values in CLI
    int32_t         get_position(int32_t pos, int16_t vel, float lag_in_seconds = 1.0);
    void            clear() { _last_velocity = 0; }

private:
    int16_t         _last_velocity;

};

int32_t LeadFilter::get_position(int32_t pos, int16_t vel, float lag_in_seconds)
{
    int16_t accel_contribution = (vel - _last_velocity) * lag_in_seconds * lag_in_seconds;
    int16_t vel_contribution = vel * lag_in_seconds;

    // store velocity for next iteration
    _last_velocity = vel;

    return pos + vel_contribution + accel_contribution;
}

LeadFilter xLeadFilter;      // Long GPS lag filter 
LeadFilter yLeadFilter;      // Lat  GPS lag filter 

typedef struct PID_PARAM_ {
  float kP;
  float kI;
  float kD;
  float Imax;
  } PID_PARAM;
  
PID_PARAM posholdPID_PARAM;
PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;

typedef struct PID_ {
  float   integrator; // integrator value
  int32_t last_input; // last input for derivative
  float   lastderivative; // last derivative for low-pass filter
  float   output;
  float   derivative;
} PID;
PID posholdPID[2];
PID poshold_ratePID[2];
PID navPID[2];

int32_t get_P(int32_t error, struct PID_PARAM_* pid) {
  return (float)error * pid->kP;
}

int32_t get_I(int32_t error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) {
  pid->integrator += ((float)error * pid_param->kI) * *dt;
  pid->integrator = constrain(pid->integrator,-pid_param->Imax,pid_param->Imax);
  return pid->integrator;
}
  
int32_t get_D(int32_t input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) { // dt in milliseconds
  pid->derivative = (input - pid->last_input) / *dt;

  /// Low pass filter cut frequency for derivative calculation.
  float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter = 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  // discrete low pass filter, cuts out the
  // high frequency noise that can drive the controller crazy
  pid->derivative = pid->lastderivative + (*dt / ( filter + *dt)) * (pid->derivative - pid->lastderivative);
  // update state
  pid->last_input = input;
  pid->lastderivative    = pid->derivative;
  // add in derivative component
  return pid_param->kD * pid->derivative;
}

void reset_PID(struct PID_* pid) {
  pid->integrator = 0;
  pid->last_input = 0;
  pid->lastderivative = 0;
}

#define _X 1
#define _Y 0

#define RADX100                    0.000174532925  

uint8_t land_detect;                 //Detect land (extern)
static uint32_t land_settle_timer;
uint8_t GPS_Frame;            // a valid GPS_Frame was detected, and data is ready for nav computation

static float  dTnav;            // Delta Time in milliseconds for navigation computations, updated with every good GPS read
static int16_t actual_speed[2] = {0,0};
static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t rate_error[2];
static int32_t error[2];

static int32_t GPS_WP[2];   //Currently used WP
static int32_t GPS_FROM[2]; //the pervious waypoint for precise track following
int32_t target_bearing;     // This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t original_target_bearing;  // deg * 100, The original angle to the next_WP when the next_WP was set, Also used to check when we pass a WP
static int16_t crosstrack_error;     // The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
uint32_t wp_distance;                // distance between plane and next_WP in cm
static uint16_t waypoint_speed_gov;  // used for slow speed wind up when start navigation;


////////////////////////////////////////////////////////////////////////////////////
// moving average filter variables
//

#define GPS_FILTER_VECTOR_LENGTH 5

static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
static uint16_t fraction3[2];

static int16_t nav_takeoff_bearing;  // saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home


//Main navigation processor and state engine
// TODO: add proceesing states to ease processing burden 
uint8_t GPS_Compute(void) {
  unsigned char axis;
  uint32_t dist;        //temp variable to store dist to copter
  int32_t  dir;         //temp variable to store dir to copter
  static uint32_t nav_loopTimer;

  //check that we have a valid frame, if not then return immediatly
  if (GPS_Frame == 0) return 0; else GPS_Frame = 0;

  //check home position and set it if it was not set
  if (f.GPS_FIX && GPS_numSat >= 5) {
    #if !defined(DONT_RESET_HOME_AT_ARM)
       if (!f.ARMED) {f.GPS_FIX_HOME = 0;}
    #endif
    if (!f.GPS_FIX_HOME && f.ARMED) {
      GPS_reset_home_position();
    }
    //Apply moving average filter to GPS data
    if (GPS_conf.filtering) {
      GPS_filter_index = (GPS_filter_index+1) % GPS_FILTER_VECTOR_LENGTH;
      for (axis = 0; axis< 2; axis++) {
        GPS_read[axis] = GPS_coord[axis]; //latest unfiltered data is in GPS_latitude and GPS_longitude
        GPS_degree[axis] = GPS_read[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t

        // How close we are to a degree line ? its the first three digits from the fractions of degree
        // later we use it to Check if we are close to a degree line, if yes, disable averaging,
        fraction3[axis] = (GPS_read[axis]- GPS_degree[axis]*10000000) / 10000;

        GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
        GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis]*10000000); 
        GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
        GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis]*10000000);
        if ( NAV_state == NAV_STATE_HOLD_INFINIT || NAV_state == NAV_STATE_HOLD_TIMED) {      //we use gps averaging only in poshold mode...
          if ( fraction3[axis]>1 && fraction3[axis]<999 ) GPS_coord[axis] = GPS_filtered[axis];
        }
      }
    }

    //dTnav calculation
    //Time for calculating x,y speed and navigation pids
    dTnav = (float)(millis() - nav_loopTimer)/ 1000.0;
    nav_loopTimer = millis();

    // prevent runup from bad GPS
    dTnav = min(dTnav, 1.0);  

    //calculate distance and bearings for gui and other stuff continously - From home to copter
    GPS_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dir);
    GPS_distance_cm(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dist);
    GPS_distanceToHome = dist/100;
    GPS_directionToHome = dir/100;

    if (!f.GPS_FIX_HOME) {     //If we don't have home set, do not display anything
      GPS_distanceToHome = 0;
      GPS_directionToHome = 0;
    }

    //Check fence setting and execute RTH if neccessary
    //TODO: autolanding
    if ((GPS_conf.fence > 0) && (GPS_conf.fence < GPS_distanceToHome) && (f.GPS_mode != GPS_MODE_RTH) ) {
      init_RTH();
    }

    //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
    GPS_calc_velocity();        

    //Navigation state engine
    if (f.GPS_mode != GPS_MODE_NONE) {   //ok we are navigating ###0002 
      //do gps nav calculations here, these are common for nav and poshold  
      GPS_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&target_bearing);
      if (GPS_conf.lead_filter) {
        GPS_distance_cm(&GPS_coord_lead[LAT],&GPS_coord_lead[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance);
        GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord_lead[LAT],&GPS_coord_lead[LON]);
      } else {
        GPS_distance_cm(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance);
        GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);
      }

      int16_t speed = 0;                   //Desired navigation speed

      switch(NAV_state)                    //Navigation state machine
        {
        case NAV_STATE_NONE:               //Just for clarity, do nothing when nav_state is none
          break;

        case NAV_STATE_LANDED:
          // Disarm if THROTTLE stick is at minimum or 5sec past after land detected
          if (rcData[THROTTLE]<MINCHECK || nav_timer_stop <= millis()) { //Throttle at minimum or 5sec passed.
            go_disarm();
            f.OK_TO_ARM = 0;                //Prevent rearming
            NAV_state = NAV_STATE_NONE;     //Disable position holding.... prevent flippover
            f.LAND_COMPLETED = 0;
            f.LAND_IN_PROGRESS = 0;
            land_detect = 0;
            f.THROTTLE_IGNORED = 0;
            GPS_reset_nav();
          }
          break;

        case NAV_STATE_HOLD_INFINIT:        //Constant position hold, no timer. Only an rcOption change can exit from this
          GPS_calc_poshold();
          break;

        case NAV_STATE_HOLD_TIMED:
          if (nav_timer_stop == 0) {                         //We are start a timed poshold
            nav_timer_stop = millis() + 1000*nav_hold_time;  //Set when we will continue
          } else if (nav_timer_stop <= millis()) {           //did we reach our time limit ?
            if (mission_step.flag != MISSION_FLAG_END) {
              NAV_state = NAV_STATE_PROCESS_NEXT;            //if yes then process next mission step
            }
            NAV_error = NAV_ERROR_TIMEWAIT;
          }
          GPS_calc_poshold();                                //BTW hold position till next command
          break;

        case NAV_STATE_RTH_START:
          GPS_set_next_wp(&GPS_home[LAT],&GPS_home[LON], &GPS_coord[LAT], &GPS_coord[LON]); //If we reached then change mode and start RTH
          NAV_state = NAV_STATE_RTH_ENROUTE;
          NAV_error = NAV_ERROR_NONE;
          break;

        case NAV_STATE_RTH_ENROUTE:                                                  //Doing RTH navigation
          speed = GPS_calc_desired_speed(GPS_conf.nav_speed_max, GPS_conf.slow_nav); 
          GPS_calc_nav_rate(speed);
          GPS_adjust_heading();
          if ((wp_distance <= GPS_conf.wp_radius) || check_missed_wp()) {            //if yes switch to poshold mode
            if (mission_step.parameter1 == 0) NAV_state = NAV_STATE_HOLD_INFINIT;
            else NAV_state = NAV_STATE_LANDED;
            if (GPS_conf.nav_rth_takeoff_heading) { magHold = nav_takeoff_bearing; }
          } 
          break;

        case NAV_STATE_WP_ENROUTE:
          speed = GPS_calc_desired_speed(GPS_conf.nav_speed_max, GPS_conf.slow_nav); 
          GPS_calc_nav_rate(speed);
          GPS_adjust_heading();

          if ((wp_distance <= GPS_conf.wp_radius) || check_missed_wp()) {               //This decides what happen when we reached the WP coordinates 
            if (mission_step.action == MISSION_LAND) {                                  //Autoland
              NAV_state = NAV_STATE_LANDED;
            } else if (mission_step.flag == MISSION_FLAG_END) {                         //If this was the last mission step (flag set by the mission planner), then switch to poshold
              NAV_state = NAV_STATE_HOLD_INFINIT;
              NAV_error = NAV_ERROR_FINISH;
            } else if (mission_step.action == MISSION_HOLD_UNLIM) {                     //If mission_step was POSHOLD_UNLIM and we reached the position then switch to poshold unlimited
              NAV_state = NAV_STATE_HOLD_INFINIT;
              NAV_error = NAV_ERROR_FINISH;
            } else if (mission_step.action == MISSION_HOLD_TIME) {                      //If mission_step was a timed poshold then initiate timed poshold
              nav_hold_time = mission_step.parameter1;
              nav_timer_stop = 0;                                                       //This indicates that we are starting a timed poshold
              NAV_state = NAV_STATE_HOLD_TIMED;
            } else {
              NAV_state = NAV_STATE_PROCESS_NEXT;                                       //Otherwise process next step
            }
          }
          break;

        case NAV_STATE_DO_JUMP:
          if (jump_times < 0) {                                  //Jump unconditionally (supposed to be -1) -10 should not be here
            next_step = mission_step.parameter1;
            NAV_state = NAV_STATE_PROCESS_NEXT;
          }
          if (jump_times == 0) {
            jump_times = -10;                                    //reset jump counter
            if (mission_step.flag == MISSION_FLAG_END) {         //If this was the last mission step (flag set by the mission planner), then switch to poshold
              NAV_state = NAV_STATE_HOLD_INFINIT;
              NAV_error = NAV_ERROR_FINISH;
            } else
              NAV_state = NAV_STATE_PROCESS_NEXT;
          }

          if (jump_times > 0) {                                  //if zero not reached do a jump
            next_step = mission_step.parameter1;
            NAV_state = NAV_STATE_PROCESS_NEXT;
            jump_times--;
          }
          break;

        case NAV_STATE_PROCESS_NEXT:                             //Processing next mission step
          NAV_error = NAV_ERROR_NONE;
          if (!recallWP(next_step)) { 
            abort_mission(NAV_ERROR_WP_CRC);
          } else {
            switch(mission_step.action)
              {
              //Waypoiny and hold commands all starts with an enroute status it includes the LAND command too
              case MISSION_WAYPOINT:
              case MISSION_HOLD_TIME:
              case MISSION_HOLD_UNLIM:
              case MISSION_LAND:
                GPS_set_next_wp(&mission_step.pos[LAT], &mission_step.pos[LON], &GPS_prev[LAT], &GPS_prev[LON]);
                if ((wp_distance/100) >= GPS_conf.safe_wp_distance)  abort_mission(NAV_ERROR_TOOFAR);
                else NAV_state = NAV_STATE_WP_ENROUTE;
                GPS_prev[LAT] = mission_step.pos[LAT];  //Save wp coordinates for precise route calc
                GPS_prev[LON] = mission_step.pos[LON];
                break;
              case MISSION_RTH:
                f.GPS_head_set = 0;
                NAV_state = NAV_STATE_RTH_START;
                break;
              case MISSION_JUMP:
                if (jump_times == -10) jump_times = mission_step.parameter2;
                if (mission_step.parameter1 > 0 && mission_step.parameter1 < mission_step.number)
                  NAV_state = NAV_STATE_DO_JUMP;
                else //Error situation, invalid jump target
                  abort_mission(NAV_ERROR_INVALID_JUMP);
                break;
              case MISSION_SET_HEADING:
                GPS_poi[LAT] = 0; GPS_poi[LON] = 0; // zeroing this out clears the possible pervious set_poi
                if (mission_step.parameter1 < 0) f.GPS_head_set = 0;
                else {
                  f.GPS_head_set = 1;
                  GPS_directionToPoi = mission_step.parameter1;
                } 
                break;
              case MISSION_SET_POI:
                GPS_poi[LAT] = mission_step.pos[LAT];
                GPS_poi[LON] = mission_step.pos[LON];
                f.GPS_head_set = 1;
                break;
              default:                                  //if we got an unknown action code abort mission and hold position
                abort_mission(NAV_ERROR_INVALID_DATA);
                break;
              }
            next_step++; //Prepare for the next step
          }
          break;
        } // switch end
    } //end of gps calcs ###0002 
  }
  return 1;
} // End of GPS_compute

// Abort current mission with the given error code (switch to poshold_infinit)
void abort_mission(unsigned char error_code) {
  GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON],&GPS_coord[LAT], &GPS_coord[LON]);
  NAV_error = error_code;
  NAV_state = NAV_STATE_HOLD_INFINIT;
}

//Adjusting heading according to settings - MAG mode must be enabled
void GPS_adjust_heading() {
  //TODO: Add slow windup for large heading change
  //This controls the heading
  if (f.GPS_head_set) { // We have seen a SET_POI or a SET_HEADING command
    if (GPS_poi[LAT] == 0)
      magHold = wrap_18000((GPS_directionToPoi*100))/100;
    else {
      GPS_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_poi[LAT],&GPS_poi[LON],&GPS_directionToPoi);
      GPS_distance_cm(&GPS_coord[LAT],&GPS_coord[LON],&GPS_poi[LAT],&GPS_poi[LON],&wp_distance);
      magHold = GPS_directionToPoi /100;
    }
  } else {                                // heading controlled by the standard defines
    if (GPS_conf.nav_controls_heading) {
      if (GPS_conf.nav_tail_first) {
        magHold = wrap_18000(target_bearing-18000)/100;
      } else {
        magHold = wrap_18000(target_bearing)/100;
      }
    }
  }
}

#define LAND_DETECT_THRESHOLD 40      //Counts of land situation

void check_land() {
  f.LAND_COMPLETED = 1;
  land_detect = 0;
}

int32_t get_new_altitude() {
  return 0;
}

////////////////////////////////////////////////////////////////////////////////////
//PID based GPS navigation functions
//Author : EOSBandi
//Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
//Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

//original constraint does not work with variables
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat) {
  GPS_scaleLonDown = cos(lat * 1.0e-7f * 0.01745329251f);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t* lat_to, int32_t* lon_to, int32_t* lat_from, int32_t* lon_from) {
  GPS_WP[LAT] = *lat_to;
  GPS_WP[LON] = *lon_to;

  GPS_FROM[LAT] = *lat_from;
  GPS_FROM[LON] = *lon_from;

  GPS_calc_longitude_scaling(*lat_to);

  GPS_bearing(&GPS_FROM[LAT],&GPS_FROM[LON],&GPS_WP[LAT],&GPS_WP[LON],&target_bearing);
  GPS_distance_cm(&GPS_FROM[LAT],&GPS_FROM[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance);
  GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_FROM[LAT],&GPS_FROM[LON]);
  waypoint_speed_gov = GPS_conf.nav_speed_min;
  original_target_bearing = target_bearing;

}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp(void) {
  int32_t temp;
  temp = target_bearing - original_target_bearing;
  temp = wrap_18000(temp);
  return (abs(temp) > 10000);   // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision

void GPS_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* bearing) {
  int32_t off_x = *lon2 - *lon1;
  int32_t off_y = (*lat2 - *lat1) / GPS_scaleLonDown;

  *bearing = 9000 + atan2(-off_y, off_x) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

void GPS_distance_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist) {
  float dLat = (float)(*lat2 - *lat1);                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown; //x
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.11318845f;
}

//*******************************************************************************************************
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
// Note: even though the positions are projected using a lead filter, the velocities are calculated
//       from the unaltered gps locations.  We do not want noise from our lead filter affecting velocity
//*******************************************************************************************************
static void GPS_calc_velocity(void){
  static int16_t speed_old[2] = {0,0};
  static int32_t last[2] = {0,0};
  static uint8_t init = 0;

  if (init) {
    float tmp = 1.0/dTnav;
    actual_speed[_X] = (float)(GPS_coord[LON] - last[LON]) *  GPS_scaleLonDown * tmp;
    actual_speed[_Y] = (float)(GPS_coord[LAT]  - last[LAT])  * tmp;

    //TODO: Check unrealistic speed changes and signal navigation about posibble gps signal degradation
    if (!GPS_conf.lead_filter) {
      actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
      actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;

      speed_old[_X] = actual_speed[_X];
      speed_old[_Y] = actual_speed[_Y];
    }
  }
  init=1;

  last[LON] = GPS_coord[LON];
  last[LAT] = GPS_coord[LAT];

  if (GPS_conf.lead_filter) {
    GPS_coord_lead[LON] = xLeadFilter.get_position(GPS_coord[LON], actual_speed[_X], GPS_LAG);
    GPS_coord_lead[LAT] = yLeadFilter.get_position(GPS_coord[LAT], actual_speed[_Y], GPS_LAG);
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//   100  = 1m
//  1000  = 11m    = 36 feet
//  1800  = 19.80m = 60 feet
//  3000  = 33m
// 10000  = 111m
//
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng ) {
  error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;  // X Error
  error[LAT] = *target_lat - *gps_lat; // Y Error
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
// TODO: check that the poshold target speed constraint can be increased for snappier poshold lock
static void GPS_calc_poshold(void) {
  int32_t d;
  int32_t target_speed;
  uint8_t axis;
  
  for (axis=0;axis<2;axis++) {
    target_speed = get_P(error[axis], &posholdPID_PARAM); // calculate desired speed from lat/lon error
    target_speed = constrain(target_speed,-100,100);      // Constrain the target speed in poshold mode to 1m/s it helps avoid runaways..
    rate_error[axis] = target_speed - actual_speed[axis]; // calc the speed error

    nav[axis]      =
        get_P(rate_error[axis],                                               &poshold_ratePID_PARAM)
       +get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

    d = get_D(error[axis],                    &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

    d = constrain(d, -2000, 2000);

    // get rid of noise
    if(abs(actual_speed[axis]) < 50) d = 0;

    nav[axis] +=d;
    // nav[axis]  = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    nav[axis]  = constrain_int16(nav[axis], -GPS_conf.nav_bank_max, GPS_conf.nav_bank_max);
    navPID[axis].integrator = poshold_ratePID[axis].integrator;
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH and WP
//
static void GPS_calc_nav_rate( uint16_t max_speed) {
  float trig[2];
  int32_t target_speed[2];
  int32_t tilt;
  uint8_t axis;

  GPS_update_crosstrack();
  int16_t cross_speed = crosstrack_error * (GPS_conf.crosstrack_gain / 100.0);  //check is it ok ?
  cross_speed = constrain(cross_speed,-200,200);
  cross_speed = -cross_speed;

  float temp = (9000l - target_bearing) * RADX100;
  trig[_X] = cos(temp);
  trig[_Y] = sin(temp);

  target_speed[_X] = max_speed * trig[_X] - cross_speed * trig[_Y];
  target_speed[_Y] = cross_speed * trig[_X] + max_speed * trig[_Y];

  for (axis=0;axis<2;axis++) {
    rate_error[axis] = target_speed[axis] - actual_speed[axis];
    rate_error[axis] = constrain(rate_error[axis],-1000,1000);
    nav[axis]      =
        get_P(rate_error[axis],                        &navPID_PARAM)
       +get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM)
       +get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

    // nav[axis] = constrain(nav[axis],-NAV_BANK_MAX,NAV_BANK_MAX);
    nav[axis]  = constrain_int16(nav[axis], -GPS_conf.nav_bank_max, GPS_conf.nav_bank_max);
    poshold_ratePID[axis].integrator = navPID[axis].integrator;
  }
}

static void GPS_update_crosstrack(void) {
  // Crosstrack Error
  // ----------------
  // If we are too far off or too close we don't do track following
  float temp = (target_bearing - original_target_bearing) * RADX100;
  crosstrack_error = sin(temp) * wp_distance; // Meters we are off track line
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow 
// speed rampup when starting a navigation
//
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
//                 |                                        +|+
//                 |< we should slow to 1 m/s as we hit the target
//
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow) {
  if(_slow){
    max_speed = min(max_speed, wp_distance / 2);
  } else {
    max_speed = min(max_speed, wp_distance);
    max_speed = max(max_speed, GPS_conf.nav_speed_min);  // go at least nav_speed_min
  }
  // limit the ramp up of the speed
  // waypoint_speed_gov is reset to 0 at each new WP command
  if(max_speed > waypoint_speed_gov){
    waypoint_speed_gov += (int)(100.0 * dTnav); // increase at .5/ms
    max_speed = waypoint_speed_gov;
  }
  return max_speed;
}

////////////////////////////////////////////////////////////////////////////////////
// Utilities
//

int32_t wrap_36000(int32_t ang) {
  if (ang > 36000) ang -= 36000;
  if (ang < 0)     ang += 36000;
  return ang;
}


/*
 * EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
 * with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased 
 * resolution also increased precision of nav calculations
*/

#define DIGIT_TO_VAL(_x)        (_x - '0')
uint32_t GPS_coord_to_degrees(char* s) {
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;
  uint8_t i;

  // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++) ;
  q = s;

  // convert degrees
  while ((p - q) > 2) {
    if (deg)
      deg *= 10;
    deg += DIGIT_TO_VAL(*q++);
  }
  // convert minutes
  while (p > q) {
    if (min)
      min *= 10;
    min += DIGIT_TO_VAL(*q++);
  }
  // convert fractional minutes
  // expect up to four digits, result is in
  // ten-thousandths of a minute
  if (*p == '.') {
    q = p + 1;
    for (i = 0; i < 4; i++) {
      frac_min *= 10;
      if (isdigit(*q))
        frac_min += *q++ - '0';
    }
  }
  return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

// helper functions 
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;

  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0)   break;
      else  src[i+mult] = 0;
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9') tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
} 

//************************************************************************
// Common GPS functions 
//
void init_RTH() {
  f.GPS_mode = GPS_MODE_RTH;           // Set GPS_mode to RTH
  GPS_hold[LAT] = GPS_coord[LAT];      //All RTH starts with a poshold 
  GPS_hold[LON] = GPS_coord[LON];      //This allows to raise to rth altitude
  GPS_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON], &GPS_hold[LAT], &GPS_hold[LON]);
  NAV_paused_at = 0;
  f.GPS_head_set = 0;                                               //Allow the RTH ti handle heading
  NAV_state = NAV_STATE_RTH_START;                                  //NAV engine status is Starting RTH.
}

void GPS_reset_home_position(void) {
  if (f.GPS_FIX && GPS_numSat >= 5) {
    GPS_home[LAT] = GPS_coord[LAT];
    GPS_home[LON] = GPS_coord[LON];
    GPS_calc_longitude_scaling(GPS_coord[LAT]);    //need an initial value for distance and bearing calc
    nav_takeoff_bearing = att.heading;             //save takeoff heading
    //TODO: Set ground altitude
    f.GPS_FIX_HOME = 1;
  }
}

//reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav(void) {
  uint8_t i;

  for(i=0;i<2;i++) {
    nav[i] = 0;
    reset_PID(&posholdPID[i]);
    reset_PID(&poshold_ratePID[i]);
    reset_PID(&navPID[i]);
    NAV_state = NAV_STATE_NONE;
    //invalidate JUMP counter
    jump_times = -10;
    //reset next step counter
    next_step = 1;
    //Clear poi
    GPS_poi[LAT] = 0; GPS_poi[LON] = 0;
    f.GPS_head_set = 0;
  }
}

//Get the relevant P I D values and set the PID controllers 
void GPS_set_pids(void) {
  posholdPID_PARAM.kP   = (float)conf.pid[PIDPOS].P8/100.0;
  posholdPID_PARAM.kI   = (float)conf.pid[PIDPOS].I8/100.0;
  posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

  poshold_ratePID_PARAM.kP   = (float)conf.pid[PIDPOSR].P8/10.0;
  poshold_ratePID_PARAM.kI   = (float)conf.pid[PIDPOSR].I8/100.0;
  poshold_ratePID_PARAM.kD   = (float)conf.pid[PIDPOSR].D8/1000.0;
  poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

  navPID_PARAM.kP   = (float)conf.pid[PIDNAVR].P8/10.0;
  navPID_PARAM.kI   = (float)conf.pid[PIDNAVR].I8/100.0;
  navPID_PARAM.kD   = (float)conf.pid[PIDNAVR].D8/1000.0;
  navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
  }
//It was moved here since even i2cgps code needs it
int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
  return ang;
}


/**************************************************************************************/
/**************************************************************************************/
/***********************       specific  GPS device section  **************************/
/**************************************************************************************/
/**************************************************************************************/

/**************************************************************************************/
/***********************       NMEA                          **************************/
/**************************************************************************************/
#if defined(NMEA)
/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     - GPS altitude
     - GPS speed
*/
#define FRAME_GGA  1
#define FRAME_RMC  2

void GPS_SerialInit(void) {
  //sserial
  //SerialOpen(GPS_SERIAL,GPS_BAUD);
  SSerialOpen(GPS_BAUD);
  delay(1000);
}

bool GPS_newFrame(uint8_t c) {
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      if      (param == 2)                     {GPS_coord[LAT] = GPS_coord_to_degrees(string);}
      else if (param == 3 && string[0] == 'S') GPS_coord[LAT] = -GPS_coord[LAT];
      else if (param == 4)                     {GPS_coord[LON] = GPS_coord_to_degrees(string);}
      else if (param == 5 && string[0] == 'W') GPS_coord[LON] = -GPS_coord[LON];
      else if (param == 6)                     {f.GPS_FIX = (string[0]  > '0');}
      else if (param == 7)                     {GPS_numSat = grab_fields(string,0);}
      //else if (param == 9)                     {GPS_altitude = grab_fields(string,0);}  // altitude in meters added by Mis
      else if (param == 9)                     {;}
    } else if (frame == FRAME_RMC) {
      if      (param == 7)                     {GPS_speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
      else if (param == 8)                     {GPS_ground_course = grab_fields(string,1); }                 //ground course deg*10 
    }
    param++; offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    if (checksum_param) { //parity checksum
      uint8_t checksum = hex_c(string[0]);
      checksum <<= 4;
      checksum += hex_c(string[1]);
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
  } else {
     if (offset < 15) string[offset++] = c;
     if (!checksum_param) parity ^= c;
  }
  return frameOK && (frame==FRAME_GGA);
}
#endif //NMEA

//#endif

#endif // GPS Defined
