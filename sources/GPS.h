#ifndef GPS_H_
#define GPS_H_

//Function prototypes for GPS frame parsing
bool GPS_newFrame(uint8_t c);
extern uint8_t GPS_Frame;            // a valid GPS_Frame was detected, and data is ready for nav computation

extern int32_t wrap_18000(int32_t ang);

void GPS_set_pids(void);
void GPS_SerialInit(void);
uint8_t GPS_Compute(void);
void GPS_reset_home_position(void);
void GPS_set_next_wp(int32_t* lat_to, int32_t* lon_to, int32_t* lat_from, int32_t* lon_from);
void GPS_reset_nav(void);

void abort_mission(unsigned char error_code);
void GPS_adjust_heading();
void init_RTH(void);

extern uint32_t wp_distance;
extern int32_t target_bearing;
#endif /* GPS_H_ */
