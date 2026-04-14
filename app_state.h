#ifndef APP_STATE_H
#define APP_STATE_H

#include "lvgl.h"

typedef struct {
    int oxygen;
    int blood_pressure;
    int fitzpatrick;
    int temperature;

    int systolic;
    int diastolic;

    int korotkoff;
    int ausc_gap_length;   // in mmHg

    bool manual_bp;

    int bpm; // Added for heart rate

} app_state_t;

extern app_state_t app_state;

const char* get_oxygen_string(void);
const char* get_bp_string(void);
const char* get_fitz_string(void);
const char* get_temp_string(void);
const char* get_korotkoff_string(void);

#endif




//#ifndef APP_STATE_H
//#define APP_STATE_H
///* Include the lvgl.h for outside ui folder */
//#include "../lvgl/lvgl.h"
//// Define functions from app_state.c
//
//
//
//typedef struct {
//    int scenario;
//    int oxygen;
//    int blood_pressure;
//    int fitzpatrick;
//    int temperature;
//
//    int systolic;
//    int diastolic;
//    bool manual_bp;
//} app_state_t;
//
//extern app_state_t app_state;
//
///* New helper functions */
//const char* get_scenario_string(void);
//const char* get_oxygen_string(void);
//const char* get_bp_string(void);
//const char* get_fitz_string(void);
//const char* get_temp_string(void);
//#endif

