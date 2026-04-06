// Author: Marissa Andrews
// Date: 03/06/2026
#ifndef SPO2_SIM_H
#define SPO2_SIM_H

#include <stdbool.h>
#include "app_state.h"

typedef enum {
    STATE_HEALTHY,
    STATE_SLIGHTLY_BELOW_NORMAL,
    STATE_HYPOXIA,
    STATE_SCENARIO1,
    STATE_SCENARIO2,
    STATE_SCENARIO3,
    STATE_SCENARIO4
} PatientState;

typedef struct {
    float mean;
    float std;
    float min;
    float max;
} BiasParams;

typedef enum {
    NORMAL,
    LOW_TEMP
} TEMP;

typedef struct {
    int observed_spo2;
    float skin_bias;
    float temp_bias;
} SpO2Result;

PatientState get_patient_state(app_state_t app_state);

float get_true_sao2(PatientState state);

float truncated_gaussian(
    float mean,
    float std,
    float min_val,
    float max_val
);

float get_skin_tone_bias(
    PatientState state,
    int fitz_value
);

int get_fitzpatrick_value(app_state_t app_state);

float get_temp_bias(bool low_temp);

TEMP is_low_temp(app_state_t app_state);


SpO2Result calculate_spo2(
    PatientState state,
    bool low_temp,
    int fitz_value
);

#endif