// Author: Marissa Andrews
// Date: 03/06/2026
#ifndef SPO2_SIM_H
#define SPO2_SIM_H

#include <stdbool.h>

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

typedef struct {
    int observed_spo2;
    float skin_bias;
    float temp_bias;
} SpO2Result;

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

float get_temp_bias(bool low_temp);

SpO2Result calculate_spo2(
    float true_sao2,
    PatientState state,
    bool low_temp,
    int fitz_value
);

#endif