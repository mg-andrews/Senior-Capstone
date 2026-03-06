// Author: Marissa Andrews
// Date: 03/06/2026
// Purpose: Convert the OS model from Python to C

#include "OS_model_v2_03_2026.h"
#include <stdlib.h>
#include <math.h>

static float rand_uniform(float min, float max)
{
    return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

/* Box-Muller Gaussian generator */
static float rand_gaussian(float mean, float std)
{
    float u1 = ((float)rand() + 1) / ((float)RAND_MAX + 1);
    float u2 = ((float)rand() + 1) / ((float)RAND_MAX + 1);

    float mag = std * sqrtf(-2.0f * logf(u1));
    float z0 = mag * cosf(2.0f * M_PI * u2);

    return mean + z0;
}

float truncated_gaussian(float mean, float std, float min_val, float max_val)
{
    float v;

    do {
        v = rand_gaussian(mean, std);
    } while (v < min_val || v > max_val);

    return v;
}

float get_true_sao2(PatientState state)
{
    switch(state)
    {
        case STATE_HEALTHY:
        case STATE_SCENARIO1:
        case STATE_SCENARIO2:
        case STATE_SCENARIO4:
            return rand_uniform(95.0f, 100.0f);

        case STATE_SLIGHTLY_BELOW_NORMAL:
            return rand_uniform(92.0f, 94.0f);

        case STATE_HYPOXIA:
        case STATE_SCENARIO3:
            return rand_uniform(80.0f, 91.0f);

        default:
            return 98.0f;
    }
}

static const BiasParams FITZ_HEALTHY[6] = {
    {0.625f,1.14f,-2.27f,3.40f},
    {0.568f,1.23f,-3.05f,3.60f},
    {0.570f,1.09f,-2.30f,3.40f},
    {0.400f,0.928f,-2.20f,2.65f},
    {0.996f,1.35f,-2.20f,4.50f},
    {1.16f,1.00f,-1.70f,3.70f}
};

static const BiasParams FITZ_SLIGHTLY[6] = {
    {0.981f,1.31f,-2.20f,1.86f},
    {1.40f,1.31f,-2.15f,4.40f},
    {1.18f,1.59f,-2.45f,5.70f},
    {0.797f,1.19f,-2.05f,3.80f},
    {1.37f,1.57f,-2.47f,5.05f},
    {1.75f,1.26f,-1.75f,4.90f}
};

static const BiasParams FITZ_HYPOXIA[6] = {
    {0.565f,1.52f,-3.60f,4.60f},
    {0.899f,1.64f,-3.32f,5.20f},
    {1.31f,1.60f,-2.90f,5.60f},
    {1.22f,1.55f,-3.0f,5.50f},
    {2.16f,2.10f,-3.80f,8.0f},
    {2.52f,1.99f,-2.96f,8.0f}
};

float get_skin_tone_bias(PatientState state, int fitz_value)
{
    if(fitz_value < 1 || fitz_value > 6)
        return 0.0f;

    int idx = fitz_value - 1;
    BiasParams params;

    switch(state)
    {
        case STATE_HEALTHY:
        case STATE_SCENARIO1:
        case STATE_SCENARIO2:
        case STATE_SCENARIO4:
            params = FITZ_HEALTHY[idx];
            break;

        case STATE_SLIGHTLY_BELOW_NORMAL:
            params = FITZ_SLIGHTLY[idx];
            break;

        case STATE_HYPOXIA:
        case STATE_SCENARIO3:
            params = FITZ_HYPOXIA[idx];
            break;

        default:
            return 0.0f;
    }

    return truncated_gaussian(params.mean, params.std, params.min, params.max);
}

float get_temp_bias(bool low_temp)
{
    if(!low_temp)
        return 0.0f;

    return truncated_gaussian(8.50f, 3.65f, 0.10f, 13.8f);
}

SpO2Result calculate_spo2(float true_sao2,
                          PatientState state,
                          bool low_temp,
                          int fitz_value)
{
    SpO2Result result;

    result.skin_bias = get_skin_tone_bias(state, fitz_value);
    result.temp_bias = get_temp_bias(low_temp);

    float observed = true_sao2 + result.skin_bias + result.temp_bias;

    if(observed < 0.0f)
        observed = 0.0f;
    if(observed > 100.0f)
        observed = 100.0f;

    result.observed_spo2 = (int)roundf(observed);

    return result;
}