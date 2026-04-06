/* Name, Date: Marissa Andrews, 3/20/2026 */
/* Purpose: Create header file pulse ox ui and define for values from microcontroller */

#pragma once
#ifndef PULSE_OX_H
#define PULSE_OX_H
/* Include the lvgl.h for outside ui folder */


#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include "spo2_calc.h"

    extern SpO2Result oxsat_data_update;

    // Initialize the pulse oximeter UI 
    void pulse_ox_init(lv_obj_t* parent);

    // Update values from microcontroller 
    void pulse_ox_update(uint8_t spo2, uint8_t heart_rate);

    void pulse_ox_create(lv_obj_t* parent);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PULSE_OX_H */





