// Name, Date: Marissa Andrews, 3/20/2026
//Purpose: Make pulse ox display screen

/* Include header file */
#include "pulse_ox.h"
#include <stdio.h>
#include "esp_log.h"
#include "lvgl.h"
#include "app_state.h"
#include "spo2_calc.h"

// define
static lv_obj_t* label_spo2;
static lv_obj_t* label_hr;
SpO2Result oxsat_data_update;

// text buffer
static char buf[64];

// declare fonts
LV_FONT_DECLARE(lv_font_montserrat_30);
//LV_FONT_DECLARE(lv_font_montserrat_10);

void pulse_ox_init(lv_obj_t* parent)
{
 
    if (parent == NULL) return;

    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);

    // spo2 label
    label_spo2 = lv_label_create(parent);
    lv_obj_set_style_text_color(label_spo2, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(label_spo2, &lv_font_montserrat_30, 0);
    lv_obj_align(label_spo2, LV_ALIGN_TOP_LEFT, 10, 20);

    // heart rate
    label_hr = lv_label_create(parent);
    lv_obj_set_style_text_color(label_hr, lv_color_hex(0x00FFFF), 0);
    lv_obj_set_style_text_font(label_hr, &lv_font_montserrat_30, 0);
    lv_obj_align(label_hr, LV_ALIGN_TOP_LEFT, 10, 100);

    //// bp label
    //label_bp = lv_label_create(parent);
    //lv_obj_set_style_text_color(label_bp, lv_color_hex(0xFFFFFF), 0);
    //lv_obj_set_style_text_font(label_bp, &lv_font_montserrat_12, 0);
    //lv_obj_align(label_bp, LV_ALIGN_TOP_LEFT, 10, 70);

    /* Default values for testing */
    //pulse_ox_update(98, 72);
}

void pulse_ox_update(uint8_t spo2, uint8_t heart_rate)
{
    if (label_spo2 == NULL || label_hr == NULL) return;

    //char* spo2_val = spo2.to_string().c_str();
    //char* hr_val = heart_rate.to_string().c_str();
    
    // spo2
    snprintf(buf, sizeof(buf), "SpO2\n%d%%", spo2);
    lv_label_set_text_fmt(label_spo2, "SpO2\n%d%%", spo2);
    //lv_label_set_text(label_spo2, buf);

    // heart rate
    snprintf(buf, sizeof(buf), "HR\n%d bpm", heart_rate);
    lv_label_set_text_fmt(label_hr, "HR\n%d bpm", heart_rate);
    //lv_label_set_text(label_hr, buf);

    ESP_LOGI("UI_PULSE", "Updated SpO2: %d%%, HR: %d bpm", spo2, heart_rate);

    // This tells the LVGL timer: "This specific area MUST be redrawn"
    lv_obj_invalidate(label_spo2);
    lv_obj_invalidate(label_hr);

    // blood pressure
    //snprintf(buf, sizeof(buf), "BP\n%d/%dmmHg", systolic, diastolic);
    //lv_label_set_text(label_bp, buf);
}



void pulse_ox_create(lv_obj_t *parent) {
    if (parent == NULL) {
        //ESP_LOGE("UI_PULSE", "Cannot create Pulse Ox UI: Display handle is NULL");
        return;
    }

    lv_obj_t *pulseox_cont = lv_obj_create(parent);

    // 2. Create the Main Container on the active screen of Screen 1

    lv_obj_set_size(pulseox_cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(pulseox_cont, lv_color_black(), 0);
    lv_obj_set_style_border_width(pulseox_cont, 0, 0);
    lv_obj_clear_flag(pulseox_cont, LV_OBJ_FLAG_SCROLLABLE);
    
    //ESP_LOGI("UI_PULSE", "Pulse Ox UI created on display %p", disp);
}
