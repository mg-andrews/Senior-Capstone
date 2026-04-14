#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "display_driver.h"
#include "touch_driver.h"
#include "uart_send.h"
#include "ui_tabs.h"
#include "pulse_ox.h"
#include "app_state.h"
#include "spo2_calc.h"
#include "ui_home_os.h"
#include "ui_home_bp.h"
#define LV_USE_PERF_MONITOR 0

// TODO Update UI to use buttons to scroll instead of dragging, since dragging is hard to do on a resistive screen.
static const char *TAG = "main";

// Global or static handle
TaskHandle_t lvgl_task_handle = NULL;

// ── LVGL tick source ──────────────────────────────────────────
static void lv_tick_task(void *arg)
{
    lv_tick_inc(10);
}

static void lvgl_tick_init(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = &lv_tick_task,
        .name = "lv_tick"
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 10000)); // 10ms period
}

// ── LVGL handler task ─────────────────────────────────────────
static void lvgl_task(void *pvArg)
{
    while (1) {
        // Force Screen 2 CS HIGH here as a safety measure 
        // to ensure the bus is "clean" for the next LVGL cycle

        uint32_t time_till_next = lv_timer_handler();
        
        if (time_till_next < 10) time_till_next = 10;
        //if (time_till_next > 50) time_till_next = 50;

        // pdTRUE = clears notification count to 0 (acting like a binary semaphore)
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(time_till_next));

        if (notified > 0) {
            oxsat_data_update = calculate_spo2(get_patient_state(app_state), 
                                                 is_low_temp(app_state), 
                                                 get_fitzpatrick_value(app_state));
            
            pulse_ox_update(oxsat_data_update.observed_spo2, app_state.bpm);
            ESP_LOGI("UI_UPDATE", "UI refreshed via Notification");
        }
    }
}

/*
// ── LVGL handler task ─────────────────────────────────────────
static void lvgl_task(void *pvArg)
{
    while (1) {
        // The return value of lv_timer_handler tells us how many ms 
        // until it needs to run again.
        uint32_t time_till_next = lv_timer_handler();
        
        // If it's less than 5ms, force at least 5ms delay to prevent WDT triggers
        if (time_till_next < 0) time_till_next = 1;
        
        vTaskDelay(pdMS_TO_TICKS(time_till_next));
    }
}
*/

/*
// Create screen orientation/calibration test
static void btn_event_cb(lv_event_t * e) {
    lv_obj_t * btn = lv_event_get_target(e);
    const char * label = (const char *)lv_event_get_user_data(e);
    
    // Get the specific input device that triggered this
    lv_indev_t * indev = lv_indev_get_act();
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    
    ESP_LOGI("CALIB", "Hit %s at Pixel: %d, %d", label, p.x, p.y);
    
    // Change color to green to show it was successfully "hit"
    lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_GREEN), 0);
}

// Create screen orientation/calibration test buttons in each corner
void create_calibration_test(void) {
    static lv_style_t style_btn;
    lv_style_init(&style_btn);
    lv_style_set_bg_color(&style_btn, lv_palette_main(LV_PALETTE_RED));

    // Top-Left (Red)
    lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn1, 60, 60);
    lv_obj_align(btn1, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_style(btn1, &style_btn, 0);
    lv_obj_add_event_cb(btn1, btn_event_cb, LV_EVENT_PRESSED, (void*)"TOP-LEFT");

    // Top-Right (Red)
    lv_obj_t * btn2 = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn2, 60, 60);
    lv_obj_align(btn2, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_add_style(btn2, &style_btn, 0);
    lv_obj_add_event_cb(btn2, btn_event_cb, LV_EVENT_PRESSED, (void*)"TOP-RIGHT");

    // Bottom-Left (Red)
    lv_obj_t * btn3 = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn3, 60, 60);
    lv_obj_align(btn3, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_add_style(btn3, &style_btn, 0);
    lv_obj_add_event_cb(btn3, btn_event_cb, LV_EVENT_PRESSED, (void*)"BOTTOM-LEFT");

    // Bottom-Right (Red)
    lv_obj_t * btn4 = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn4, 60, 60);
    lv_obj_align(btn4, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_add_style(btn4, &style_btn, 0);
    lv_obj_add_event_cb(btn4, btn_event_cb, LV_EVENT_PRESSED, (void*)"BOTTOM-RIGHT");
}
*/

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    ESP_LOGI(TAG, "Starting LVGL System...");
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield to feed the watchdog

    setup_cs_pins();

    lv_init();
    lvgl_tick_init();


    // CRITICAL: Check if LVGL actually has a display now
    /*
    lv_disp_t * default_disp = lv_disp_get_default();
    if (default_disp == NULL) {
        ESP_LOGE(TAG, "CANNOT CREATE UI: No display registered!");
        return; // Stop here so we don't crash
    }
    ESP_LOGI(TAG, "Display registered. Screen resolution: %d x %d", 
             lv_disp_get_hor_res(default_disp), 
             lv_disp_get_ver_res(default_disp));
    */

    // Initialize displays and get their handles
    disp_handles_t screens = display_init();
    lv_disp_t *screen1 = screens.s1;
    lv_disp_t *screen2 = screens.s2;

    if (screen1 == NULL) {
        ESP_LOGE(TAG, "Failed to find screen 1! S1: %p, S2: %p", screen1, screen2);
        // Optional: return or handle partial failure
    } else if (screen2 == NULL) {
        ESP_LOGE(TAG, "Failed to find screen 2! S1: %p, S2: %p", screen1, screen2);
        // Optional: return or handle partial failure
    } else if (screen1 == NULL && screen2 == NULL) {
        ESP_LOGI(TAG, "Failed to find either screen! S1: %p, S2: %p", screen1, screen2);
    } else {
        ESP_LOGI(TAG, "Screen 1: %p, Screen 2: %p", screen1, screen2);
    }

    lv_disp_set_default(screen1);
    //touch_init(screen1);
    touch_init(screen1);
    ESP_LOGI(TAG, ">>> touch_init done");
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield to feed the watchdog

    ESP_LOGI(TAG, ">>> tick init done");
    uart_init();
    ESP_LOGI(TAG, ">>> uart init done");

    // Now it is safe to create the UI
    ESP_LOGI(TAG, "Creating UI Tabs...");
    ui_tabs_create();
    ESP_LOGI(TAG, ">>> ui_tabs_create done");

   // ONLY attempt Screen 2 if the registration actually worked
    if (screen2 != NULL) {
        // 1. Tell LVGL we are now working on Screen 2
        lv_disp_set_default(screen2);

        // 2. Create a "Main Container" that fills Screen 2
        // This ensures all subsequent objects belong to this display
        lv_obj_t *pulseox_cont = lv_obj_create(lv_scr_act()); 
        lv_obj_set_size(pulseox_cont, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_color(pulseox_cont, lv_color_black(), 0); // High contrast for testing
        lv_obj_set_style_border_width(pulseox_cont, 0, 0);

        // 3. Pass this specific container to your UI setup
        pulse_ox_init(pulseox_cont);
        SpO2Result oxsat_data = calculate_spo2(get_patient_state(app_state), is_low_temp(app_state), get_fitzpatrick_value(app_state));
        pulse_ox_update(oxsat_data.observed_spo2, app_state.bpm);

        ESP_LOGI(TAG, "S2 UI Created on Display: %p", screen2);
    } else {
        ESP_LOGW(TAG, "Skipping S2 UI creation because S2 is NULL");
    }

    lv_disp_set_default(screen1);

    // Screen calibration test (comment out ui_tabs_create() if you want to run this)
    //create_calibration_test();
    
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 32768, NULL, 5, &lvgl_task_handle, 1);
    ESP_LOGI(TAG, ">>> lvgl task created");
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        
    }
}