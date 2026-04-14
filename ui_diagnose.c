/* Name, Date: Marissa Andrews, 4/14/2026
   Purpose: Create layout and functionality for the diagnose page.
            Scenario information is only displayed after STOP is pressed.
            The display resets to a waiting message as soon as the user
            changes any input after a scenario has been shown. */

#include "ui_diagnose.h"
#include "app_state.h"
#include "ui_home_os.h"
#include "ui_home_bp.h"
#include <string.h>
#include <stdio.h>

            /* ---------------- GLOBALS ---------------- */
static lv_obj_t* result_label = NULL;
static lv_obj_t* submit_btn = NULL;
static lv_obj_t* stop_btn = NULL;
static lv_obj_t* overlay = NULL;

/* True once STOP has been pressed for the current run.
   The next call to diagnose_update() will clear the display,
   showing the user that their previous scenario is stale.   */
static bool scenario_dirty = false;

/* True while a scenario is actively running (SUBMIT pressed,
   STOP not yet pressed).  Blocks display updates mid-run.   */
static bool scenario_running = false;

/* ---------------- INTERNAL: BUILD BUFFER ---------------- */
/* Fills `buf` (size `bufsz`) with the current scenario text. */
static void build_scenario_text(char* buf, size_t bufsz)
{
    buf[0] = '\0';

    /* Oxygen */
    if (app_state.oxygen < 3) {
        char t[128];
        snprintf(t, sizeof(t), "Oxygen: %s\n", get_oxygen_string());
        strncat(buf, t, bufsz - strlen(buf) - 1);
    }

    /* BP selection + values */
    if (app_state.blood_pressure < 5) {
        char t[256];
        snprintf(t, sizeof(t),
            "BP Selection: %s\nSystolic: %d mmHg\nDiastolic: %d mmHg\nBPM: %d\n",
            get_bp_string(),
            app_state.systolic,
            app_state.diastolic,
            app_state.bpm);
        strncat(buf, t, bufsz - strlen(buf) - 1);
    }

    /* Korotkoff and gap */
    if (app_state.korotkoff < 2) {
        char t[64];
        snprintf(t, sizeof(t), "Korotkoff: %s\n", get_korotkoff_string());
        strncat(buf, t, bufsz - strlen(buf) - 1);
    }
    if (app_state.korotkoff == 1) {
        char t[64];
        snprintf(t, sizeof(t), "Auscultatory Gap: %d mmHg\n", app_state.ausc_gap_length);
        strncat(buf, t, bufsz - strlen(buf) - 1);
    }

    /* Fitzpatrick */
    if (app_state.fitzpatrick < 6) {
        char t[64];
        snprintf(t, sizeof(t), "Fitzpatrick: %d\n", app_state.fitzpatrick + 1);
        strncat(buf, t, bufsz - strlen(buf) - 1);
    }

    /* Temperature */
    if (app_state.temperature == 1)
        strncat(buf, "Patient Temperature: Low\n", bufsz - strlen(buf) - 1);

    if (strlen(buf) == 0)
        strncat(buf, "No selections made.", bufsz - strlen(buf) - 1);
}

/* ---------------- DIAGNOSE UPDATE (called on every input change) ---------------- */
void diagnose_update(void)
{
    if (result_label == NULL) return;

    if (scenario_running)
    {
        /* A run is in progress – don't touch the display at all. */
        return;
    }

    if (scenario_dirty)
    {
        /* User changed something after the last STOP.
           Clear the displayed scenario so they know it is stale. */
        lv_label_set_text(result_label, "The input selections will be shown once STOP is selected.");
        scenario_dirty = false;
        return;
    }

    /* Idle state (nothing has run yet, or was just reset):
       show a neutral waiting message rather than live values. */
    lv_label_set_text(result_label, "The input selections will be shown once STOP is selected.");
}

/* ---------------- DIAGNOSE SHOW (called only by STOP) ---------------- */
static void diagnose_show_results(void)
{
    if (result_label == NULL) return;

    static char buffer[1024];
    build_scenario_text(buffer, sizeof(buffer));
    lv_label_set_text(result_label, buffer);
}

/* ---------------- BUTTON LOGIC ---------------- */
static void submit_button_event_cb(lv_event_t* e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    scenario_running = true;
    scenario_dirty = false;

    /* Disable submit to prevent spam */
    lv_obj_add_state(submit_btn, LV_STATE_DISABLED);

    /* Show overlay */
    //lv_obj_clear_flag(overlay, LV_OBJ_FLAG_HIDDEN);

    /* Transmit */
    send_app_state();
}

static void stop_button_event_cb(lv_event_t* e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    /* Transmit stop */
    send_stop_signal();

    /* Reveal the scenario that was just run */
    diagnose_show_results();

    /* Mark dirty so the next input change clears the display */
    scenario_dirty = true;
    scenario_running = false;

    /* Re-enable submit */
    lv_obj_clear_state(submit_btn, LV_STATE_DISABLED);

    /* Hide overlay */
    //lv_obj_add_flag(overlay, LV_OBJ_FLAG_HIDDEN);
}

/* ---------------- UI CREATE ---------------- */
void ui_diagnose_create(lv_obj_t* parent)
{
    lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(parent, 20, 0);
    lv_obj_set_style_pad_row(parent, 20, 0);

    /* -------- Title -------- */
    lv_obj_t* title = lv_label_create(parent);
    lv_label_set_text(title, "Scenario Information");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);

    /* -------- Result Text -------- */
    result_label = lv_label_create(parent);
    lv_obj_set_width(result_label, 400);
    lv_label_set_long_mode(result_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(result_label, "Adjust inputs, then submit.");

    /* -------- BUTTON ROW -------- */
    lv_obj_t* btn_row = lv_obj_create(parent);
    lv_obj_set_width(btn_row, lv_pct(100));
    lv_obj_set_height(btn_row, LV_SIZE_CONTENT);

    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row,
        LV_FLEX_ALIGN_SPACE_EVENLY,
        LV_FLEX_ALIGN_CENTER,
        LV_FLEX_ALIGN_CENTER);

    lv_obj_set_style_pad_all(btn_row, 10, 0);

    /* -------- SUBMIT BUTTON -------- */
    submit_btn = lv_btn_create(btn_row);
    lv_obj_set_size(submit_btn, 120, 50);

    lv_obj_t* submit_label = lv_label_create(submit_btn);
    lv_label_set_text(submit_label, "SUBMIT");
    lv_obj_center(submit_label);

    lv_obj_add_event_cb(submit_btn, submit_button_event_cb, LV_EVENT_CLICKED, NULL);

    ui_home_set_submit_btn(submit_btn);

    /* -------- STOP BUTTON -------- */
    stop_btn = lv_btn_create(btn_row);
    lv_obj_set_size(stop_btn, 120, 50);

    lv_obj_t* stop_label = lv_label_create(stop_btn);
    lv_label_set_text(stop_label, "STOP");
    lv_obj_center(stop_label);

    lv_obj_add_event_cb(stop_btn, stop_button_event_cb, LV_EVENT_CLICKED, NULL);

    /* -------- OVERLAY -------- */
    //overlay = lv_obj_create(parent);
    //lv_obj_set_size(overlay, lv_pct(100), lv_pct(100));
    //lv_obj_center(overlay);

    //lv_obj_clear_flag(overlay, LV_OBJ_FLAG_CLICKABLE);
    //lv_obj_set_style_bg_opa(overlay, LV_OPA_50, 0);
    //lv_obj_add_flag(overlay, LV_OBJ_FLAG_HIDDEN);

    //lv_obj_t* overlay_label = lv_label_create(overlay);
    //lv_label_set_text(overlay_label, "Sending data...\nPress STOP to cancel");
    //lv_obj_center(overlay_label);
}
