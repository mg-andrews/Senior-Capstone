////* Name, Date: Marissa Andrews, 4/8/2026
////Purpose: Separate home page for just BP components /

#include "ui_home_bp.h"
#include "bp_output.h"
#include "app_state.h"
#include "ui_diagnose.h"
#include "ui_tabs.h"
#include "ui_scroll.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

LV_FONT_DECLARE(lv_font_montserrat_20);
LV_FONT_DECLARE(lv_font_montserrat_12);

/* ---------------- GLOBALS (BP SIDE ONLY) ---------------- */
static lv_obj_t* dd_blood_pressure;
static lv_obj_t* ta_systolic;
static lv_obj_t* ta_diastolic;
static lv_obj_t* ta_bpm;
static lv_obj_t* dd_korotkoff;
static lv_obj_t* ta_ausc_gap;
static lv_obj_t* gap_row;

static lv_obj_t* kb = NULL;
static lv_obj_t* go_btn_global;
static lv_obj_t* overlay;

/* Tracks whether a preset (non-manual) BP option is active.
   When true, tapping SYS/DIA boxes will NOT open the keyboard. */
static bool bp_preset_active = false;

/* ---------------- KEYBOARD ---------------- */
static void keyboard_event_cb(lv_event_t* e)
{
    if (lv_event_get_code(e) == LV_EVENT_READY ||
        lv_event_get_code(e) == LV_EVENT_CANCEL)
    {
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
}

static void show_keyboard(lv_obj_t* ta)
{
    if (!kb)
    {
        kb = lv_keyboard_create(lv_scr_act());
        lv_obj_set_size(kb, lv_pct(100), 125);
        lv_obj_add_event_cb(kb, keyboard_event_cb, LV_EVENT_ALL, NULL);
    }

    lv_keyboard_set_textarea(kb, ta);
    lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
}

/* For SYS/DIA: only open the keyboard when manual mode is active. */
static void sys_dia_focus_cb(lv_event_t* e)
{
    if (bp_preset_active) return;   /* preset selected – block keyboard */
    show_keyboard(lv_event_get_target(e));
}

/* BPM and gap always allow keyboard. */
static void textarea_focus_cb(lv_event_t* e)
{
    show_keyboard(lv_event_get_target(e));
}

/* ---------------- VALIDATION ---------------- */
static bool inputs_valid(void)
{
    if (app_state.systolic > 250 || app_state.systolic < 30 || app_state.systolic < app_state.diastolic) return false;
    if (app_state.diastolic < 30 || app_state.diastolic > 250) return false;
    if (app_state.ausc_gap_length > 60) return false;
    if (app_state.bpm < 50 || app_state.bpm > 140) return false;
    return true;
}

static void update_submit_state(void)
{
    if (!go_btn_global) return;

    if (inputs_valid())
        lv_obj_clear_state(go_btn_global, LV_STATE_DISABLED);
    else
        lv_obj_add_state(go_btn_global, LV_STATE_DISABLED);
}

/* ---------------- LOCK / UNLOCK (called by bp_randomizer) ---------------- */
/*
 * When a preset is active we do NOT want the user typing in the SYS/DIA
 * boxes.  Rather than LV_STATE_DISABLED (which greys them out), we just
 * set the bp_preset_active flag so the focus callback blocks the keyboard.
 * The boxes still show their placeholder text and look normal.
 */
void ui_home_bp_lock_manual_inputs(bool locked)
{
    bp_preset_active = locked;

    /* If the keyboard is currently open for one of these fields, hide it. */
    if (locked && kb)
    {
        lv_obj_t* current_ta = lv_keyboard_get_textarea(kb);
        if (current_ta == ta_systolic || current_ta == ta_diastolic)
            lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
}

/* ---------------- BP INPUT CALLBACKS ---------------- */
static void bp_textarea_event_cb(lv_event_t* e)
{
    lv_obj_t* ta = lv_event_get_target(e);
    int value = atoi(lv_textarea_get_text(ta));

    if (ta == ta_systolic)       app_state.systolic = value;
    else if (ta == ta_diastolic) app_state.diastolic = value;

    diagnose_update();
    update_submit_state();
}

static void bpm_textarea_event_cb(lv_event_t* e)
{
    app_state.bpm = atoi(lv_textarea_get_text(lv_event_get_target(e)));
    diagnose_update();
    update_submit_state();
}

static void ausc_gap_textarea_event_cb(lv_event_t* e)
{
    app_state.ausc_gap_length = atoi(lv_textarea_get_text(lv_event_get_target(e)));
    diagnose_update();
    update_submit_state();
}

/* ---------------- DEFAULTS ---------------- */
static void set_default_bp_healthy(void)
{
    lv_textarea_set_text(ta_systolic, "110");
    lv_textarea_set_text(ta_diastolic, "65");
    app_state.systolic = 110;
    app_state.diastolic = 65;
}

static void set_default_bpm(void)
{
    lv_textarea_set_text(ta_bpm, "60");
    app_state.bpm = 60;
}

static void set_default_gap_value(void)
{
    lv_textarea_set_text(ta_ausc_gap, "20");
    app_state.ausc_gap_length = 20;
}

/* ---------------- DROPDOWN ---------------- */
static void dropdown_event_cb(lv_event_t* e)
{
    uint32_t selected = lv_dropdown_get_selected(lv_event_get_target(e));
    uint32_t index = (uint32_t)(uintptr_t)lv_event_get_user_data(e);

    if (index == 0)
    {
        /* Delegate entirely to bp_randomizer:
           - updates app_state.blood_pressure
           - randomizes sys/dia for presets
           - calls ui_home_bp_lock_manual_inputs()             */
        bp_handle_dropdown_selection((int)selected);

        /* For manual mode, restore the default healthy values
           into the text areas so the boxes are not empty.     */
        if (selected == 0)
            set_default_bp_healthy();
    }
    else if (index == 1)
    {
        app_state.korotkoff = selected;

        if (selected == 1)
        {
            lv_obj_clear_flag(gap_row, LV_OBJ_FLAG_HIDDEN);
            set_default_gap_value();
        }
        else
        {
            lv_obj_add_flag(gap_row, LV_OBJ_FLAG_HIDDEN);
            app_state.ausc_gap_length = 0;  /* reset so validation passes */
        }
    }

    diagnose_update();
    update_submit_state();
}

/* ---------------- SEND ---------------- */
void send_stop_signal(void)
{
    printf("STOP\n");
}

void send_app_state(void)
{
    printf("OXYGEN:%s\nBP:%s\nSYSTOLIC:%d\nDIASTOLIC:%d\nBPM:%d\nKorotkoff:%s\nAuscultatory Gap:%d\n",
        get_oxygen_string(),
        get_bp_string(),
        app_state.systolic,
        app_state.diastolic,
        app_state.bpm,
        get_korotkoff_string(),
        app_state.ausc_gap_length);
}

/* Allow other UI modules to register the submit button. */
void ui_home_set_submit_btn(lv_obj_t* btn)
{
    go_btn_global = btn;
    update_submit_state();
}

/* ---------------- UI ---------------- */
void ui_home_create(lv_obj_t* parent)
{
    lv_obj_t* cont = ui_create_scrollable_container(parent);

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_START);

    lv_obj_set_style_pad_all(cont, 20, 0);
    lv_obj_set_style_pad_row(cont, 20, 0);
    lv_obj_set_style_pad_right(cont, 60, 0);

    const char* labels[2] = {
        "Blood Pressure",
        "Korotkoff Sounds"
    };

    const char* options_list[2] = {
        "Manual Selection\nHealthy\nStage I\nStage II\nHypotensive",
        "Normal\nAuscultatory Gap"
    };

    for (int i = 0; i < 2; i++)
    {
        lv_obj_t* row = lv_obj_create(cont);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row,
            LV_FLEX_ALIGN_SPACE_BETWEEN,
            LV_FLEX_ALIGN_CENTER,
            LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_all(row, 5, 0);

        lv_obj_t* label = lv_label_create(row);
        lv_label_set_text(label, labels[i]);

        lv_obj_t* dd = lv_dropdown_create(row);
        lv_obj_set_width(dd, 175);
        lv_dropdown_set_dir(dd, LV_DIR_LEFT);
        lv_dropdown_set_options(dd, options_list[i]);
        lv_obj_add_event_cb(dd, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, (void*)(uintptr_t)i);

        if (i == 0)
        {
            dd_blood_pressure = dd;

            /* ---- Manual SYS / DIA row ---- */
            lv_obj_t* bp_row = lv_obj_create(cont);
            lv_obj_set_width(bp_row, lv_pct(100));
            lv_obj_set_height(bp_row, LV_SIZE_CONTENT);
            lv_obj_set_flex_flow(bp_row, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(bp_row,
                LV_FLEX_ALIGN_SPACE_BETWEEN,
                LV_FLEX_ALIGN_CENTER,
                LV_FLEX_ALIGN_CENTER);
            lv_obj_set_style_pad_all(bp_row, 5, 0);

            lv_obj_t* bp_label = lv_label_create(bp_row);
            lv_label_set_text(bp_label, "Manual BP (SYS/DIA)");

            ta_systolic = lv_textarea_create(bp_row);
            lv_obj_set_width(ta_systolic, 80);
            lv_textarea_set_placeholder_text(ta_systolic, "SYS");
            lv_textarea_set_one_line(ta_systolic, true);
            lv_textarea_set_accepted_chars(ta_systolic, "0123456789");
            lv_textarea_set_max_length(ta_systolic, 3);
            /* Use sys_dia_focus_cb so keyboard is blocked when a preset is active */
            lv_obj_add_event_cb(ta_systolic, sys_dia_focus_cb, LV_EVENT_FOCUSED, NULL);
            lv_obj_add_event_cb(ta_systolic, bp_textarea_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

            ta_diastolic = lv_textarea_create(bp_row);
            lv_obj_set_width(ta_diastolic, 80);
            lv_textarea_set_placeholder_text(ta_diastolic, "DIA");
            lv_textarea_set_one_line(ta_diastolic, true);
            lv_textarea_set_accepted_chars(ta_diastolic, "0123456789");
            lv_textarea_set_max_length(ta_diastolic, 3);
            lv_obj_add_event_cb(ta_diastolic, sys_dia_focus_cb, LV_EVENT_FOCUSED, NULL);
            lv_obj_add_event_cb(ta_diastolic, bp_textarea_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

            set_default_bp_healthy();

            /* ---- BPM row ---- */
            lv_obj_t* bpm_row = lv_obj_create(cont);
            lv_obj_set_width(bpm_row, lv_pct(100));
            lv_obj_set_height(bpm_row, LV_SIZE_CONTENT);
            lv_obj_set_flex_flow(bpm_row, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(bpm_row,
                LV_FLEX_ALIGN_SPACE_BETWEEN,
                LV_FLEX_ALIGN_CENTER,
                LV_FLEX_ALIGN_CENTER);
            lv_obj_set_style_pad_all(bpm_row, 5, 0);

            lv_obj_t* bpm_label = lv_label_create(bpm_row);
            lv_label_set_text(bpm_label, "Heart Rate (BPM)");

            ta_bpm = lv_textarea_create(bpm_row);
            lv_obj_set_width(ta_bpm, 100);
            lv_textarea_set_placeholder_text(ta_bpm, "BPM");
            lv_textarea_set_one_line(ta_bpm, true);
            lv_textarea_set_accepted_chars(ta_bpm, "0123456789");
            lv_textarea_set_max_length(ta_bpm, 3);
            lv_obj_add_event_cb(ta_bpm, textarea_focus_cb, LV_EVENT_FOCUSED, NULL);
            lv_obj_add_event_cb(ta_bpm, bpm_textarea_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

            set_default_bpm();
        }

        if (i == 1)
        {
            dd_korotkoff = dd;

            gap_row = lv_obj_create(cont);
            lv_obj_set_width(gap_row, lv_pct(100));
            lv_obj_set_height(gap_row, LV_SIZE_CONTENT);
            lv_obj_set_flex_flow(gap_row, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(gap_row,
                LV_FLEX_ALIGN_SPACE_BETWEEN,
                LV_FLEX_ALIGN_CENTER,
                LV_FLEX_ALIGN_CENTER);
            lv_obj_set_style_pad_all(gap_row, 5, 0);

            lv_obj_t* gap_label = lv_label_create(gap_row);
            lv_label_set_text(gap_label, "Gap Length (mmHg)");

            ta_ausc_gap = lv_textarea_create(gap_row);
            lv_obj_set_width(ta_ausc_gap, 100);
            lv_textarea_set_placeholder_text(ta_ausc_gap, "mmHg");
            lv_textarea_set_one_line(ta_ausc_gap, true);
            lv_textarea_set_accepted_chars(ta_ausc_gap, "0123456789");
            lv_textarea_set_max_length(ta_ausc_gap, 3);
            lv_obj_add_event_cb(ta_ausc_gap, textarea_focus_cb, LV_EVENT_FOCUSED, NULL);
            lv_obj_add_event_cb(ta_ausc_gap, ausc_gap_textarea_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

            lv_obj_add_flag(gap_row, LV_OBJ_FLAG_HIDDEN);
        }
    }

    overlay = lv_obj_create(parent);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_HIDDEN);

    ui_add_scroll_buttons(parent, cont);
}


