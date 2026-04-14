/* Name, Date: Marissa Andrews, 4/14/2026
   Purpose: Randomize systolic and diastolic values within
            clinically defined ranges for each BP state, and
            lock the manual SYS/DIA text-area inputs whenever
            a preset (non-manual) dropdown option is active. */

#include "bp_output.h"
#include "app_state.h"
#include "ui_home_bp.h"   /* ui_home_bp_lock_manual_inputs() */
#include "ui_diagnose.h"  /* diagnose_update()                */
#include <stdlib.h>



typedef struct {
    int sys_min;
    int sys_max;
    int dia_min;
    int dia_max;
} bp_range_t;

/* Index 0 is unused (Manual Selection) kept so array index
   matches the dropdown's selected value directly.              */
static const bp_range_t bp_ranges[] = {
    { 0,   0,   0,   0  },   /* 0 Manual Selection (not used) */
    { 100, 119, 60,  79  },  /* 1 Healthy                     */
    { 130, 139, 80,  89  },  /* 2 Stage I Hypertensive        */
    { 140, 179, 90,  119 },  /* 3 Stage II Hypertensive       */
    { 60,  89,  40,  59  },  /* 4 Hypotensive                 */
};

#define BP_RANGE_COUNT  ((int)(sizeof(bp_ranges) / sizeof(bp_ranges[0])))

/* ---------------------------------------------------------------
 * bp_randomize_for_state()
 *
 * Picks a random systolic and diastolic value within the range
 * defined for `state` (1-based dropdown index) and writes them
 * into app_state.  Calls diagnose_update() so the Scenario tab
 * reflects the new values immediately.
 *
 * Returns false and does nothing if state == 0 (Manual) or if
 * the index is out of range.
 * --------------------------------------------------------------- */
bool bp_randomize_for_state(int state)
{
    if (state <= 0 || state >= BP_RANGE_COUNT)
        return false;

    const bp_range_t* r = &bp_ranges[state];

    int sys_range = r->sys_max - r->sys_min;
    int dia_range = r->dia_max - r->dia_min;

    app_state.systolic = r->sys_min + (sys_range > 0 ? rand() % (sys_range + 1) : 0);
    app_state.diastolic = r->dia_min + (dia_range > 0 ? rand() % (dia_range + 1) : 0);

    /* Guard: diastolic must always be strictly less than systolic.
       In practice the ranges above already guarantee this, but
       this clamp is a safety net.                               */
    if (app_state.diastolic >= app_state.systolic)
        app_state.diastolic = app_state.systolic - 5;

    /* Refresh the Scenario / Diagnose tab */
    diagnose_update();

    return true;
}

/* ---------------------------------------------------------------
 * bp_handle_dropdown_selection()
 *
 * Call this from the blood-pressure dropdown event callback
 * (dropdown_event_cb in ui_home_bp.c) instead of the old
 * fixed-value assignment block.
 *
 *  selected == 0  ?  Manual: unlock inputs, leave values alone.
 *  selected >= 1  ?  Preset: randomize values, lock inputs.
 * --------------------------------------------------------------- */
void bp_handle_dropdown_selection(int selected)
{
    app_state.blood_pressure = selected;

    if (selected == 0)
    {
        /* Manual mode give the user back the text areas */
        ui_home_bp_lock_manual_inputs(false);
    }
    else
    {
        /* Preset mode randomize then lock */
        bp_randomize_for_state(selected);
        ui_home_bp_lock_manual_inputs(true);
    }
}
