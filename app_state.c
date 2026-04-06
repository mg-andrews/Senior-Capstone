#include "app_state.h"

// Global state instance
app_state_t app_state = { 0 };

/* ── Lookup Tables (Synchronized with PC Simulator) ────────── */

static const char* scenario_strings[] = {
    "N/A",
    "Scenario 1",
    "Scenario 2",
    "Scenario 3",
    "Scenario 4"
};

static const char* oxygen_strings[] = {
    "Healthy",
    "Hypoxia",
    "Slightly below normal"
};

static const char* bp_strings[] = {
    "N/A",
    "Healthy",
    "Stage I Hypertensive",
    "Stage II Hypertensive",
    "Hypotensive"
};

static const char* fitz_strings[] = {
    "No Selection",
    "I",
    "II",
    "III",
    "IV",
    "V",
    "VI"
};

static const char* temp_strings[] = {
    "Normal",
    "Low Temperature"
};

// Added from PC Simulator
static const char* korotkoff_strings[] = {
    "Normal",
    "Auscultatory Gap"
};

/* ── Getter Functions ──────────────────────────────────────── */

const char* get_scenario_string(void)
{
    return scenario_strings[app_state.scenario];
}

const char* get_oxygen_string(void)
{
    return oxygen_strings[app_state.oxygen];
}

const char* get_bp_string(void)
{
    return bp_strings[app_state.blood_pressure];
}

const char* get_fitz_string(void)
{
    return fitz_strings[app_state.fitzpatrick];
}

const char* get_temp_string(void)
{
    return temp_strings[app_state.temperature];
}

// Added from PC Simulator
const char* get_korotkoff_string(void)
{
    return korotkoff_strings[app_state.korotkoff];
}


/*
#include "app_state.h"

app_state_t app_state = { 0 };

// Scenario #
static const char* scenario_strings[] = {
    "NA",
    "1",
    "2",
    "3",
    "4"
};

// Oxygen saturation levels
static const char* oxygen_strings[] = {
    "Healthy",
    "Hypoxia",
    "Slightly below normal"
};

// Blood pressure states
static const char* bp_strings[] = {
    "NA",
    "Healthy",
    "Stage I Hypertensive",
    "Stage II Hypertensive",
    "Hypotensive"
};

// Fitzpatrick skin types
static const char* fitz_strings[] = {
    "NA",
    "I",
    "II",
    "III",
    "IV",
    "V",
    "VI"
};

// Temperature states
static const char* temp_strings[] = {
    "Normal",
    "Low Temperature"
};

const char* get_scenario_string(void)
{
    return scenario_strings[app_state.scenario];
}

const char* get_oxygen_string(void)
{
    return oxygen_strings[app_state.oxygen];
}

const char* get_bp_string(void)
{
    return bp_strings[app_state.blood_pressure];
}

const char* get_fitz_string(void)
{
    return fitz_strings[app_state.fitzpatrick];
}

const char* get_temp_string(void)
{
    return temp_strings[app_state.temperature];
}
*/



///* Include header file */
//#include "app_state.h"
//
////app_state_t app_state = { 0 };
//#include "app_state.h"
//
//app_state_t app_state = { 0 };
//
///* Lookup tables */
//
//static const char* scenario_strings[] = {
//    "N/A",
//    "Scenario 1",
//    "Scenario 2",
//    "Scenario 3",
//    "Scenario 4"
//};
//
//static const char* oxygen_strings[] = {
//    "Healthy",
//    "Hypoxia",
//    "Slightly below normal"
//};
//
//static const char* bp_strings[] = {
//    "Healthy",
//    "Stage I Hypertensive",
//    "Stage II Hypertensive",
//    "Hypotensive"
//};
//
//static const char* fitz_strings[] = {
//    "No Selection",
//    "Type I",
//    "Type II",
//    "Type III",
//    "Type IV",
//    "Type V",
//    "Type VI"
//};
//
//static const char* temp_strings[] = {
//    "Normal",
//    "Low Temperature"
//};
//
///* Getter functions */
//
//const char* get_scenario_string(void)
//{
//    return scenario_strings[app_state.scenario];
//}
//
//const char* get_oxygen_string(void)
//{
//    return oxygen_strings[app_state.oxygen];
//}
//
//const char* get_bp_string(void)
//{
//    return bp_strings[app_state.blood_pressure];
//}
//
//const char* get_fitz_string(void)
//{
//    return fitz_strings[app_state.fitzpatrick];
//}
//
//const char* get_temp_string(void)
//{
//    return temp_strings[app_state.temperature];
//}
////#endif
