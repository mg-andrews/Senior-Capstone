

#ifndef BP_OUTPUT_H
#define BP_OUTPUT_H

#include <stdbool.h>

/* Randomize systolic/diastolic for the given dropdown state index (1-4).
   Returns false if state == 0 (Manual) or out of range.               */
bool bp_randomize_for_state(int state);

/* Call from dropdown_event_cb instead of the old fixed-value block.
   Handles app_state update, randomization, and input locking.        */
void bp_handle_dropdown_selection(int selected);

#endif 
