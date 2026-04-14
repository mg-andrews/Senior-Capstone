#ifndef UI_HOME_BP_H
#define UI_HOME_BP_H

#include "lvgl.h"

/* Only expose if used elsewhere */
void ui_home_create(lv_obj_t* parent);

/* From ui_home_bp.c: allow diagnose to call send and stop */
void send_app_state(void);
void send_stop_signal(void);

/* Allow ui_diagnose to register its submit button so BP code can enable/disable it */
void ui_home_set_submit_btn(lv_obj_t* btn);

// Adding the sys/dia values
void ui_home_bp_lock_manual_inputs(bool locked);

#endif /* UI_HOME_BP_H */
#pragma once
