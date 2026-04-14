/* Name, Date: Marissa Andrews, 2/14/2026 */
/* Purpose: Create header file for tabs built for the UI */

#pragma once
#ifndef UI_TABS_H
#define UI_TABS_H
/* Include the lvgl.h for outside ui folder */
#include "../lvgl/lvgl.h"
// Added to be able to toggle the diagnose tab.
extern lv_obj_t* main_tabview;
extern lv_obj_t* sub_tabview;
extern uint32_t diagnose_tab_index;

void ui_tabs_create(void);



#endif
