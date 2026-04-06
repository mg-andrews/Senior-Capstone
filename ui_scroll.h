#ifndef UI_SCROLL_H
#define UI_SCROLL_H

#include "lvgl.h"

lv_obj_t* ui_create_scrollable_container(lv_obj_t* parent);
lv_obj_t* ui_add_scroll_buttons(lv_obj_t* parent, lv_obj_t* target);

#endif
