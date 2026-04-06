#include "ui_scroll.h"

// event handlers

static void scroll_up_event(lv_event_t* e)
{
    lv_obj_t* cont = lv_event_get_user_data(e);

    // Get current vertical scroll position (how far down we have scrolled)
    lv_coord_t scroll_y = lv_obj_get_scroll_y(cont);

    // Only scroll up if we aren't already at the very top (0)
    if (scroll_y > 0) {
        // Clamp the scroll: don't scroll up more than the distance to the top
        lv_coord_t step = (scroll_y < 60) ? scroll_y : 60;
        lv_obj_scroll_by(cont, 0, step, LV_ANIM_ON);
    }
}

static void scroll_down_event(lv_event_t* e)
{
    lv_obj_t* cont = lv_event_get_user_data(e);
    
    // Get the remaining distance to the bottom of the content
    lv_coord_t scroll_bottom = lv_obj_get_scroll_bottom(cont);

    // Only scroll down if there is content left to see below the current view
    if (scroll_bottom > 0) {
        // Clamp the scroll: don't scroll down more than the distance to the bottom
        lv_coord_t step = (scroll_bottom < 60) ? scroll_bottom : 60;
        lv_obj_scroll_by(cont, 0, -step, LV_ANIM_ON);
    }
}

//public functions 

lv_obj_t* ui_create_scrollable_container(lv_obj_t* parent)
{
    lv_obj_t* cont = lv_obj_create(parent);

    lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_AUTO);

    lv_obj_add_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    return cont;
}
lv_obj_t* ui_add_scroll_buttons(lv_obj_t* parent, lv_obj_t* target)
{
    /* Container for both buttons */
    lv_obj_t* btn_cont = lv_obj_create(parent);

    lv_obj_set_size(btn_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(btn_cont, LV_ALIGN_TOP_RIGHT, -10, 10);

    lv_obj_add_flag(btn_cont, LV_OBJ_FLAG_FLOATING);

    /* Horizontal layout */
    lv_obj_set_flex_flow(btn_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_cont,
        LV_FLEX_ALIGN_CENTER,
        LV_FLEX_ALIGN_CENTER,
        LV_FLEX_ALIGN_CENTER);

    lv_obj_set_style_pad_all(btn_cont, 5, 0);
    lv_obj_set_style_pad_gap(btn_cont, 5, 0);

    /* UP button */
    lv_obj_t* btn_up = lv_btn_create(btn_cont);
    lv_obj_set_size(btn_up, 40, 40);

    lv_obj_t* label_up = lv_label_create(btn_up);
    lv_label_set_text(label_up, LV_SYMBOL_UP);
    lv_obj_center(label_up);

    /* DOWN button */
    lv_obj_t* btn_down = lv_btn_create(btn_cont);
    lv_obj_set_size(btn_down, 40, 40);

    lv_obj_t* label_down = lv_label_create(btn_down);
    lv_label_set_text(label_down, LV_SYMBOL_DOWN);
    lv_obj_center(label_down);

    /* Events (hold-to-scroll feels better) */
    lv_obj_add_event_cb(btn_up, scroll_up_event, LV_EVENT_PRESSING, target);
    lv_obj_add_event_cb(btn_down, scroll_down_event, LV_EVENT_PRESSING, target);

    return btn_cont;
}
//void ui_add_scroll_buttons(lv_obj_t* parent, lv_obj_t* target)
//{
//    /* Up button */
//    lv_obj_t* btn_up = lv_btn_create(parent);
//    lv_obj_align(btn_up, LV_ALIGN_TOP_RIGHT, -10, 50);
//    lv_obj_add_flag(btn_up, LV_OBJ_FLAG_FLOATING);
//
//    lv_obj_t* label_up = lv_label_create(btn_up);
//    lv_label_set_text(label_up, LV_SYMBOL_UP);
//    lv_obj_center(label_up);
//
//    /* Down button */
//    lv_obj_t* btn_down = lv_btn_create(parent);
//    lv_obj_align(btn_down, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
//    lv_obj_add_flag(btn_down, LV_OBJ_FLAG_FLOATING);
//
//    lv_obj_t* label_down = lv_label_create(btn_down);
//    lv_label_set_text(label_down, LV_SYMBOL_DOWN);
//    lv_obj_center(label_down);
//
//    /* Events (hold-to-scroll) */
//    lv_obj_add_event_cb(btn_up, scroll_up_event, LV_EVENT_PRESSING, target);
//    lv_obj_add_event_cb(btn_down, scroll_down_event, LV_EVENT_PRESSING, target);
//}
