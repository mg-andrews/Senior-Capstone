////* Name, Date: Marissa Andrews, 4/8/2026
////Purpose: Separate home page for just OS components /

#include "ui_home_os.h"
#include "ui_diagnose.h"
#include "app_state.h"
#include "ui_scroll.h"
#include <stdint.h>

/* ---------------- GLOBALS ---------------- */
static lv_obj_t* dd_oxygen;

/* ---------------- DROPDOWN ---------------- */
static void dropdown_event_cb_2(lv_event_t* e)
{
    lv_obj_t* dd = lv_event_get_target(e);
    uint32_t selected = lv_dropdown_get_selected(dd);
    uint32_t index = (uint32_t)(uintptr_t)lv_event_get_user_data(e);

    if (index == 0)
        app_state.oxygen = selected;
    else if (index == 1)
        app_state.fitzpatrick = selected;
    else if (index == 2)
        app_state.temperature = selected;

    diagnose_update();
}


/* ---------------- UI ---------------- */
void ui_home_create_2(lv_obj_t* parent)
{
    lv_obj_t* cont = ui_create_scrollable_container(parent);

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_START);

    lv_obj_set_style_pad_all(cont, 20, 0);
    lv_obj_set_style_pad_row(cont, 20, 0);

    /* optional but recommended */
    lv_obj_set_style_pad_right(cont, 60, 0);

    const char* labels[3] = {
        "Oxygen Saturation",
        "Fitzpatrick Value",
        "Temperature"
    };

    const char* options_list[3] = {
        "Healthy\nHypoxia\nSlightly below normal",
        "Type I\nType II\nType III\nType IV\nType V\nType VI",
        "Normal\nLow Temperature"
    };

    for (int i = 0; i < 3; i++)
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

        /* FIXED CALLBACK */
        lv_obj_add_event_cb(dd, dropdown_event_cb_2, LV_EVENT_VALUE_CHANGED, (void*)(uintptr_t)i);

        if (i == 0)
            dd_oxygen = dd;
    }

    ui_add_scroll_buttons(parent, cont);
}
