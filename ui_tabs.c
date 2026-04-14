/* Name, Date: Marissa Andrews, 2/14/2026
/* Purpose: Intilize tabs for ui*/

#include "ui_tabs.h"
#include "ui_home_os.h"
#include "ui_home_bp.h"
#include "ui_diagnose.h"
#include "ui_usage.h"
#include "ui_os.h"

#include "lvgl/lvgl.h"

/* Global objects */
lv_obj_t* main_tabview;
lv_obj_t* sub_tabview;
int diagnose_tab_index;

void ui_tabs_create(void)
{
    /* =========================
     * Main Tabview
     * ========================= */
    main_tabview = lv_tabview_create(lv_scr_act());

    lv_tabview_set_tab_bar_position(main_tabview, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(main_tabview, 40);

    /* Create main tabs */
    lv_obj_t* tab1 = lv_tabview_add_tab(main_tabview, "Home");
    lv_obj_t* tab2 = lv_tabview_add_tab(main_tabview, "Information");
    lv_obj_t* tab3 = lv_tabview_add_tab(main_tabview, "SpO2");
    lv_obj_t* tab4 = lv_tabview_add_tab(main_tabview, "System Usage");

    diagnose_tab_index = 1;


     /* Optional: remove layout interference */
    lv_obj_set_layout(tab1, LV_LAYOUT_NONE);

    /* Create sub tabview LAST so it stays on top */
    sub_tabview = lv_tabview_create(tab1);

    /* Make it fill the entire tab */
    lv_obj_set_size(sub_tabview, lv_pct(100), lv_pct(100));

    /* Position sub tab buttons on the left */
    lv_tabview_set_tab_bar_position(sub_tabview, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(sub_tabview, 30);

    /* Create sub-tabs */
    lv_obj_t* sub1 = lv_tabview_add_tab(sub_tabview, "Blood Pressure");
    lv_obj_t* sub2 = lv_tabview_add_tab(sub_tabview, "Oxygen Saturation");

    /* Populate sub-tabs */
    ui_home_create(sub1); // fix this function!!!
    ui_home_create_2(sub2);

    /* =========================
     * OTHER MAIN TABS
     * ========================= */

    ui_diagnose_create(tab2);
    ui_os_create(tab3);
    ui_usage_create(tab4);

    /* =========================
     * OPTIONAL SETTINGS
     * ========================= */

     /* Disable swipe between main tabs */
    lv_obj_clear_flag(lv_tabview_get_content(main_tabview), LV_OBJ_FLAG_SCROLLABLE);
}





