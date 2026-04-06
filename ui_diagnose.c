/* Name, Date: Marissa Andrews, 2/17/2026
   Purpose: Create layout and functionality for the diagnose page */

#include <stdio.h>
#include "ui_diagnose.h"
#include "app_state.h"

LV_FONT_DECLARE(lv_font_montserrat_12);  // missing
LV_FONT_DECLARE(lv_font_montserrat_14);  // missing
LV_FONT_DECLARE(lv_font_montserrat_16);
LV_FONT_DECLARE(lv_font_montserrat_16_bold);
LV_FONT_DECLARE(lv_font_montserrat_18);  // missing
LV_FONT_DECLARE(lv_font_montserrat_20);

   /* Static label pointer so this file owns it */
static lv_obj_t* result_label = NULL;

/* Function to update diagnose text */
void diagnose_update()
{

    if (result_label == NULL) return;

    static char buffer[1024];
    buffer[0] = '\0';   // clear buffer

// Fitz
    if (app_state.fitzpatrick > 0)
    {
        char temp[64];
        snprintf(temp, sizeof(temp),
            "Fitzpatrick: %d\n",
            app_state.fitzpatrick);
        strcat(buffer, temp);
    }

// Temp
    if (app_state.temperature == 1)
    {
        strcat(buffer, "Patient Temperature: Low\n");
    }

// Scenarios
    switch (app_state.scenario)
    {
    case 1:
        strcat(buffer,
            "Scenario 1: Healthy patient annual visit.\n");
        break;

    case 2:
        strcat(buffer,
            "Scenario 2: Newly diagnosed hypertension.\n");
        break;

    case 3:
        strcat(buffer,
            "Scenario 3: Elderly patient with COPD and difficulty breathing.\n");
        break;

    case 4:
        strcat(buffer,
            "Scenario 4: Post-dialysis CKD patient with fatigue.\n");
        break;
    }

  // When nothing is selected
    if (strlen(buffer) == 0)
    {
        strcat(buffer, "No selections made.");
    }

    lv_label_set_text(result_label, buffer);
}
          

//Create Diagnose UI
    void ui_diagnose_create(lv_obj_t * parent)
    {
        lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_all(parent, 20, 0);

        // Title
        lv_obj_t* title = lv_label_create(parent);
        lv_label_set_text(title, "Scenario Information");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);

        result_label = lv_label_create(parent);
        lv_obj_set_width(result_label, 400);
        lv_label_set_long_mode(result_label, LV_LABEL_LONG_WRAP);

       

        // Intro text
        //lv_obj_t* instructions = lv_label_create(parent);

        //lv_label_set_long_mode(instructions, LV_LABEL_LONG_WRAP);
        //lv_obj_set_width(instructions, lv_pct(100));

        //lv_label_set_text(instructions,
        //    "Scenario Information"
        //);

        diagnose_update();  // initialize text
    }
