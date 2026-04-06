// display_driver.h
#ifndef DISPLAY_DRIVER_H
#define DISPLAY_DRIVER_H
#include "lvgl.h"
    typedef struct {
        lv_disp_t *s1;
        lv_disp_t *s2;
    } disp_handles_t;
    disp_handles_t display_init(void);
    void setup_cs_pins(void);
#endif
