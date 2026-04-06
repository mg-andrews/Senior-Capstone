#ifndef TOUCH_DRIVER_H
#define TOUCH_DRIVER_H

#include "lvgl.h"

    void touch_hw_init(void);
    uint8_t touch_spi_transfer(uint8_t data);
    uint16_t touch_read_register(uint8_t command);

    void touch_init(lv_disp_t *disp);
    void touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

#endif