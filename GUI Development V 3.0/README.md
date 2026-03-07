# GUI Development V 3.0 Information

## Overview

This project implements a touchscreen-based UI for scenario input using **LVGL**. The UI is fully functional on a PC simulator and can be deployed to an **ESP32 microcontroller** with an **SPI TFT ST7796 display and touch input**.  

The system allows the user to select various clinical parameters (Scenario, Oxygen Saturation, Blood Pressure, Fitzpatrick Value, Temperature) and submit the selections. Upon pressing the **Submit** button, the data is formatted as **readable strings** and sent to a secondary microcontroller over **UART**.

---

## Touchscreen Functionality

- The LVGL library abstracts input events, so **touchscreen presses act like mouse clicks**. No UI changes are required to support touch.
- Touch input requires a driver for your hardware that reads coordinates and reports press/release events to LVGL.
- Once the touch driver is registered with LVGL, **all buttons, dropdowns, switches, and tabs respond to touch automatically**.

**Steps to enable touchscreen on ESP32:**

1. Initialize the display driver for ST7796 (SPI TFT) in `display_driver.c`.
2. Initialize the touch driver in `touch_driver.c`.
3. Register the touch driver with LVGL in `main.c`:

```c
lv_indev_drv_t indev_drv;
lv_indev_drv_init(&indev_drv);
indev_drv.type = LV_INDEV_TYPE_POINTER;
indev_drv.read_cb = touch_read;
lv_indev_drv_register(&indev_drv);
```
## File Organization
lv_sim_eclipse_sdl/
│
├── main.c                 # Contain functions to run tab_create function and sending strings. Very few edits are included on this. 
│
├── vc_ui_v1/
│   ├── ui_home.c              # Home page UI (dropdowns, submit)
│   ├── ui_diagnose.c          # Scenario Information tab
│   ├── ui_usage.c             # Usage instructions page
|   ├── display_driver.c/h     # ST7796 display initialization
|   ├── touch_driver.c/h       # Touchscreen input driver
|   ├── uart_send.c/h          # UART communication / send_app_state()
|   ├── app_state.c/h          # Application state & string getters for each state that is selected in the dropdown
│   └── ui_tabs.c              # Tab creation/management
│
├── CMakeLists.txt / Makefile  # Build configuration
└── README.md