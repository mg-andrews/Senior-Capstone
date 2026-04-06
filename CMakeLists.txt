idf_component_register(
    SRCS 
        "main.c"
        "display_driver.c"
        "touch_driver.c"
        "vc_ui_v1/uart_send.c"
        "vc_ui_v1/ui_tabs.c"
        "vc_ui_v1/ui_home.c"
        "vc_ui_v1/ui_usage.c"
        "vc_ui_v1/app_state.c"
        "vc_ui_v1/ui_diagnose.c"
        "vc_ui_v1/usage_image.c"
        "vc_ui_v1/ui_scroll.c"
        "vc_ui_v1/pulse_ox.c"
        "vc_ui_v1/spo2_calc.c"
        # add all other .c files from vc_ui_v1 here
    INCLUDE_DIRS 
        "."
        "vc_ui_v1"        # ← this lets #include "ui_tabs.h" resolve
    REQUIRES
        lvgl
        driver
        esp_lcd
        esp_lcd_st7796
        esp_lcd_st7789
        freertos
        esp_timer
)