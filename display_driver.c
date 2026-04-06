#include <stdio.h>
#include "display_driver.h"
#include "lvgl.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_st7789.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "display";

// ====== CONFIGURE YOUR PINS ======
// Screen 1 - SPI2
#define D1_MOSI 15
#define D1_SCLK 18
#define D1_CS   40  // Changed from 36
#define D1_DC   41  // Changed from 37
#define D1_RST  42  // Changed from 21
#define LCD1_HOST SPI2_HOST

// Screen 2 - SPI3
#define D2_MOSI   5   // often shared with display
#define D2_SCLK   16   // often shared with display
#define D2_CS     21
#define D2_DC     39   // set to -1 if not connected
#define D2_RST    45  // set to -1 if not connected
#define LCD2_HOST SPI3_HOST  // must match display if sharing SPI bus

static esp_lcd_panel_handle_t panel_handle_1 = NULL;
static esp_lcd_panel_handle_t panel_handle_2 = NULL;

static lv_disp_draw_buf_t draw_buf1;
static lv_disp_draw_buf_t draw_buf2;
static lv_disp_drv_t disp1_drv;
static lv_disp_drv_t disp2_drv;

void setup_cs_pins(void) {
    // 1. Define which pins to configure (using a bit mask)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 17) | (1ULL << 21), // Touch CS (17) and Screen 2 CS (21)
        .mode = GPIO_MODE_OUTPUT,                    // CS pins must be outputs
        .pull_up_en = GPIO_PULLUP_ENABLE,            // <--- ENABLES THE INTERNAL 45k PULL-UP
        .pull_down_en = GPIO_PULLDOWN_DISABLE,       // Ensure pull-down is off
        .intr_type = GPIO_INTR_DISABLE               // No interrupts needed for CS
    };

    // 2. Apply the configuration
    gpio_config(&io_conf);

    // 3. IMMEDIATELY set them HIGH (Idle state for SPI)
    // This prevents the "Teal Screen" or "0/4096" noise at startup
    gpio_set_level(17, 1); 
    gpio_set_level(21, 1); 
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false; // Return false to indicate we didn't yield a higher priority task
}

// Display 1 flush callback
void disp_1_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    if (panel_handle_1 == NULL) {
        lv_disp_flush_ready(disp);
        return;
    }

    esp_lcd_panel_draw_bitmap(
        panel_handle_1,
        area->x1,
        area->y1,
        area->x2 + 1, // +1 because coordinates are inclusive
        area->y2 + 1,
        color_p
    );
}

// Display 2 flush callback
void disp_2_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    if (panel_handle_2 == NULL) {
        lv_disp_flush_ready(disp);
        return;
    }

    esp_lcd_panel_draw_bitmap(
        panel_handle_2,
        area->x1,
        area->y1,
        area->x2 + 1, // +1 because coordinates are inclusive
        area->y2 + 1,
        color_p
    );
}

// ====== DISPLAY INIT ======
disp_handles_t display_init(void)
{


    // Define buffer size: 480 pixels wide * 80 pixels tall (splits screen into 4 horizontal strips)
    size_t disp1_buffer_pixels = 480 * 80;
    size_t disp1_buffer_size_bytes = disp1_buffer_pixels * sizeof(lv_color_t);
    size_t disp2_buffer_pixels = 240 * 80;
    size_t disp2_buffer_size_bytes = disp2_buffer_pixels * sizeof(lv_color_t);

    // Double Buffer Allocation
    // Allocating in Internal RAM for maximum DMA speed and reliability
    lv_color_t *buf1a = heap_caps_malloc(disp1_buffer_pixels * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_color_t *buf1b = heap_caps_malloc(disp1_buffer_pixels * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_color_t *buf2a = heap_caps_malloc(disp2_buffer_pixels * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    lv_color_t *buf2b = heap_caps_malloc(disp2_buffer_pixels * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    
    if (!buf1a || !buf1b || !buf2a || !buf2b) {
        ESP_LOGE(TAG, "Buffer alloc failed! Free DMA RAM: %d bytes",
                 heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
        return (disp_handles_t){0}; // Return empty handles on failure
    }

    // SPI Bus Init for Display 1 (SPI2)
    spi_bus_config_t buscfg1 = {
        .mosi_io_num = D1_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = D1_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // Max transfer must be at least the size of one buffer
        .max_transfer_sz = disp1_buffer_size_bytes + 1024, 
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD1_HOST, &buscfg1, SPI_DMA_CH_AUTO));

        // SPI Bus Init for Display 2 (SPI3)
    spi_bus_config_t buscfg2 = {
        .mosi_io_num = D2_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = D2_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // Max transfer must be at least the size of one buffer
        .max_transfer_sz = disp2_buffer_size_bytes + 1024, 
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD2_HOST, &buscfg2, SPI_DMA_CH_AUTO));


    // IO Config for Display 1
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = D1_DC,
        .cs_gpio_num = D1_CS,
        .pclk_hz = 80 * 1000 * 1000, 
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 30, // Increase if you have a lot of updates to push
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD1_HOST, &io_config, &io_handle));

    // IO Config for Display 2
    esp_lcd_panel_io_handle_t io_handle_2 = NULL;
    esp_lcd_panel_io_spi_config_t io_config_2 = {
        .dc_gpio_num = D2_DC,
        .cs_gpio_num = D2_CS,
        .pclk_hz = 10 * 1000 * 1000, 
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 30, // Increase if you have a lot of updates to push
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD2_HOST, &io_config_2, &io_handle_2));

    // For Screen 1
    esp_lcd_panel_io_callbacks_t cbs1 = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs1, &disp1_drv);

    // For Screen 2
    esp_lcd_panel_io_callbacks_t cbs2 = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle_2, &cbs2, &disp2_drv);

    // Panel Config for Display 1
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = D1_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle_1));

    // Panel Config for Display 2
    esp_lcd_panel_dev_config_t panel_config_2 = {
        .reset_gpio_num = D2_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR, // ST7789 typically uses RGB order
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle_2, &panel_config_2, &panel_handle_2));

    // Hardware Sequence for Display 1 and 2
    esp_lcd_panel_reset(panel_handle_1);
    vTaskDelay(pdMS_TO_TICKS(100)); // Increased reset delay
    esp_lcd_panel_init(panel_handle_1);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_lcd_panel_reset(panel_handle_2);
    vTaskDelay(pdMS_TO_TICKS(200)); // Increased reset delay
    esp_lcd_panel_init(panel_handle_2);
    vTaskDelay(pdMS_TO_TICKS(120)); 

    // LANDSCAPE SETTINGS: true; false, false
    esp_lcd_panel_swap_xy(panel_handle_1, true);
    esp_lcd_panel_mirror(panel_handle_1, false, false);

    esp_lcd_panel_swap_xy(panel_handle_2, false);
    esp_lcd_panel_mirror(panel_handle_2, false, false);

    esp_lcd_panel_invert_color(panel_handle_2, true); // Toggle this true/false

    // Explicitly clear any hardware offset/gap
    esp_lcd_panel_set_gap(panel_handle_1, 0, 0); 
    esp_lcd_panel_set_gap(panel_handle_2, 0, 0); 
    
    esp_lcd_panel_disp_on_off(panel_handle_1, true);
    esp_lcd_panel_disp_on_off(panel_handle_2, true);

    // LVGL setup with TWO buffers
    // Pass buf1a AND buf2a to the draw buffer init
    lv_disp_draw_buf_init(&draw_buf1, buf1a, buf1b, disp1_buffer_pixels);
    lv_disp_draw_buf_init(&draw_buf2, buf2a, buf2b, disp2_buffer_pixels);
    

    disp_handles_t handles;
    // Screen 1
    lv_disp_drv_init(&disp1_drv);
    disp1_drv.hor_res = 480;
    disp1_drv.ver_res = 320;
    disp1_drv.draw_buf = &draw_buf1; // Ensure this is linked
    disp1_drv.flush_cb = disp_1_flush;
    handles.s1 = lv_disp_drv_register(&disp1_drv);

    // Screen 2
    lv_disp_drv_init(&disp2_drv);
    disp2_drv.hor_res = 240;
    disp2_drv.ver_res = 320;
    disp2_drv.draw_buf = &draw_buf2; // Ensure this is linked
    disp2_drv.flush_cb = disp_2_flush;
    ESP_LOGI(TAG, "Debug: S2 Buf: %p, S2 Res: %dx%d", disp2_drv.draw_buf, disp2_drv.hor_res, disp2_drv.ver_res);
    handles.s2 = lv_disp_drv_register(&disp2_drv);

    if (handles.s1) ESP_LOGI(TAG, "S1 registered: %p", handles.s1);
    if (handles.s2) ESP_LOGI(TAG, "S2 registered: %p", handles.s2);
    else ESP_LOGE(TAG, "S2 REGISTRATION FAILED!");
    return handles;
}