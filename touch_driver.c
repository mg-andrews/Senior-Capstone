#include "touch_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "rom/ets_sys.h" // For microsecond delays

static const char *TAG = "touch";

// ====== CONFIGURE YOUR PINS ======
#define TOUCH_MOSI   12   // often shared with display
#define TOUCH_MISO   4   // XPT2046 needs MISO
#define TOUCH_SCLK   11   // often shared with display
#define TOUCH_CS     17
#define TOUCH_IRQ    2   // set to -1 if not connected
#define TOUCH_HOST   SPI3_HOST  // must match display if sharing SPI bus

// Screen resolution for coordinate mapping
#define TOUCH_X_MIN  200
#define TOUCH_X_MAX  3900
#define TOUCH_Y_MIN  200
#define TOUCH_Y_MAX  3900
#define DISP_WIDTH   480
#define DISP_HEIGHT  320

void touch_hw_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TOUCH_CS) | (1ULL << TOUCH_SCLK) | (1ULL << TOUCH_MOSI),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << TOUCH_MISO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Internal 45k helps the 100k external
    };
    gpio_config(&in_conf);

    gpio_set_level(TOUCH_CS, 1);
    gpio_set_level(TOUCH_SCLK, 0);
}

// Send 8 bits and receive 8 bits simultaneously
uint8_t touch_spi_transfer(uint8_t data) {
    uint8_t received = 0;
    for (int i = 7; i >= 0; i--) {
        // 1. Set MOSI (Data Out)
        gpio_set_level(TOUCH_MOSI, (data >> i) & 0x01);
        
        // 2. Wait for 100k resistor/capacitance to settle (1 microsecond)
        ets_delay_us(1); 
        
        // 3. Clock High
        gpio_set_level(TOUCH_SCLK, 1);
        ets_delay_us(1);

        // 4. Sample MISO (Data In)
        if (gpio_get_level(TOUCH_MISO)) {
            received |= (1 << i);
        }

        // 5. Clock Low
        gpio_set_level(TOUCH_SCLK, 0);
    }
    return received;
}

uint16_t touch_read_register(uint8_t command) {
    uint16_t result = 0;
    gpio_set_level(TOUCH_CS, 0);

    touch_spi_transfer(command); // Send command (e.g., 0x90 for X, 0xD0 for Y)
    
    // Read 16 bits (The XPT2046 returns 12 bits of data)
    uint8_t msb = touch_spi_transfer(0x00);
    uint8_t lsb = touch_spi_transfer(0x00);

    gpio_set_level(TOUCH_CS, 1);

    result = ((msb << 8) | lsb) >> 3; // Shift to get 12-bit value
    return result;
}

// Add this to touch_driver.c
void touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    //ESP_LOGD(TAG, "LVGL Tick: Checking Touch...");
    // 1. Read Raw Values (0x90 = X, 0xD0 = Y for most orientations)
    // Note: You might need to swap these based on your specific screen's wiring
    uint16_t x_raw = touch_read_register(0x90); 
    uint16_t y_raw = touch_read_register(0xD0);
    //ESP_LOGI("TOUCH_RAW", "X: %u, Y: %u", x_raw, y_raw);

    x_raw = 4095 - x_raw; // Fix mirroring if needed (adjust based on your wiring)
    y_raw = 4095 - y_raw; // Fix mirroring if needed (adjust based on your wiring)

    // 2. Determine if the screen is actually being pressed
    // XPT2046 usually returns near 0 or 4095 if not touched
    if (x_raw > 100 && x_raw < 4000 && y_raw > 100 && y_raw < 4000) {
        data->state = LV_INDEV_STATE_PR;

        // 3. Map Raw to Screen Pixels (480x320)
        // Adjust the MIN/MAX values based on your calibration
        int32_t x = (x_raw - TOUCH_X_MIN) * DISP_WIDTH / (TOUCH_X_MAX - TOUCH_X_MIN);
        int32_t y = (y_raw - TOUCH_Y_MIN) * DISP_HEIGHT / (TOUCH_Y_MAX - TOUCH_Y_MIN);

        // Clamping
        if (x < 0) x = 0;
        if (x >= DISP_WIDTH) x = DISP_WIDTH - 1;
        if (y < 0) y = 0;
        if (y >= DISP_HEIGHT) y = DISP_HEIGHT - 1;

        data->point.x = x;
        data->point.y = y;

        //ESP_LOGI(TAG, "Touch: Raw(%u, %u) -> Pxl(%ld, %ld)", x_raw, y_raw, x, y);
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// Add this to touch_driver.c
void touch_init(lv_disp_t *disp) {
    // Initialize the GPIOs first
    touch_hw_init();

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    indev_drv.disp = disp; // Attach it to Screen 1
    lv_indev_drv_register(&indev_drv);
    
    ESP_LOGI(TAG, "Bit-Bang Touch Initialized on pins CS:%d, CLK:%d, MOSI:%d, MISO:%d", 
             TOUCH_CS, TOUCH_SCLK, TOUCH_MOSI, TOUCH_MISO);
}

/*
static spi_device_handle_t touch_spi = NULL;

// ── Raw SPI read ──────────────────────────────────────────────
static uint16_t xpt2046_read_raw(uint8_t cmd)
{
    spi_transaction_t t = {
        .length = 24,       // 8 cmd + 16 response
        .tx_data = {cmd, 0x00, 0x00},
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    spi_device_polling_transmit(touch_spi, &t);
    return ((t.rx_data[1] << 8) | t.rx_data[2]) >> 3;
}

// ── Init ──────────────────────────────────────────────────────
void touch_init(lv_disp_t *disp)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = TOUCH_CS,
        .queue_size = 10
    };
    ESP_ERROR_CHECK(spi_bus_add_device(TOUCH_HOST, &devcfg, &touch_spi));

    if (TOUCH_IRQ >= 0) {
        gpio_config_t irq_conf = {
            .pin_bit_mask = (1ULL << TOUCH_IRQ),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        gpio_config(&irq_conf);
    }

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    
    // THIS IS THE FIX for the "set_display" issue:
    indev_drv.disp = disp; 

    lv_indev_drv_register(&indev_drv);
    ESP_LOGI(TAG, "Touch linked to Display %p", disp);

    ESP_LOGI(TAG, "XPT2046 touch initialized");
}

// ── LVGL read callback ────────────────────────────────────────
void touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    // Check IRQ pin — low means touched
    bool touched = (TOUCH_IRQ >= 0)
        ? (gpio_get_level(TOUCH_IRQ) == 0)
        : true; // if no IRQ, always try to read

    if (!touched) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }

    // Read raw X and Y (0xD0 = Y, 0x90 = X for XPT2046)
    uint16_t raw_x = xpt2046_read_raw(0x90);
    uint16_t raw_y = xpt2046_read_raw(0xD0);
    ESP_LOGW(TAG, "RAW SPI DATA -> X: %u, Y: %u", raw_x, raw_y); // Uncomment for debugging raw values

    // Map raw ADC values to screen coordinates
    int16_t x = ((int32_t)(raw_x - TOUCH_X_MIN) * DISP_WIDTH)
                / (TOUCH_X_MAX - TOUCH_X_MIN);
    int16_t y = ((int32_t)(raw_y - TOUCH_Y_MIN) * DISP_HEIGHT)
                / (TOUCH_Y_MAX - TOUCH_Y_MIN);
    
    // Fix mirroring
    x = 480 - x; 
    y = 320 - y;

    // Clamp to screen bounds
    x = x < 0 ? 0 : (x >= DISP_WIDTH  ? DISP_WIDTH  - 1 : x);
    y = y < 0 ? 0 : (y >= DISP_HEIGHT ? DISP_HEIGHT - 1 : y);

    data->point.x = x;
    data->point.y = y;
    data->state = LV_INDEV_STATE_PR;

}
*/