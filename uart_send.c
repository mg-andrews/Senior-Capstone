// Name, Date: Marissa Andrews, 3/4/26

#include "uart_send.h"
#include "app_state.h"
#include <stdio.h>

#ifdef ESP_PLATFORM
#include "driver/uart.h"
#endif

#define TXD_PIN 20
#define RXD_PIN 47

void uart_init(void)
{
#ifdef ESP_PLATFORM

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM_2, &uart_config);

    uart_set_pin(UART_NUM_2,
        TXD_PIN,
        RXD_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0);

#endif
}


void uart_send_state(void)
{
    char buffer[256];

    snprintf(buffer, sizeof(buffer),
        //"Scenario:%d\nOxygen:%d\nBloodPressure:%d\nFitzpatrick:%d\nTemperature:%d\n",
        "%s,%s,%d,%d,%s,%s,%s,%s,%d,%d\n",
        get_scenario_string(),
        get_oxygen_string(),
        app_state.systolic,
        app_state.diastolic,
        get_bp_string(),
        get_fitz_string(),
        get_temp_string(),
        get_korotkoff_string(),
        app_state.ausc_gap_length,
        app_state.bpm

    );

#ifdef ESP_PLATFORM
    uart_write_bytes(UART_NUM_2, buffer, strlen(buffer));
    printf("UART OUTPUT:\n%s\n", buffer);
#else
    printf("UART OUTPUT:\n%s\n", buffer);
#endif
}

void uart_send_stop(void)
{
#ifdef ESP_PLATFORM
    uart_write_bytes(UART_NUM_2, "STOP\n", strlen("STOP\n"));
#else
    printf("UART OUTPUT:\nSTOP\n");
#endif
}