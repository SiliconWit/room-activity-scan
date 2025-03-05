#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_NUM            UART_NUM_1
#define TXD_PIN             GPIO_NUM_17  // Adjust according to your setup
#define RXD_PIN             GPIO_NUM_18  // Adjust according to your setup
#define BUF_SIZE            256

void hlk_ld2420_task(void *arg) {
    uint8_t data[BUF_SIZE];
    const char *cmd = "AT+READ\r\n"; // Replace with actual command

    while (1) {
        // Send command to sensor
        uart_write_bytes(UART_NUM, cmd, strlen(cmd));

        // Delay to allow sensor response
        vTaskDelay(pdMS_TO_TICKS(100));

        // Read response
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';  // Null-terminate the response
            printf("HLK-LD2420 Response: %s\n", data);
        }
    }
}

void app_main() {
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Create task to communicate with HLK-LD2420
    xTaskCreate(hlk_ld2420_task, "hlk_ld2420_task", 4096, NULL, 10, NULL);
}
