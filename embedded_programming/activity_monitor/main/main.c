#include <stdio.h>
#include <string.h>
#include <time.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"       // Add this for esp_timer_get_time()
#include "nvs_flash.h"

#define UART_PORT UART_NUM_1
#define TX_PIN GPIO_NUM_17
#define RX_PIN GPIO_NUM_18
#define BUF_SIZE 1024

// Output pins for demo purposes
#define LED_PIN GPIO_NUM_2       // Built-in LED on most ESP32 dev boards
#define RELAY_PIN GPIO_NUM_4     // Connect to a relay for controlling appliances

// Sensor parameters
#define PRESENCE_TIMEOUT_SEC 60  // How long to consider the room occupied after last detection
#define MOTION_THRESHOLD 50      // Minimum change in range to be considered motion
#define SAMPLING_RATE_MS 100     // How often to sample the sensor in milliseconds

static const char *TAG = "HLK-LD2420";

typedef struct {
    bool presence_detected;   // ON or OFF
    int range;               // Range value in cm/dm
    int64_t last_motion_time; // Timestamp of last motion detection
    int previous_range;      // Previous range reading for motion detection
    char raw_data[256];      // Raw data from sensor for debugging
} sensor_reading_t;

typedef enum {
    ROOM_EMPTY,
    ROOM_OCCUPIED,
    MOTION_DETECTED,
    PERSON_STILL
} occupancy_state_t;

// Global state variables
static sensor_reading_t current_reading = {0};
static occupancy_state_t room_state = ROOM_EMPTY;
static int64_t room_occupied_time = 0;  // How long the room has been occupied (milliseconds)
static int occupancy_count = 0;         // Estimated number of people in the room

// Function prototypes
bool parse_sensor_data(const char *data, sensor_reading_t *reading);
void update_room_state(sensor_reading_t *reading);
void handle_room_state_change(occupancy_state_t old_state, occupancy_state_t new_state);
void control_smart_devices(occupancy_state_t state);

// Parse the sensor data into a structured format
bool parse_sensor_data(const char *data, sensor_reading_t *reading) {
    // Save previous range for motion detection
    reading->previous_range = reading->range;
    
    // Initialize with default values
    reading->presence_detected = false;
    reading->range = -1;
    strncpy(reading->raw_data, data, sizeof(reading->raw_data) - 1);
    reading->raw_data[sizeof(reading->raw_data) - 1] = '\0';
    
    // Check for presence status
    if (strstr(data, "ON") != NULL) {
        reading->presence_detected = true;
        reading->last_motion_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    }
    
    // Try to find the range value
    const char *range_str = strstr(data, "Range");
    if (range_str != NULL) {
        // Skip "Range " text and parse the number
        if (sscanf(range_str + 6, "%d", &reading->range) != 1) {
            reading->range = -1; // Parsing failed
        }
    }
    
    // Return true if we got at least one valid piece of data
    return (reading->presence_detected || reading->range >= 0);
}

// Determine the current state of the room based on sensor readings
void update_room_state(sensor_reading_t *reading) {
    occupancy_state_t previous_state = room_state;
    int64_t current_time = esp_timer_get_time() / 1000;
    
    // Detect motion based on range changes
    bool motion_detected = false;
    if (reading->range > 0 && reading->previous_range > 0) {
        int range_diff = abs(reading->range - reading->previous_range);
        if (range_diff > MOTION_THRESHOLD) {
            motion_detected = true;
            reading->last_motion_time = current_time;
        }
    }
    
    // Update room state based on presence and motion
    if (reading->presence_detected) {
        // Room is definitely occupied
        if (previous_state == ROOM_EMPTY) {
            // Someone entered the room
            room_state = ROOM_OCCUPIED;
            room_occupied_time = 0;
            occupancy_count++;
        }
        
        // Check if there's active motion
        if (motion_detected) {
            room_state = MOTION_DETECTED;
        } else {
            // Person is present but still
            int64_t time_since_last_motion = current_time - reading->last_motion_time;
            if (time_since_last_motion > 5000) { // 5 seconds
                room_state = PERSON_STILL;
            }
        }
        
        // Update room occupied time
        room_occupied_time += SAMPLING_RATE_MS;
    } else {
        // No presence detected - check if it's a temporary absence
        int64_t time_since_last_presence = current_time - reading->last_motion_time;
        if (time_since_last_presence > (PRESENCE_TIMEOUT_SEC * 1000)) {
            // No one has been detected for timeout period
            if (previous_state != ROOM_EMPTY) {
                room_state = ROOM_EMPTY;
                room_occupied_time = 0;
                if (occupancy_count > 0) occupancy_count--;
            }
        }
    }
    
    // Log state changes and handle transitions
    if (previous_state != room_state) {
        handle_room_state_change(previous_state, room_state);
    }
}

// Handle state transitions
void handle_room_state_change(occupancy_state_t old_state, occupancy_state_t new_state) {
    const char* state_names[] = {
        "ROOM_EMPTY", "ROOM_OCCUPIED", "MOTION_DETECTED", "PERSON_STILL"
    };
    
    ESP_LOGI(TAG, "State change: %s -> %s (People count: %d, Occupied for: %.1f min)",
        state_names[old_state], state_names[new_state], 
        occupancy_count, room_occupied_time / 60000.0);
    
    // Control smart devices based on new state
    control_smart_devices(new_state);
    
    // Here you could send state changes to a server, MQTT broker, etc.
}

// Control connected devices based on room state
void control_smart_devices(occupancy_state_t state) {
    // Simple example: control LED and relay based on occupancy
    switch (state) {
        case ROOM_EMPTY:
            // Turn off lights and appliances when room is empty
            gpio_set_level(LED_PIN, 0);
            gpio_set_level(RELAY_PIN, 0);
            break;
            
        case ROOM_OCCUPIED:
        case MOTION_DETECTED:
            // Turn on lights when room is occupied
            gpio_set_level(LED_PIN, 1);
            gpio_set_level(RELAY_PIN, 1);
            break;
            
        case PERSON_STILL:
            // Person is still - could dim lights after a while
            // For demo, just keep LED on
            gpio_set_level(LED_PIN, 1);
            break;
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting HLK-LD2420 smart presence detection system");
    
    // Initialize NVS flash (needed for many ESP32 functions)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Configure output pins
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(RELAY_PIN);
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "Sensor monitoring system initialized, waiting for data...");

    uint8_t data[BUF_SIZE];
    int sample_count = 0;
    
    // Main loop to read and process sensor data
    while (1) {
        // Read data from the HLK-LD2420
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Null-terminate the data
            data[len] = '\0';
            
            // Parse the received data
            if (parse_sensor_data((char *)data, &current_reading)) {
                // Update room state based on new reading
                update_room_state(&current_reading);
                
                // Log data periodically (every 10 samples)
                if (++sample_count % 10 == 0) {
                    ESP_LOGI(TAG, "Presence: %s, Range: %d cm", 
                             current_reading.presence_detected ? "DETECTED" : "NONE", 
                             current_reading.range);
                }
            } else {
                // If parsing failed, log the raw data for debugging
                ESP_LOGW(TAG, "Failed to parse sensor data: %s", data);
            }
        }
        
        // Delay to control sampling rate
        vTaskDelay(pdMS_TO_TICKS(SAMPLING_RATE_MS));
    }
}