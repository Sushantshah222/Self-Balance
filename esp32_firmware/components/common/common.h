#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "driver/ledc.h"
#include <stdbool.h>

#define UDP_BUFFER_SIZE 128

typedef struct {
        // Add WiFi status
    struct {
        bool wifi_connected;
    } net;

    // IMU Data
    struct {
        float pitch, roll, yaw;
        TickType_t last_update;
        bool data_valid;
        char raw_buffer[UDP_BUFFER_SIZE];
    } imu;

    // Motor Data
    struct {
        int speed_left;
        int speed_right;
        uint8_t deadzone;
        ledc_channel_t pwm_channel_left;
        ledc_channel_t pwm_channel_right;
        uint16_t max_pwm;

        uint8_t pwm_gpio_left;
        uint8_t pwm_gpio_right;

        uint8_t gpio_left_a;
        uint8_t gpio_left_b;
        uint8_t gpio_right_a;
        uint8_t gpio_right_b;
    } motors;

    // Mutexes
    SemaphoreHandle_t imu_mutex;
    SemaphoreHandle_t motor_mutex;
} shared_data_t;

extern shared_data_t shared_data;
void common_init();