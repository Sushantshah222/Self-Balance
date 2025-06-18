#include "common.h"
#include <string.h>

shared_data_t shared_data;

void common_init() {
    // Initialize mutexes
    shared_data.imu_mutex = xSemaphoreCreateMutex();
    shared_data.motor_mutex = xSemaphoreCreateMutex();
    
    // Initialize Wifi data
    shared_data.net.wifi_connected = false;

    // Initialize IMU data
    memset(&shared_data.imu, 0, sizeof(shared_data.imu));
    shared_data.imu.data_valid = false;
    
    // Initialize motor data
    shared_data.motors.pwm_channel_left = LEDC_CHANNEL_0;
    shared_data.motors.pwm_channel_right = LEDC_CHANNEL_1;
    shared_data.motors.deadzone = 0;
    shared_data.motors.max_pwm = 255;

    shared_data.motors.pwm_gpio_left = 12;
    shared_data.motors.pwm_gpio_right = 13;

    shared_data.motors.gpio_left_a = 14;
    shared_data.motors.gpio_left_b = 15;
    shared_data.motors.gpio_right_a = 16;
    shared_data.motors.gpio_right_b = 17;

    shared_data.motors.speed_left = 0;
    shared_data.motors.speed_right = 0;
}
