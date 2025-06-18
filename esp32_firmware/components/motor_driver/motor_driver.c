#include "motor_driver.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "MOTOR";
#define CLAMP(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))

// Motor control function
static void set_motor_speed(ledc_channel_t channel, uint8_t gpio_a, uint8_t gpio_b, int speed) {
    speed = CLAMP(speed, -255, 255);

    if (speed > 5) {  // Deadzone for small values
        gpio_set_level(gpio_a, 1);
        gpio_set_level(gpio_b, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, speed);
    } 
    else if (speed < -5) {
        gpio_set_level(gpio_a, 0);
        gpio_set_level(gpio_b, 1);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, -speed);
    }
    else {  // Brake mode
        gpio_set_level(gpio_a, 0);
        gpio_set_level(gpio_b, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, 0);
    }
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

// Add emergency stop:
void motor_emergency_stop() {
    if (xSemaphoreTake(shared_data.motor_mutex, pdMS_TO_TICKS(10))) {
        set_motor_speed(LEDC_CHANNEL_0, 
                        shared_data.motors.gpio_left_a,
                        shared_data.motors.gpio_left_b,
                        0);

        set_motor_speed(LEDC_CHANNEL_1,
                        shared_data.motors.gpio_right_a,
                        shared_data.motors.gpio_right_b,
                        0);
        xSemaphoreGive(shared_data.motor_mutex);
    }
}

void motor_init(void) {
    // PWM timer config
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // PWM channel configs
    ledc_channel_config_t ch_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ch_conf.channel = LEDC_CHANNEL_0;
    ch_conf.gpio_num = shared_data.motors.pwm_gpio_left;
    ledc_channel_config(&ch_conf);

    ch_conf.channel = LEDC_CHANNEL_1;
    ch_conf.gpio_num = shared_data.motors.pwm_gpio_right;
    ledc_channel_config(&ch_conf);

    // Direction GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << shared_data.motors.gpio_left_a) |
                        (1ULL << shared_data.motors.gpio_left_b) |
                        (1ULL << shared_data.motors.gpio_right_a) |
                        (1ULL << shared_data.motors.gpio_right_b),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
}

static void motor_task(void* pvParameters) {
    ESP_LOGI(TAG, "Motor task started");

    while (1) {
        int left_pwm = 0, right_pwm = 0;

        if (xSemaphoreTake(shared_data.motor_mutex, pdMS_TO_TICKS(10))) {
            left_pwm = shared_data.motors.speed_left;
            right_pwm = shared_data.motors.speed_right;
            xSemaphoreGive(shared_data.motor_mutex);
        }

        set_motor_speed(LEDC_CHANNEL_0,
                        shared_data.motors.gpio_left_a,
                        shared_data.motors.gpio_left_b,
                        left_pwm);

        set_motor_speed(LEDC_CHANNEL_1,
                        shared_data.motors.gpio_right_a,
                        shared_data.motors.gpio_right_b,
                        right_pwm);

        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}

void motor_start_task(void) {
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 4, NULL);
}

void motor_set_speeds(int left, int right) {
    if (xSemaphoreTake(shared_data.motor_mutex, pdMS_TO_TICKS(10))) {
        shared_data.motors.speed_left = left;
        shared_data.motors.speed_right = right;
        xSemaphoreGive(shared_data.motor_mutex);
    }
}