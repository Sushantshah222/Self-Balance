#include "pid_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor_driver.h"
#include <math.h>

static const char* TAG = "PID";
static pid_controller_t pid;

void pid_init(float kp, float ki, float kd, float setpoint) {
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.setpoint = setpoint;
    pid.integral = 0;
    pid.prev_error = 0;
    ESP_LOGI(TAG, "PID Initialized: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
}

float pid_update(float measurement) {
    float error = pid.setpoint - measurement;
    
    // P term
    float p_term = pid.kp * error;
    
    // I term with anti-windup
    if (!pid.enable_anti_windup || fabs(pid.output) < pid.max_output) {
        pid.integral += error;
        pid.integral = fmaxf(fminf(pid.integral, pid.max_integral), -pid.max_integral);
    }
    float i_term = pid.ki * pid.integral;
    
    // Filtered D term (first-order low-pass)
    static float prev_derivative = 0;
    float derivative = (error - pid.prev_error) * 0.2 + prev_derivative * 0.8;
    prev_derivative = derivative;
    float d_term = pid.kd * derivative;
    
    pid.output = p_term + i_term + d_term;
    pid.output = fmaxf(fminf(pid.output, pid.max_output), -pid.max_output);
    
    pid.prev_error = error;
    return pid.output;
}

// Modify pid_task():
static void pid_task(void* pvParameters) {
    // Initialize with more aggressive gains
    pid_init(25.0f, 0.5f, 5.0f, 0.0f); // Kp, Ki, Kd, Setpoint
    pid.max_output = 255.0f;  // Full motor range
    pid.max_integral = 100.0f;
    pid.enable_anti_windup = true;

    while (1) {
        if (shared_data.imu.data_valid) {
            float current_angle;
            if (xSemaphoreTake(shared_data.imu_mutex, pdMS_TO_TICKS(10))) {
                current_angle = shared_data.imu.pitch;
                xSemaphoreGive(shared_data.imu_mutex);
                
                // More conservative safety limit (30 degrees)
                if (fabs(current_angle) > 0.52f) { // ~30 degrees
                    motor_emergency_stop();
                    continue;
                }

                float output = pid_update(current_angle);
                
                // Scale PID output directly to PWM range
                int left = (int)(output * 1.5f);  // Increased gain
                int right = -(int)(output * 1.5f);
                
                // Apply non-linear response curve
                left = (abs(left) < 30) ? 0 : left;  // Deadzone
                right = (abs(right) < 30) ? 0 : right;
                
                if (xSemaphoreTake(shared_data.motor_mutex, pdMS_TO_TICKS(10))) {
                    shared_data.motors.speed_left = left;
                    shared_data.motors.speed_right = right;
                    xSemaphoreGive(shared_data.motor_mutex);
                }
                ESP_LOGI(TAG, "Angle: %.2f° | PID Out: %.1f | Motors: L=%d R=%d", 
                current_angle * 57.2958f,  // Rad to deg
                output,
                left, right);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Faster 200Hz control loop
    }
}



void pid_start_task(void) {
    // Initialize with reasonable values for a balancing robot
    // These will likely need tuning for your specific robot
    pid_init(15.0f, 0.0f, 2.0f, 0.0f); // Setpoint 0 = balanced position
    
    xTaskCreate(pid_task, "pid_task", 4096, NULL, 5, NULL);
}