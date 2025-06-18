#pragma once
#include "common.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float input;
    float output;
    float integral;
    float prev_error;
    float max_output;
    float max_integral;
    bool  enable_anti_windup;
} pid_controller_t;

/**
 * @brief Initialize PID controller with parameters
 */
void pid_init(float kp, float ki, float kd, float setpoint);

/**
 * @brief Update PID controller with new measurement
 * @param measurement Current process value
 * @return Computed control output
 */
float pid_update(float measurement);

/**
 * @brief Start PID controller task
 */
void pid_start_task(void);