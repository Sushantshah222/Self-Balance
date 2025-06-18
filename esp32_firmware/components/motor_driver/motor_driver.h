#pragma once
#include "common.h"

/**
 * @brief Initialize motor driver hardware
 */
void motor_init(void);

/**
 * @brief Start motor control task
 */
void motor_start_task(void);

/**
 * @brief Set motor speeds directly (for testing)
 * @param left Left motor speed (-255 to 255)
 * @param right Right motor speed (-255 to 255)
 */
void motor_set_speeds(int left, int right);

/**
 * @brief Emergency stop
 */
void motor_emergency_stop(void);