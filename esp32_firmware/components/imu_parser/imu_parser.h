#pragma once
#include "common.h"

/**
 * @brief Parses raw IMU data string into shared_data
 * @param input Input string format: "PITCH,ROLL,YAW"
 * @return true if parsing succeeded, false otherwise
 */
bool imu_parser_parse(const char* input);

/**
 * @brief Task that monitors and parses incoming IMU data
 */
void imu_parser_task(void* pvParameters);

/**
 * @brief Starts IMU server task
 */
void imu_parser_start(void);