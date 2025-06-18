#include "imu_parser.h"
#include "esp_log.h"
#include <math.h>

#define TAG "IMU_PARSER"

bool imu_parser_parse(const char* input) {
    float pitch = 0, roll = 0, yaw = 0;
    ESP_LOGI(TAG, "Parsing: %s", input);
    // Parse "PITCH,ROLL,YAW" format
    if (sscanf(input, "%f,%f,%f", &pitch, &roll, &yaw) != 3) {
        ESP_LOGE(TAG, "Invalid IMU format: %s", input);
        return false;
    }

    // Validate realistic values (e.g., |pitch| < π/2)
    if (fabs(pitch) > 1.57f || fabs(roll) > 1.57f) {  // ~90 degrees
        ESP_LOGW(TAG, "Dangerous IMU values: %s", input);
        return false;
    }

    // Update shared data (thread-safe)
    if (xSemaphoreTake(shared_data.imu_mutex, pdMS_TO_TICKS(100))) {
        shared_data.imu.pitch = pitch;
        shared_data.imu.roll = roll;
        shared_data.imu.yaw = yaw;
        shared_data.imu.last_update = xTaskGetTickCount();
        shared_data.imu.data_valid = true;
        xSemaphoreGive(shared_data.imu_mutex);
        return true;
    }

    ESP_LOGW(TAG, "Failed to acquire mutex");
    return false;
}

void imu_parser_task(void* pvParameters) {
    while (1) {
        if (xSemaphoreTake(shared_data.imu_mutex, pdMS_TO_TICKS(50))) {
            if (strlen(shared_data.imu.raw_buffer) > 0) {
                char local_copy[UDP_BUFFER_SIZE];
                strlcpy(local_copy, shared_data.imu.raw_buffer, sizeof(local_copy));
                
                // clear buffer
                shared_data.imu.raw_buffer[0] = '\0';
                xSemaphoreGive(shared_data.imu_mutex);  
                
                // Attempt parse
                if (!imu_parser_parse(local_copy)) {
                    ESP_LOGE(TAG, "Discarding invalid IMU: %s", local_copy);
                }
            } else {
                xSemaphoreGive(shared_data.imu_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Yield
    }
}

void imu_parser_start(void)
{
    xTaskCreate(imu_parser_task, "imu_parser", 4096, NULL, 4, NULL);
}