#include "common.h"
#include "wifi_sta.h"
#include "udp_network.h"
#include "imu_parser.h"
#include "pid_controller.h"
#include "motor_driver.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

void app_main() {
    // Initialize all components
    common_init();
    wifi_init_sta();
    motor_init();
    udp_server_start();
    udp_client_start();
    imu_parser_start();

    // Start control tasks
    pid_start_task();
    motor_start_task();

    // Main monitoring loop
    while(1) {
        ESP_LOGI(TAG, "System Status | "
                        "IMU Valid: %d | "
                        "Pitch: %.2f | "
                        "Motors: L=%d, R=%d | "
                        "WiFi: %s",
                shared_data.imu.data_valid,
                shared_data.imu.pitch,
                shared_data.motors.pwm_channel_left,
                shared_data.motors.pwm_channel_right,
                shared_data.net.wifi_connected ? "Connected" : "Disconnected");
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}