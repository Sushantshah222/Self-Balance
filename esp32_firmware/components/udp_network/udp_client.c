#include "udp_network.h"
#include "common.h"

#include <string.h>
#include <sys/param.h>
#include <errno.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "esp_system.h"

#define TAG "UDP_CLIENT"

static void udp_client_task(void *pvParameters) {
    int sock = -1;
    struct sockaddr_in dest_addr;
    char buffer[32];
    uint32_t seq_num = 0;
    
    while(1) {
        // Wait for WiFi
        if (!shared_data.net.wifi_connected) {
            ESP_LOGW(TAG, "Network not ready, skipping send");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Socket management
        if (sock < 0) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock < 0) {
                ESP_LOGE(TAG, "Socket creation failed: %d", errno);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            
            // Configure destination
            memset(&dest_addr, 0, sizeof(dest_addr));
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(ROS_PORT);
            inet_pton(AF_INET, ROS_IP, &dest_addr.sin_addr);
        }

        // Message formatting
        int len = 0;
        if (xSemaphoreTake(shared_data.motor_mutex, pdMS_TO_TICKS(100))) {
            len = snprintf(buffer, sizeof(buffer), "%d,%d,%lu",
                            shared_data.motors.pwm_channel_left,
                            shared_data.motors.pwm_channel_right,
                            seq_num++);
            xSemaphoreGive(shared_data.motor_mutex);
        }

        // Send with retries
        if (len > 0) {
            int retries = 0;
            while (retries < MAX_SEND_RETRIES) {
                int err = sendto(sock, buffer, len, 0,
                                (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                
                if (err >= 0) {
                    ESP_LOGD(TAG, "Sent: %s", buffer);
                    break;
                }
                
                ESP_LOGE(TAG, "Send error %d (attempt %d)", errno, retries+1);
                retries++;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            
            if (retries == MAX_SEND_RETRIES) {
                close(sock);
                sock = -1; // Force reconnect
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CLIENT_SEND_INTERVAL_MS));
    }
}

void udp_client_start(void)
{
    // Start UDP client task
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
}