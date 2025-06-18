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

#define TAG "UDP_SERVER"

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[UDP_BUFFER_SIZE];
    int sock = -1;
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_IMU_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    // Create socket
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        goto cleanup;
    }

    // Set socket timeout
    struct timeval timeout = {
        .tv_sec = SOCKET_RX_TIMEOUT_MS / 1000,
        .tv_usec = (SOCKET_RX_TIMEOUT_MS % 1000) * 1000
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // Enable address reuse
    int enable = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

    // Bind socket
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket: errno %d", errno);
        goto cleanup;
    }

    ESP_LOGI(TAG, "UDP server started on port %d", UDP_IMU_PORT);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                          (struct sockaddr *)&source_addr, &addr_len);

        if (len < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            }
            vTaskDelay(pdMS_TO_TICKS(SERVER_TASK_DELAY_MS));
            continue;
        }

        // Null-terminate received data
        rx_buffer[len] = '\0';
        
        ESP_LOGI(TAG, "Raw IMU: %.*s", len, rx_buffer);
        
        // Log received data (limit to first 64 chars for readability)
        ESP_LOGI(TAG, "Received %d bytes from %s: %.*s%s", len,
                inet_ntoa(source_addr.sin_addr),
                MIN(len, 64), rx_buffer,
                len > 64 ? "..." : "");

        // Update shared data with mutex protection
        if (xSemaphoreTake(shared_data.imu_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            strlcpy(shared_data.imu.raw_buffer, rx_buffer, sizeof(shared_data.imu.raw_buffer));
            shared_data.imu.last_update = xTaskGetTickCount();
            xSemaphoreGive(shared_data.imu_mutex);
        }
        vTaskDelay(1);
    }

    cleanup:
        if (sock != -1) {
            close(sock);
        }
        vTaskDelete(NULL);
}

void udp_server_start(void)
{
    // Start UDP server task
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
}