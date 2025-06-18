#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "common.h"

// UDP Configuration
#define UDP_BUFFER_SIZE 128             // Max packet size
#define UDP_IMU_PORT    3333            // Port for receiving IMU data
#define ROS_PORT        3334            // ROS UDP server port
#define ROS_IP          "machine IP"    // ROS machine IP

#define SOCKET_RX_TIMEOUT_MS 100
#define SERVER_TASK_DELAY_MS 2
#define CLIENT_SEND_INTERVAL_MS 5
#define MAX_SEND_RETRIES 3

/**
 * @brief Starts UDP server task
 */
void udp_server_start(void);

/**
 * @brief Sends motor commands to ROS
 */
void udp_client_start(void);