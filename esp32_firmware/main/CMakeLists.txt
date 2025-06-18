idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES pid_controller
                    motor_driver
                    udp_network
                    imu_parser
                    wifi)
