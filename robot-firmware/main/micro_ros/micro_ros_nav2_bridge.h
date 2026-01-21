#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "drive/drive_base.h"
#include "orientation/orientation_task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Starts the micro-ROS Nav2 bridge (odom + tf + /cmd_vel).
 *
 * @param drive_base        DriveBase used to command the robot and read wheel states.
 * @param orientation_queue Queue providing latest OrientationSample from the IMU task.
 */
esp_err_t micro_ros_nav2_bridge_start(DriveBase &drive_base, QueueHandle_t orientation_queue);

#ifdef __cplusplus
}
#endif
