#include "micro_ros_int32_publisher.h"

#include <stdio.h>

#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "micro_ros_serial_transport.h"

static const char *TAG = "micro_ros_int32_pub";

#define RCCHECK(fn)                                                                                 \
    do                                                                                                \
    {                                                                                                 \
        rcl_ret_t rc_ = (fn);                                                                         \
        if (rc_ != RCL_RET_OK)                                                                        \
        {                                                                                             \
            ESP_LOGE(TAG, "RCL error %d at %s:%d", (int)rc_, __FILE__, __LINE__);                     \
            vTaskDelete(NULL);                                                                        \
        }                                                                                             \
    } while (0)

#define RCSOFTCHECK(fn)                                                                              \
    do                                                                                                \
    {                                                                                                 \
        rcl_ret_t rc_ = (fn);                                                                         \
        if (rc_ != RCL_RET_OK)                                                                        \
        {                                                                                             \
            ESP_LOGW(TAG, "RCL soft error %d at %s:%d", (int)rc_, __FILE__, __LINE__);                \
        }                                                                                             \
    } while (0)

static rcl_publisher_t publisher;
static std_msgs__msg__Int32 msg;

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL)
    {
        return;
    }

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
}

static void micro_ros_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "micro-ROS task starting");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    ESP_LOGI(TAG, "Initializing rclc support...");
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    ESP_LOGI(TAG, "rclc support initialized");

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "robot_int32_publisher", "", &support));
    ESP_LOGI(TAG, "Node created, creating publisher on topic '%s'", "freertos_int32_publisher");

    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "freertos_int32_publisher"));

    rcl_timer_t timer;
    const unsigned int timer_timeout_ms = 1000;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout_ms),
        timer_callback,
        true));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;

    ESP_LOGI(TAG, "Publishing Int32 every %u ms", timer_timeout_ms);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t micro_ros_int32_publisher_start(void)
{
#if !CONFIG_ROBOT_MICRO_ROS_INT32_PUBLISHER
    return ESP_OK;
#else
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    static uart_port_t uart_port = (uart_port_t)CONFIG_ROBOT_MICRO_ROS_UART_PORT;

    ESP_LOGI(TAG,
             "Configuring micro-ROS custom UART transport: uart_port=%d baud=%d TX=%d RX=%d",
             (int)uart_port,
             (int)CONFIG_ROBOT_MICRO_ROS_UART_BAUDRATE,
             (int)CONFIG_MICROROS_UART_TXD,
             (int)CONFIG_MICROROS_UART_RXD);

    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        micro_ros_serial_open,
        micro_ros_serial_close,
        micro_ros_serial_write,
        micro_ros_serial_read);
#else
    return ESP_ERR_INVALID_STATE;
#endif

    BaseType_t ok = xTaskCreate(
        micro_ros_task,
        "uros_int32_pub",
        CONFIG_ROBOT_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_ROBOT_MICRO_ROS_APP_TASK_PRIO,
        NULL);

    ESP_LOGI(TAG, "micro-ROS task create result: %s", (ok == pdPASS) ? "ok" : "failed");
    return (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
#endif
}
