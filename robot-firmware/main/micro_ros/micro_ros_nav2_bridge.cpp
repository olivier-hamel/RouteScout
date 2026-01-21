#include "micro_ros_nav2_bridge.h"

#include <sys/time.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include "micro_ros_nav2_bridge.h"

#include <sys/time.h>

#include <memory>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw/error_handling.h>
#include <rmw/qos_profiles.h>
#include <rmw_microros/rmw_microros.h>
#include "driver/uart.h"
#include <rmw_microros/time_sync.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/header.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>

#include "esp_timer.h"

#include "utils/angles.h"

#include "micro_ros/nav2_odom.h"
#include "micro_ros_serial_transport.h"

#include "sdkconfig.h"

#ifndef CONFIG_ROBOT_CMD_VEL_TIMEOUT_MS
#define CONFIG_ROBOT_CMD_VEL_TIMEOUT_MS 300
#endif

#ifndef CONFIG_ROBOT_TWIST_MAX_ANGULAR_MRADPS
#define CONFIG_ROBOT_TWIST_MAX_ANGULAR_MRADPS 2000
#endif

#ifndef CONFIG_ROBOT_TWIST_YAW_PID_KP_MILLI
#define CONFIG_ROBOT_TWIST_YAW_PID_KP_MILLI 1800
#endif

#ifndef CONFIG_ROBOT_TWIST_YAW_PID_KI_MILLI
#define CONFIG_ROBOT_TWIST_YAW_PID_KI_MILLI 0
#endif

#ifndef CONFIG_ROBOT_TWIST_YAW_PID_KD_MILLI
#define CONFIG_ROBOT_TWIST_YAW_PID_KD_MILLI 60
#endif

#ifndef CONFIG_ROBOT_TWIST_YAW_PID_MAX_CORRECTION_MRADPS
#define CONFIG_ROBOT_TWIST_YAW_PID_MAX_CORRECTION_MRADPS 1200
#endif

#ifndef CONFIG_ROBOT_TWIST_YAW_PID_INTEGRAL_LIMIT_MRAD
#define CONFIG_ROBOT_TWIST_YAW_PID_INTEGRAL_LIMIT_MRAD 2000
#endif

namespace
{
    static const char *TAG = "micro_ros_nav2";

    class Nav2Bridge;
    static Nav2Bridge *g_instance = nullptr;

#define RCCHECK(fn)                                                                                     \
    do                                                                                                  \
    {                                                                                                   \
        rcl_ret_t rc_ = (fn);                                                                           \
        if (rc_ != RCL_RET_OK)                                                                          \
        {                                                                                               \
            ESP_LOGE(TAG, "RCL error %d at %s:%d", (int)rc_, __FILE__, __LINE__);                      \
            vTaskDelete(nullptr);                                                                       \
        }                                                                                               \
    } while (0)

#define RCSOFTCHECK(fn)                                                                                  \
    do                                                                                                   \
    {                                                                                                    \
        rcl_ret_t rc_ = (fn);                                                                            \
        if (rc_ != RCL_RET_OK)                                                                           \
        {                                                                                                \
            ESP_LOGW(TAG, "RCL soft error %d at %s:%d", (int)rc_, __FILE__, __LINE__);                 \
        }                                                                                                \
    } while (0)

    class PidController
    {
    public:
        void setGains(float kp, float ki, float kd)
        {
            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
        }

        void setLimits(float max_output, float integral_limit)
        {
            max_output_ = (max_output >= 0.0f) ? max_output : 0.0f;
            integral_limit_ = (integral_limit >= 0.0f) ? integral_limit : 0.0f;
        }

        void reset()
        {
            integral_ = 0.0f;
            prev_error_ = 0.0f;
            has_prev_ = false;
        }

        float step(float error, float dt)
        {
            if (dt <= 1e-4f)
            {
                return 0.0f;
            }

            // Integrate with clamp.
            integral_ += error * dt;
            if (integral_limit_ > 0.0f)
            {
                integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
            }

            float deriv = 0.0f;
            if (has_prev_)
            {
                deriv = (error - prev_error_) / dt;
            }
            prev_error_ = error;
            has_prev_ = true;

            float out = (kp_ * error) + (ki_ * integral_) + (kd_ * deriv);
            if (max_output_ > 0.0f)
            {
                out = std::clamp(out, -max_output_, max_output_);
            }
            return out;
        }

    private:
        float kp_{0.0f};
        float ki_{0.0f};
        float kd_{0.0f};
        float integral_{0.0f};
        float prev_error_{0.0f};
        bool has_prev_{false};
        float max_output_{0.0f};
        float integral_limit_{0.0f};
    };

    class Nav2Bridge
    {
    public:
        esp_err_t start(DriveBase &drive_base, QueueHandle_t orientation_queue)
        {
            drive_base_ = &drive_base;
            orientation_queue_ = orientation_queue;

            if (orientation_queue_ == nullptr)
            {
                ESP_LOGE(TAG, "Orientation queue is null");
                return ESP_ERR_INVALID_ARG;
            }

            if (task_ != nullptr)
            {
                return ESP_ERR_INVALID_STATE;
            }

            BaseType_t created = xTaskCreate(
                &Nav2Bridge::taskEntry,
                "uros_nav2",
                CONFIG_ROBOT_MICRO_ROS_APP_STACK,
                this,
                CONFIG_ROBOT_MICRO_ROS_APP_TASK_PRIO,
                &task_);

            ESP_LOGI(TAG, "micro-ROS Nav2 task create result: %s", (created == pdPASS) ? "ok" : "failed");
            return (created == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
        }

    private:
        static void taskEntry(void *arg)
        {
            auto *self = static_cast<Nav2Bridge *>(arg);
            if (self)
            {
                self->run();
            }
            vTaskDelete(nullptr);
        }

        void run()
        {
            ESP_LOGI(TAG, "Starting micro-ROS Nav2 bridge");

            configureTransport();

            if (!waitForAgent(5000))
            {
                ESP_LOGE(TAG, "micro-ROS agent not reachable, Nav2 bridge not started");
                return;
            }

            rcl_allocator_t allocator = rcl_get_default_allocator();
            RCCHECK(rclc_support_init(&support_, 0, nullptr, &allocator));

            RCCHECK(rclc_node_init_default(&node_, "robot_nav2_bridge", "", &support_));

            // Synchronize MCU epoch time with the agent so TF/odom stamps are comparable to other
            // ROS 2 nodes (e.g., laser drivers). Without this, stamps can be near 0/1970 and RViz
            // message_filters will start dropping messages because transforms never match.
            syncTimeIfNeeded(true);

            // NOTE: nav_msgs/Odometry is a relatively large message. With Micro XRCE-DDS,
            // best-effort streams can be MTU-limited; using reliable avoids size-related
            // publish failures over serial.
            rmw_qos_profile_t odom_qos = rmw_qos_profile_default;
            odom_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            odom_qos.depth = 1;
            odom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
            odom_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            RCCHECK(rclc_publisher_init(
                &odom_pub_,
                &node_,
                ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                "odom",
                &odom_qos));

            rmw_qos_profile_t tf_qos = rmw_qos_profile_default;
            tf_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            tf_qos.depth = 1;
            tf_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
            tf_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            RCCHECK(rclc_publisher_init(
                &tf_pub_,
                &node_,
                ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
                "tf",
                &tf_qos));

            // For cmd_vel we care about the *latest* command. Best-effort + depth=1 avoids
            // building backlog on slow links and reduces end-to-end latency.
            rmw_qos_profile_t cmd_vel_qos = rmw_qos_profile_default;
            cmd_vel_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            cmd_vel_qos.depth = 1;
            cmd_vel_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
            cmd_vel_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            RCCHECK(rclc_subscription_init(
                &cmd_vel_sub_,
                &node_,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                "cmd_vel",
                &cmd_vel_qos));

            period_ms_ = CONFIG_ROBOT_MICRO_ROS_ODOM_PERIOD_MS;
            RCCHECK(rclc_timer_init_default2(
                &odom_timer_,
                &support_,
                RCL_MS_TO_NS(period_ms_),
                &Nav2Bridge::timerThunk,
                true));

            RCCHECK(rclc_executor_init(&executor_, &support_.context, 2, &allocator));
            RCCHECK(rclc_executor_add_subscription(&executor_, &cmd_vel_sub_, &cmd_vel_msg_, &Nav2Bridge::cmdVelThunk, ON_NEW_DATA));
            RCCHECK(rclc_executor_add_timer(&executor_, &odom_timer_));

            // One-time stack headroom log to catch low-stack crashes early
            UBaseType_t watermark_words = uxTaskGetStackHighWaterMark(nullptr);
            ESP_LOGI(TAG, "Nav2 task stack high water mark: %lu words (~%lu bytes)",
                     (unsigned long)watermark_words,
                     (unsigned long)(watermark_words * sizeof(StackType_t)));

            nav_msgs__msg__Odometry__init(&odom_msg_);
            tf2_msgs__msg__TFMessage__init(&tf_msg_);
            geometry_msgs__msg__Twist__init(&cmd_vel_msg_);
            geometry_msgs__msg__TransformStamped__Sequence__init(&tf_msg_.transforms, 1);
            initFrameIds();

            odom_integrator_ = std::make_unique<OdomIntegrator>(drive_base_->geometry(), drive_base_->config());

            while (true)
            {
                // Keep the spin loop responsive: timer callbacks still run at their configured
                // period, but subscriptions (cmd_vel) won’t be delayed by long waits/sleeps.
                rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(5));
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        static void timerThunk(rcl_timer_t *timer, int64_t last_call_time)
        {
            (void)timer;
            (void)last_call_time;
            if (g_instance)
            {
                g_instance->onTimer();
            }
        }

        static void cmdVelThunk(const void *msg)
        {
            if (g_instance)
            {
                g_instance->onCmdVel(msg);
            }
        }

        void onTimer()
        {
            const int64_t now_us = esp_timer_get_time();

            OrientationSample orientation{};
            if (peekOrientation(orientation))
            {
                latest_orientation_ = orientation;
                has_orientation_ = true;
            }

            // Apply latest cmd_vel with IMU-based yaw correction.
            applyControl(now_us);

            WheelStates wheels{};
            if (drive_base_->sampleWheelStates(wheels) != ESP_OK)
            {
                return;
            }

            if (!odom_integrator_)
            {
                return;
            }

            float dt_sec = 0.001f * static_cast<float>(period_ms_);
            if (odom_integrator_->state().last_ts_us > 0)
            {
                dt_sec = static_cast<float>(wheels.timestamp_us - odom_integrator_->state().last_ts_us) / 1e6f;
            }

            float linear_mps = 0.0f;
            float angular_radps = 0.0f;
            float imu_yaw_rad = 0.0f;
            bool has_yaw = false;
            if (has_orientation_)
            {
                imu_yaw_rad = OdomIntegrator::degToRad(latest_orientation_.orientation.yaw);
                imu_yaw_rad = robot::angles::wrapPi(imu_yaw_rad);
                has_yaw = true;
            }
            odom_integrator_->integrate(wheels, dt_sec, imu_yaw_rad, has_yaw, linear_mps, angular_radps);

            const OrientationSample used_orientation = has_orientation_ ? latest_orientation_ : orientation;
            publish(linear_mps, angular_radps, used_orientation);
        }

        void onCmdVel(const void *msg)
        {
            if (msg == nullptr)
            {
                return;
            }
            const auto *twist = static_cast<const geometry_msgs__msg__Twist *>(msg);

            last_cmd_linear_mps_ = static_cast<float>(twist->linear.x);
            last_cmd_angular_radps_ = static_cast<float>(twist->angular.z);
            last_cmd_ts_us_ = esp_timer_get_time();
            has_cmd_ = true;
        }

        void applyControl(int64_t now_us)
        {
            if (drive_base_ == nullptr)
            {
                return;
            }

            // Compute controller dt from actual timer cadence.
            float dt = 0.001f * static_cast<float>(period_ms_);
            if (last_control_ts_us_ > 0)
            {
                dt = static_cast<float>(now_us - last_control_ts_us_) / 1e6f;
            }
            last_control_ts_us_ = now_us;

            float cmd_lin = 0.0f;
            float cmd_ang = 0.0f;

            const int64_t timeout_us = static_cast<int64_t>(CONFIG_ROBOT_CMD_VEL_TIMEOUT_MS) * 1000LL;
            const bool cmd_fresh = has_cmd_ && (timeout_us <= 0 || (now_us - last_cmd_ts_us_) <= timeout_us);
            if (cmd_fresh)
            {
                cmd_lin = last_cmd_linear_mps_;
                cmd_ang = last_cmd_angular_radps_;
            }
            else
            {
                cmd_lin = 0.0f;
                cmd_ang = 0.0f;
            }

            // If stopped, don't fight noise.
            if (std::fabs(cmd_lin) < 1e-3f && std::fabs(cmd_ang) < 1e-3f)
            {
                yaw_pid_.reset();
                yaw_target_initialized_ = false;
                (void)drive_base_->setTwist(0.0f, 0.0f);
                return;
            }

#if CONFIG_ROBOT_TWIST_YAW_PID_ENABLE
            if (has_orientation_)
            {
                const float imu_yaw_wrapped = robot::angles::wrapPi(OdomIntegrator::degToRad(latest_orientation_.orientation.yaw));
                if (!imu_yaw_initialized_)
                {
                    imu_yaw_cont_rad_ = imu_yaw_wrapped;
                    imu_yaw_initialized_ = true;
                }
                else
                {
                    imu_yaw_cont_rad_ = robot::angles::unwrap(imu_yaw_cont_rad_, imu_yaw_wrapped);
                }

                if (!yaw_target_initialized_)
                {
                    yaw_target_cont_rad_ = imu_yaw_cont_rad_;
                    yaw_target_initialized_ = true;
                    yaw_pid_.reset();

                    const float kp = 0.001f * static_cast<float>(CONFIG_ROBOT_TWIST_YAW_PID_KP_MILLI);
                    const float ki = 0.001f * static_cast<float>(CONFIG_ROBOT_TWIST_YAW_PID_KI_MILLI);
                    const float kd = 0.001f * static_cast<float>(CONFIG_ROBOT_TWIST_YAW_PID_KD_MILLI);
                    yaw_pid_.setGains(kp, ki, kd);
                    yaw_pid_.setLimits(0.001f * static_cast<float>(CONFIG_ROBOT_TWIST_YAW_PID_MAX_CORRECTION_MRADPS),
                                       0.001f * static_cast<float>(CONFIG_ROBOT_TWIST_YAW_PID_INTEGRAL_LIMIT_MRAD));
                }

                // Desired yaw follows commanded angular velocity as a feed-forward trajectory.
                yaw_target_cont_rad_ += cmd_ang * dt;

                const float yaw_error = robot::angles::shortestDistance(imu_yaw_cont_rad_, yaw_target_cont_rad_);
                const float correction = yaw_pid_.step(yaw_error, dt);

                const float max_w = 0.001f * static_cast<float>(CONFIG_ROBOT_TWIST_MAX_ANGULAR_MRADPS);
                const float out_w = (max_w > 0.0f) ? std::clamp(cmd_ang + correction, -max_w, max_w) : (cmd_ang + correction);
                (void)drive_base_->setTwist(cmd_lin, out_w);
                return;
            }
#endif

            // Fallback: no IMU yaw available or PID disabled.
            (void)drive_base_->setTwist(cmd_lin, cmd_ang);
        }

        bool peekOrientation(OrientationSample &out)
        {
            if (orientation_queue_ == nullptr)
            {
                return false;
            }
            return xQueuePeek(orientation_queue_, &out, 0) == pdTRUE;
        }

        void publish(float linear_mps, float angular_radps, const OrientationSample &orientation)
        {
            if (!odom_integrator_)
            {
                return;
            }

            setTimestamp(odom_msg_.header);

            odom_integrator_->fillOdometry(odom_msg_, orientation, linear_mps, angular_radps);
            if (!publishSafe(&odom_pub_, &odom_msg_, "odom"))
            {
                return;
            }

            auto &tf = tf_msg_.transforms.data[0];
            tf.header.stamp = odom_msg_.header.stamp;
            odom_integrator_->fillTransform(tf, orientation);
            publishSafe(&tf_pub_, &tf_msg_, "tf");
        }

        bool publishSafe(rcl_publisher_t *pub, void *msg, const char *name)
        {
            const rcl_ret_t rc = rcl_publish(pub, msg, nullptr);
            if (rc != RCL_RET_OK)
            {
                agent_connected_ = false;
                // Capture and clear rcl error state ASAP (it can be overwritten by subsequent calls).
                const rcl_error_string_t err = rcl_get_error_string();
                if (err.str[0] != '\0')
                {
                    ESP_LOGW(TAG, "rcl_publish(%s) failed rc=%d: %s", name, (int)rc, err.str);
                }
                else
                {
                    ESP_LOGW(TAG, "rcl_publish(%s) failed rc=%d", name, (int)rc);
                }
                rcl_reset_error();

                const rmw_error_string_t rmw_err = rmw_get_error_string();
                if (rmw_err.str[0] != '\0')
                {
                    ESP_LOGW(TAG, "rmw error: %s", rmw_err.str);
                }
                rmw_reset_error();

                // Quick health check: if ping still succeeds, the transport/session is likely alive and
                // the failure is due to local state (publisher invalid, memory corruption, etc.).
                const rmw_ret_t ping_rc = rmw_uros_ping_agent(50, 1);
                ESP_LOGW(TAG, "Ping agent after publish failure: %s (%d)",
                         (ping_rc == RMW_RET_OK) ? "ok" : "fail", (int)ping_rc);

                logAgentWarningOnce(rc, name);
                return false;
            }
            agent_connected_ = true;
            return true;
        }

        void logAgentWarningOnce(rcl_ret_t rc, const char *name)
        {
            const TickType_t now = xTaskGetTickCount();
            const TickType_t interval = pdMS_TO_TICKS(2000);
            if (last_agent_warn_tick_ == 0 || (now - last_agent_warn_tick_) > interval)
            {
                ESP_LOGW(TAG, "micro-ROS publish failed for %s (rc=%d)", name, (int)rc);
                last_agent_warn_tick_ = now;
            }
        }

        bool waitForAgent(uint32_t timeout_ms)
        {
            const uint32_t step_ms = 200;
            uint32_t waited = 0;
            while (waited < timeout_ms)
            {
                if (rmw_uros_ping_agent(step_ms, 1) == RMW_RET_OK)
                {
                    agent_connected_ = true;
                    return true;
                }
                waited += step_ms;
                ESP_LOGW(TAG, "Waiting for micro-ROS agent... (%u/%u ms)", waited, timeout_ms);
            }
            return false;
        }

        void initFrameIds()
        {
            rosidl_runtime_c__String__assign(&odom_msg_.header.frame_id, "odom");
            rosidl_runtime_c__String__assign(&odom_msg_.child_frame_id, "base_link");

            if (tf_msg_.transforms.size > 0)
            {
                auto &tf = tf_msg_.transforms.data[0];
                rosidl_runtime_c__String__assign(&tf.header.frame_id, "odom");
                rosidl_runtime_c__String__assign(&tf.child_frame_id, "base_link");
            }
        }

        void configureTransport()
        {
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
            static uart_port_t uart_port = (uart_port_t)CONFIG_ROBOT_MICRO_ROS_UART_PORT;

#if CONFIG_ESP_CONSOLE_UART
            if ((int)uart_port == (int)CONFIG_ESP_CONSOLE_UART_NUM)
            {
                ESP_LOGE(TAG,
                         "micro-ROS UART transport is configured on UART%d, but the ESP-IDF console is also using UART%d. "
                         "This will corrupt the XRCE stream (logs/boot messages mix with micro-ROS frames) and cause intermittent agent disconnects. "
                         "Fix: set `ROBOT_MICRO_ROS_UART_PORT` to a different UART (recommended: 1) or move console to USB Serial/JTAG.",
                         (int)uart_port,
                         (int)CONFIG_ESP_CONSOLE_UART_NUM);
                vTaskDelete(nullptr);
                return;
            }
#endif

            ESP_LOGI(TAG,
                     "Configuring micro-ROS UART transport: uart_port=%d baud=%d TX=%d RX=%d",
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
            ESP_LOGE(TAG, "Custom transport not enabled (RMW_UXRCE_TRANSPORT_CUSTOM)");
            vTaskDelete(nullptr);
#endif
        }

        void syncTimeIfNeeded(bool force)
        {
            const TickType_t now = xTaskGetTickCount();
            const TickType_t min_interval = pdMS_TO_TICKS(2000);
            if (!force)
            {
                if (time_synced_)
                {
                    return;
                }
                if (last_time_sync_try_tick_ != 0 && (now - last_time_sync_try_tick_) < min_interval)
                {
                    return;
                }
            }

            last_time_sync_try_tick_ = now;
            const rmw_ret_t rc = rmw_uros_sync_session(500);
            if (rc == RMW_RET_OK)
            {
                time_synced_ = true;
                ESP_LOGI(TAG, "micro-ROS time sync: ok");
            }
            else
            {
                ESP_LOGW(TAG, "micro-ROS time sync failed (rc=%d)", (int)rc);
            }
        }

        void setTimestamp(std_msgs__msg__Header &header)
        {
            // Try to keep time in sync with the agent so message_filters can match TF to sensor stamps.
            if (!time_synced_)
            {
                syncTimeIfNeeded(false);
            }

            uint64_t ns = rmw_uros_epoch_nanos();
            if (ns == 0)
            {
                // Fallback: local epoch (may be unset). Prefer trying sync over trusting local time.
                struct timeval tv;
                gettimeofday(&tv, nullptr);
                ns = static_cast<uint64_t>(tv.tv_sec) * 1000000000ULL + static_cast<uint64_t>(tv.tv_usec) * 1000ULL;
            }
            header.stamp.sec = static_cast<int32_t>(ns / 1000000000ULL);
            header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000ULL);
        }

    private:
        DriveBase *drive_base_{nullptr};
        QueueHandle_t orientation_queue_{nullptr};
        TaskHandle_t task_{nullptr};

        rclc_support_t support_{};
        rcl_node_t node_{};
        rclc_executor_t executor_{};
        rcl_publisher_t odom_pub_{};
        rcl_publisher_t tf_pub_{};
        rcl_subscription_t cmd_vel_sub_{};
        rcl_timer_t odom_timer_{};

        nav_msgs__msg__Odometry odom_msg_{};
        tf2_msgs__msg__TFMessage tf_msg_{};
        geometry_msgs__msg__Twist cmd_vel_msg_{};

        OrientationSample latest_orientation_{};
        bool has_orientation_{false};
        uint32_t period_ms_{50};

        bool has_cmd_{false};
        float last_cmd_linear_mps_{0.0f};
        float last_cmd_angular_radps_{0.0f};
        int64_t last_cmd_ts_us_{0};

        int64_t last_control_ts_us_{0};
        bool imu_yaw_initialized_{false};
        float imu_yaw_cont_rad_{0.0f};

        bool yaw_target_initialized_{false};
        float yaw_target_cont_rad_{0.0f};
        PidController yaw_pid_{};

        bool agent_connected_{false};
        TickType_t last_agent_warn_tick_{0};

        bool time_synced_{false};
        TickType_t last_time_sync_try_tick_{0};

        std::unique_ptr<OdomIntegrator> odom_integrator_{};
    };

}

esp_err_t micro_ros_nav2_bridge_start(DriveBase &drive_base, QueueHandle_t orientation_queue)
{
#if !CONFIG_ROBOT_MICRO_ROS_NAV2_BRIDGE
    (void)drive_base;
    (void)orientation_queue;
    return ESP_OK;
#else
    static Nav2Bridge bridge;
    g_instance = &bridge;
    return bridge.start(drive_base, orientation_queue);
#endif
}
