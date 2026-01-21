#pragma once

#include <cstdint>

#include "drive/drive_base.h"
#include "orientation/orientation_task.h"

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>

struct OdomState
{
    double x_m{0.0};
    double y_m{0.0};
    double theta_rad{0.0};
    int64_t last_ts_us{0};
};

class OdomIntegrator
{
public:
    OdomIntegrator(const DriveGeometry &geometry, const MotorConfig &config);

    void reset();

    /**
     * @brief Integrate wheel states using differential drive model, optionally fusing IMU yaw.
     *
     * When IMU yaw is provided (has_imu_yaw=true) and enabled via config, theta follows IMU,
     * and angular velocity is derived from IMU yaw delta. This keeps pose accurate even if wheels
     * are not perfectly straight/aligned.
     */
    void integrate(const WheelStates &wheels,
                   float dt_sec,
                   float imu_yaw_rad,
                   bool has_imu_yaw,
                   float &linear_mps,
                   float &angular_radps);

    void fillOdometry(nav_msgs__msg__Odometry &odom_msg,
                      const OrientationSample &orientation,
                      float linear_mps,
                      float angular_radps) const;

    void fillTransform(geometry_msgs__msg__TransformStamped &tf_msg,
                       const OrientationSample &orientation) const;

    const OdomState &state() const { return state_; }

    static float degToRad(float deg);

private:
    static geometry_msgs__msg__Quaternion yawToQuat(float roll_rad, float pitch_rad, float yaw_rad);

    const DriveGeometry geometry_;
    const MotorConfig config_;
    const float wheel_radius_m_;
    OdomState state_{};
};
