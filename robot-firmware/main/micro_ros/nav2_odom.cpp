#include "micro_ros/nav2_odom.h"

#include <cmath>

#include "utils/angles.h"

#include "sdkconfig.h"

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kMaxDt = 0.5f;
    constexpr float kMinDt = 0.001f;
}

OdomIntegrator::OdomIntegrator(const DriveGeometry &geometry, const MotorConfig &config)
    : geometry_(geometry), config_(config), wheel_radius_m_((config.wheel_diameter_mm * 0.001f) * 0.5f)
{
    reset();
}

void OdomIntegrator::reset()
{
    state_ = {};
}

void OdomIntegrator::integrate(const WheelStates &wheels,
                               float dt_sec,
                               float imu_yaw_rad,
                               bool has_imu_yaw,
                               float &linear_mps,
                               float &angular_radps)
{
    float dt = dt_sec;
    if (dt <= kMinDt || dt > kMaxDt)
    {
        dt = 0.02f; // fallback to 50 Hz equivalent
    }

    const float left_rad_s = 0.5f * (wheels.velocity_rad_s[0] + wheels.velocity_rad_s[1]);
    const float right_rad_s = 0.5f * (wheels.velocity_rad_s[2] + wheels.velocity_rad_s[3]);

    const float v_left = left_rad_s * wheel_radius_m_;
    const float v_right = right_rad_s * wheel_radius_m_;

    linear_mps = 0.5f * (v_left + v_right);
    const float wheel_angular_radps = (geometry_.track_width_m > 1e-4f) ? ((v_right - v_left) / geometry_.track_width_m) : 0.0f;

    const double prev_theta = state_.theta_rad;

#if CONFIG_ROBOT_ODOM_FUSE_IMU_YAW
    if (has_imu_yaw)
    {
        const float theta_unwrapped = robot::angles::unwrap(static_cast<float>(state_.theta_rad), robot::angles::wrapPi(imu_yaw_rad));
        state_.theta_rad = static_cast<double>(theta_unwrapped);
        angular_radps = static_cast<float>((state_.theta_rad - prev_theta) / static_cast<double>(dt));
    }
    else
    {
        angular_radps = wheel_angular_radps;
        state_.theta_rad = static_cast<double>(static_cast<float>(state_.theta_rad) + angular_radps * dt);
    }
#else
    (void)imu_yaw_rad;
    (void)has_imu_yaw;
    angular_radps = wheel_angular_radps;
    state_.theta_rad = static_cast<double>(static_cast<float>(state_.theta_rad) + angular_radps * dt);
#endif

    const float yaw_rad = static_cast<float>(state_.theta_rad);
    state_.x_m += static_cast<double>(linear_mps * std::cos(yaw_rad) * dt);
    state_.y_m += static_cast<double>(linear_mps * std::sin(yaw_rad) * dt);
    state_.last_ts_us = wheels.timestamp_us;
}

void OdomIntegrator::fillOdometry(nav_msgs__msg__Odometry &odom_msg,
                                  const OrientationSample &orientation,
                                  float linear_mps,
                                  float angular_radps) const
{
    const float roll_rad = degToRad(orientation.orientation.roll);
    const float pitch_rad = degToRad(orientation.orientation.pitch);
    const float yaw_rad = degToRad(orientation.orientation.yaw);
    const auto q = yawToQuat(roll_rad, pitch_rad, yaw_rad);

    odom_msg.pose.pose.position.x = state_.x_m;
    odom_msg.pose.pose.position.y = state_.y_m;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = q;

    odom_msg.twist.twist.linear.x = linear_mps;
    odom_msg.twist.twist.linear.y = 0.0f;
    odom_msg.twist.twist.angular.z = angular_radps;

    odom_msg.pose.covariance[0] = 0.02;
    odom_msg.pose.covariance[7] = 0.02;
    odom_msg.pose.covariance[35] = 0.04;
    odom_msg.twist.covariance[0] = 0.05;
    odom_msg.twist.covariance[7] = 0.05;
    odom_msg.twist.covariance[35] = 0.08;
}

void OdomIntegrator::fillTransform(geometry_msgs__msg__TransformStamped &tf_msg,
                                   const OrientationSample &orientation) const
{
    const float roll_rad = degToRad(orientation.orientation.roll);
    const float pitch_rad = degToRad(orientation.orientation.pitch);
    const float yaw_rad = degToRad(orientation.orientation.yaw);
    const auto q = yawToQuat(roll_rad, pitch_rad, yaw_rad);

    tf_msg.transform.translation.x = state_.x_m;
    tf_msg.transform.translation.y = state_.y_m;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = q;
}

float OdomIntegrator::degToRad(float deg)
{
    return deg * (kPi / 180.0f);
}

geometry_msgs__msg__Quaternion OdomIntegrator::yawToQuat(float roll_rad, float pitch_rad, float yaw_rad)
{
    const float cy = std::cos(yaw_rad * 0.5f);
    const float sy = std::sin(yaw_rad * 0.5f);
    const float cr = std::cos(roll_rad * 0.5f);
    const float sr = std::sin(roll_rad * 0.5f);
    const float cp = std::cos(pitch_rad * 0.5f);
    const float sp = std::sin(pitch_rad * 0.5f);

    geometry_msgs__msg__Quaternion q{};
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}
