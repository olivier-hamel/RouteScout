#include "orientation/orientation_filter.h"

#include <cmath>

static float invSqrt(float x)
{
    return 1.0f / std::sqrt(x);
}

MadgwickFilter::MadgwickFilter(float beta) : beta_(beta)
{
    // Info ici : https://medium.com/@k66115704/imu-madgwick-filter-explanation-556fbe7f02e3
    reset();
}

void MadgwickFilter::reset()
{
    q0_ = 1.0f;
    q1_ = q2_ = q3_ = 0.0f;
}

Orientation MadgwickFilter::getOrientation() const
{
    // Conversion quaternion -> angles d'Euler
    Orientation o{};

    // Roll
    float sinr_cosp = 2.0f * (q0_ * q1_ + q2_ * q3_);
    float cosr_cosp = 1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_);
    o.roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0f / static_cast<float>(M_PI);

    // Pitch
    float sinp = 2.0f * (q0_ * q2_ - q3_ * q1_);
    if (std::fabs(sinp) >= 1.0f)
    {
        o.pitch = std::copysign(90.0f, sinp);
    }
    else
    {
        o.pitch = std::asin(sinp) * 180.0f / static_cast<float>(M_PI);
    }

    // yaw
    float siny_cosp = 2.0f * (q0_ * q3_ + q1_ * q2_);
    float cosy_cosp = 1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_);
    o.yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0f / static_cast<float>(M_PI);
    return o;
}

void MadgwickFilter::update(const AccelReading &accel, const GyroReading &gyro, float dt)
{
    float ax = accel.x_g;
    float ay = accel.y_g;
    float az = accel.z_g;
    float gx = gyro.x_dps * static_cast<float>(M_PI) / 180.0f;
    float gy = gyro.y_dps * static_cast<float>(M_PI) / 180.0f;
    float gz = gyro.z_dps * static_cast<float>(M_PI) / 180.0f;

    if (ax == 0.0f && ay == 0.0f && az == 0.0f)
    {
        return; // invalid accel
    }

    // Normalisation
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Variable auxiliaires
    float _2q0 = 2.0f * q0_;
    float _2q1 = 2.0f * q1_;
    float _2q2 = 2.0f * q2_;
    float _2q3 = 2.0f * q3_;
    float _4q0 = 4.0f * q0_;
    float _4q1 = 4.0f * q1_;
    float _4q2 = 4.0f * q2_;
    float _8q1 = 8.0f * q1_;
    float _8q2 = 8.0f * q2_;
    float q0q0 = q0_ * q0_;
    float q1q1 = q1_ * q1_;
    float q2q2 = q2_ * q2_;
    float q3q3 = q3_ * q3_;

    // Gradient decent
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Feedback
    float qDot0 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz) - beta_ * s0;
    float qDot1 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy) - beta_ * s1;
    float qDot2 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx) - beta_ * s2;
    float qDot3 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx) - beta_ * s3;

    // Intégration
    q0_ += qDot0 * dt;
    q1_ += qDot1 * dt;
    q2_ += qDot2 * dt;
    q3_ += qDot3 * dt;

    // Normalisation
    recipNorm = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= recipNorm;
    q1_ *= recipNorm;
    q2_ *= recipNorm;
    q3_ *= recipNorm;
}
