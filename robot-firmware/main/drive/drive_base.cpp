#include "drive/drive_base.h"

#include <algorithm>
#include <cmath>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace
{
    static const char *TAG = "DriveBase";
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kDeltaSampleSeconds = 0.01f; // fenetre de 10 ms
}

DriveBase::DriveBase(YahboomMotorDriver &driver, MotorConfig config, DriveGeometry geometry) : driver_(driver), config_(config), geometry_(geometry) {}

esp_err_t DriveBase::setTwist(float linear_x_mps, float angular_z_radps)
{
    // mapping différentiel: left = v - w * b/2, right = v + w * b/2.
    const float half_track = geometry_.track_width_m * 0.5f;
    const float left_mps = linear_x_mps - angular_z_radps * half_track;
    const float right_mps = linear_x_mps + angular_z_radps * half_track;

    // Convertion en mm/s pour chaque roue
    std::array<float, 4> wheel_mmps{
        left_mps * 1000.0f,  // Avant gauche
        left_mps * 1000.0f,  // Arrière gauche
        right_mps * 1000.0f, // avant droit
        right_mps * 1000.0f  // Arrière droit
    };

    return setWheelLinearSpeedsMmps(wheel_mmps);
}

esp_err_t DriveBase::setWheelLinearSpeedsMmps(const std::array<float, 4> &wheel_mmps)
{
    // Build motor-channel command array [M1,M2,M3,M4] from wheel-order input [FL,RL,FR,RR].
    std::array<int16_t, 4> motor_cmd{};
    for (size_t wheel_idx = 0; wheel_idx < wheel_mmps.size(); ++wheel_idx)
    {
        const uint8_t motor_idx = kWheelToMotor[wheel_idx];
        const float signed_mmps = wheel_mmps[wheel_idx] * static_cast<float>(kWheelSign[wheel_idx]);
        motor_cmd[motor_idx] = mmpsToRegister(signed_mmps);
    }
    return driver_.setSpeed(motor_cmd);
}

esp_err_t DriveBase::setWheelPwm(const std::array<int16_t, 4> &pwm)
{
    // Map wheel-order PWM to motor-channel order.
    std::array<int16_t, 4> motor_pwm{};
    for (size_t wheel_idx = 0; wheel_idx < pwm.size(); ++wheel_idx)
    {
        const uint8_t motor_idx = kWheelToMotor[wheel_idx];
        const int16_t signed_pwm = static_cast<int16_t>(pwm[wheel_idx] * kWheelSign[wheel_idx]);
        motor_pwm[motor_idx] = signed_pwm;
    }
    return driver_.setPwm(motor_pwm);
}

esp_err_t DriveBase::sampleWheelStates(WheelStates &out)
{
    std::array<int32_t, 4> totals{};
    std::array<int16_t, 4> deltas{};
    float battery = 0.0f;

    ESP_RETURN_ON_ERROR(driver_.readEncoderTotals(totals), TAG, "Read totals failed");
    ESP_RETURN_ON_ERROR(driver_.readEncoderDelta(deltas), TAG, "Read deltas failed");
    ESP_RETURN_ON_ERROR(driver_.readBatteryVoltage(battery), TAG, "Read battery failed");

    // Convert from motor-channel order to wheel order, and apply per-wheel sign correction.
    for (size_t motor_idx = 0; motor_idx < totals.size(); ++motor_idx)
    {
        const uint8_t wheel_idx = kMotorToWheel[motor_idx];
        const int32_t signed_total = totals[motor_idx] * static_cast<int32_t>(kWheelSign[wheel_idx]);
        const int16_t signed_delta = static_cast<int16_t>(deltas[motor_idx] * kWheelSign[wheel_idx]);
        out.position_rad[wheel_idx] = wheelCountsToRadians(signed_total);
        out.velocity_rad_s[wheel_idx] = deltaCountsToRadPerSec(signed_delta, kDeltaSampleSeconds);
    }

    out.battery_v = battery;
    out.timestamp_us = esp_timer_get_time();
    return ESP_OK;
}

float DriveBase::wheelCountsToRadians(int32_t counts) const
{
    const float counts_per_rev = YahboomMotorDriver::countsPerWheelRevolution(config_);
    const float revs = static_cast<float>(counts) / counts_per_rev;
    return revs * 2.0f * kPi;
}

float DriveBase::deltaCountsToRadPerSec(int16_t delta_counts, float dt_seconds) const
{
    const float counts_per_rev = YahboomMotorDriver::countsPerWheelRevolution(config_);
    const float revs = static_cast<float>(delta_counts) / counts_per_rev;
    return (revs * 2.0f * kPi) / dt_seconds;
}

int16_t DriveBase::mmpsToRegister(float mmps) const
{
    const float clamped = std::clamp(mmps, -geometry_.max_linear_mmps, geometry_.max_linear_mmps);
    // Le registre de vitesse est big-endian [-1000, 1000]
    const float limited = std::clamp(clamped, -1000.0f, 1000.0f);
    return static_cast<int16_t>(std::lround(limited));
}