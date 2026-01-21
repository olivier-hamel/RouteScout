#pragma once

#include <array>
#include <cstdint>

#include "esp_err.h"
#include "drive/yahboom_motor_driver.h"

/** @brief Index des roues */
enum class Wheel : uint8_t
{
    FrontLeft = 0,  /** @brief Roue avant gauche. */
    RearLeft = 1,   /** @brief Roue arrière gauche. */
    FrontRight = 2, /** @brief Roue avant droite. */
    RearRight = 3   /** @brief Roue arrière droite. */
};

/** @brief Paramètre géométrique de la base */
struct DriveGeometry
{
    float track_width_m;   /** @brief Distance entre les centres des roues gauche et droite (mètres). */
    float max_linear_mmps; /** @brief Saturation de la consigne de vitesse linéaire roue (mm/s). */
};

/**
 * @brief États estimés des roues + batterie
 */
struct WheelStates
{
    std::array<float, 4> position_rad{};   /** @brief Angles des roues (rad). */
    std::array<float, 4> velocity_rad_s{}; /** @brief Vitesses angulaires des roues (rad/s). */
    float battery_v{0.0f};                 /** @brief Tension mesurée (V). */
    int64_t timestamp_us{0};               /** @brief Timestamp de l’échantillon (microsecondes, esp_timer). */
};

class DriveBase
{
public:
    DriveBase(YahboomMotorDriver &driver, MotorConfig config, DriveGeometry geometry);

    /**
     * @brief Applique une commande de twist (vitesse linéaire(v) + vitesse angulaire(w))
     * vitesse gauche = v - w * (b/2)
     * vitesse droit = v + w * (b/2)
     * Où b = track_with_m
     *
     * @param linear_x_mps vitesse lineraire
     * @param angular_z_radps vitesse angulaire autour de l'axe verticale
     */
    esp_err_t setTwist(float linear_x_mps, float angular_z_radps);

    esp_err_t setWheelLinearSpeedsMmps(const std::array<float, 4> &wheel_mmps);
    esp_err_t setWheelPwm(const std::array<int16_t, 4> &pwm);
    esp_err_t sampleWheelStates(WheelStates &out);

    const MotorConfig &config() const { return config_; }
    const DriveGeometry &geometry() const { return geometry_; };

private:
    YahboomMotorDriver &driver_;
    MotorConfig config_;
    DriveGeometry geometry_;

    // Wheel array order used throughout this firmware (and by OdomIntegrator):
    // [0]=FrontLeft, [1]=RearLeft, [2]=FrontRight, [3]=RearRight.
    // Yahboom motor driver arrays are in motor-channel order: [M1, M2, M3, M4].
    // This mapping matches the user's wiring:
    //   FR : M2
    //   FL : M4
    //   RL : M3
    //   RR : M1
    static constexpr std::array<uint8_t, 4> kWheelToMotor{{3, 2, 1, 0}}; // wheel_idx -> motor_idx (M1=0..M4=3)
    static constexpr std::array<uint8_t, 4> kMotorToWheel{{3, 2, 1, 0}}; // motor_idx -> wheel_idx
    // Sign correction is in wheel-order: [FL, RL, FR, RR].
    // With the wiring above, M4 corresponds to FL (wheel_idx 0).
    static constexpr std::array<int8_t, 4> kWheelSign{{-1, -1, -1, -1}}; // M4 (FL) direction inverted

    float wheelCountsToRadians(int32_t counts) const;
    float deltaCountsToRadPerSec(int16_t delta_counts, float dt_seconds) const;
    int16_t mmpsToRegister(float mmps) const;
};