#pragma once

#include "orientation/mpu6050.h"

/**
 * @brief Interface pour les filtres
 */
struct OrientationFilter
{
    virtual ~OrientationFilter() = default;

    virtual void reset() = 0;
    virtual void update(const AccelReading &accel, const GyroReading &gyro, float dt_seconds) = 0;
    virtual Orientation getOrientation() const = 0;
};

class MadgwickFilter : public OrientationFilter
{
public:
    /**
     * @param beta Gain du filtre (force du feedback/correction). ~0.01 à 0.3 selon bruit/latence/usage
     */
    explicit MadgwickFilter(float beta = 0.1f);

    /**
     * @brief Reset état interne du filtre
     */
    void reset() override;

    /**
     * @brief Met à jour le filtre avec les nouvelles mesures
     *
     * @param accel      Mesures d'accélération en g
     * @param gyro       Mesures de gyroscope en deg/s
     * @param dt_seconds Nombre de secondes depuis la dernière update
     */
    void update(const AccelReading &accel, const GyroReading &gyro, float dt_seconds) override;

    /**
     * @brief Retourn orientation actuelle (roll/pitch/yaw en degrés)
     */
    Orientation getOrientation() const override;

private:
    float beta_;

    // État interne : quaternion d'orientation (q0 + i*q1 + j*q2 + k*q3).
    // Convention : q0 = partie scalaire, q1/q2/q3 = partie vectorielle.
    float q0_, q1_, q2_, q3_;
};
