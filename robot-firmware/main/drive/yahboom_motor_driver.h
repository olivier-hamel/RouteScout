
#pragma once

#include <array>
#include <cstdint>

#include "esp_err.h"
#include "bus/i2c_bus.h"

/**
 * @brief Configuration des moteurs
 */
struct MotorConfig
{
    uint8_t motor_type;       // 1 = 520
    uint16_t deadzone;        // PWM deadband
    uint16_t encoder_lines;   // Magnetic ring lines
    uint16_t reduction_ratio; // Gear ratio
    float wheel_diameter_mm;  // Diamètre des roues

    /**
     * @brief Config par defaut pour le moteur de type 520
     * @return Retourne la structure remplie
     */
    static MotorConfig Default520();
};

/**
 * @brief Driver I2C pour le controlleur de moteur
 */
class YahboomMotorDriver
{
public:
    /**
     * @brief Constructeur
     * @param bus Référence vers le bus I2C
     * @param address Adresse I2C du controleur (par defaut 0x26)
     * @param clk_hz Fréquence I2C
     */
    explicit YahboomMotorDriver(I2CBus &bus, uint8_t address = 0x26, uint32_t clk_hz = 400000);

    /**
     * @brief Initialise le driver
     */
    esp_err_t init();

    /**
     * @brief Applique une config moteur au controleur
     */
    esp_err_t applyConfig(const MotorConfig &config, bool apply_delays = true) const;

    /**
     * @brief Commande la vitesse des 4 moteurs (mm/s)
     * Utilise le PID interne du controlleur
     *
     * @param speed_mm Array des 4 vitesses {M1, M2, M3, M4}
     */
    esp_err_t setSpeed(const std::array<int16_t, 4> &speed_mm) const;

    /**
     * @brief Commande directement la rotation des 4 moteurs
     * @param speed_mm Array des 4 fréquences {M1, M2, M3, M4}
     */
    esp_err_t setPwm(const std::array<int16_t, 4> &pwm) const;

    /**
     * @brief Lit la tension de la batterie
     * @param voltage_v tension en volt, utilisé comme sortie
     */
    esp_err_t readBatteryVoltage(float &voltage_v) const;

    /**
     * @brief Lit le delta d'encoder pour chaque moteur
     * Variation depuis dernière lecture
     * @param delta_counts array des 4 deltas en sortie : {M1, M2, M3, M4}
     */
    esp_err_t readEncoderDelta(std::array<int16_t, 4> &delta_counts) const;

    /**
     * @brief Lit les encoders total pour chaque moteurs
     * @param total_counts array des 4 totaux en sortie : {M1, M2, M3, M4}
     */
    esp_err_t readEncoderTotals(std::array<int32_t, 4> &total_counts) const;

    /**
     * @brief Calcule le nombre de counts d'encodeur par tour de roue
     * (encoder_lines * 4) * reduction_ratio
     * @param config Config des moteurs
     * @return float Counts par révolution de roue
     */
    static float countsPerWheelRevolution(const MotorConfig &config);

    /**
     * @brief Calcule le nombre de counts d'encodeur par millimètre parcouru
     * @param config moteur
     * @return Count par millimètre
     */
    static float countsPerMillimeter(const MotorConfig &config);

    /**
     * @brief Convertit un delta d'encodeur en vitesse lineaire
     *
     * @param delta_counts delta d'encodeur sur l'intervalle
     * @param config Config moteur/roue
     * @param dt_seconds durée de l'intervalle
     * @return float Vitesse linéaire (mm/s)
     */
    static float deltaCountsToLinearSpeedMmPerSec(int16_t delta_counts, const MotorConfig &config, float dt_seconds = 0.01f);

private:
    I2CBus &bus_;
    uint8_t address_;
    uint32_t clk_hz_;
    i2c_master_dev_handle_t handle_;
    bool initialized_;

    esp_err_t writeU8(uint8_t reg, uint8_t value) const;
    esp_err_t writeU16Be(uint8_t reg, uint16_t value) const;
    esp_err_t writeFloatLe(uint8_t reg, float value) const;
    esp_err_t writeI16x4Be(uint8_t reg, const std::array<int16_t, 4> &values) const;

    esp_err_t readI16Be(uint8_t reg, int16_t &out) const;
    esp_err_t readTotal32(uint8_t reg_high, uint8_t reg_low, int32_t &out) const;
};
