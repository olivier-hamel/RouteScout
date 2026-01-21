#pragma once

#include "esp_err.h"
#include "driver/gpio.h"

#include "bus/i2c_bus.h"

/**
 * @brief Orientation (Euleur) en degrés
 */
struct Orientation
{
    float roll;  // degrees
    float pitch; // degrees
    float yaw;   // degrees
};

/**
 * @brief Mesure d'accélération (g)
 */
struct AccelReading
{
    float x_g;
    float y_g;
    float z_g;
};

/**
 * @brief Mesure en degrés/secondes (deg/s)
 */
struct GyroReading
{
    float x_dps;
    float y_dps;
    float z_dps;
};

/**
 * @brief Driver MPU6050 sur I2C
 *
 * - Se connect au composant
 * - Configure le mpu6050
 * - Lit l'accélération et gyroscope
 * - Expose le GPIO d'interruption
 */
class Mpu6050
{
public:
    /**
     * @brief Constructeur
     *
     * @param bus     Référence vers le bus I2C
     * @param address Adress I2C (0x68 par défaut avec 0x69 en backup)
     * @param intPin  GPIO utilisé pour l'interruption
     */
    explicit Mpu6050(I2CBus &bus, uint8_t address = 0x68, gpio_num_t intPin = GPIO_NUM_2);

    /**
     * @brief Initialise la communication et le capteur
     */
    esp_err_t init();

    /**
     * @brief Lit l'accélération (x, y, z) et la convertit en g.
     *
     * @param out Structure de sortie remplie si succès
     */
    esp_err_t readAcceleration(AccelReading &out);

    /**
     * @brief Lit le gyroscope (x, y, z) et le convertir en deg/s.
     *
     * @param out Structure de sortie remplie si succès
     */
    esp_err_t readGyroscope(GyroReading &out);

    /**
     * @brief Reads accel + gyro in one I2C transaction (lower latency, less bus contention).
     */
    esp_err_t readAccelGyro(AccelReading &accel_out, GyroReading &gyro_out);

    /**
     * @brief Retourne la pin GPIO pour l'interruption
     */
    gpio_num_t intPin() const { return int_pin_; }

private:
    /**
     * @brief Écrit les registres de config (réveil, sample rate, DLPF, ranges, etc.)
     */
    esp_err_t configure();

    /**
     * @brief Configure le GPIO d'interruption
     */
    esp_err_t setupInterruptPin();

    /**
     * @brief (Re)crée le handle device_ sur une adresse donnée
     *
     * Si device_ existe déjà, on le retire du bus puis on l'ajoute à nouveau
     */
    esp_err_t addOrReconnect(uint8_t address);

    I2CBus &bus_;
    uint8_t address_;
    gpio_num_t int_pin_;
    i2c_master_dev_handle_t device_;
};
