#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * @brief Wrapper pour gérer le bus I2C.
 */

class I2CBus
{
public:
    I2CBus();
    ~I2CBus();

    I2CBus(const I2CBus &) = delete;
    I2CBus &operator=(const I2CBus &) = delete;

    /**
     * @brief Initialise le bus I2C
     */
    esp_err_t begin();

    /**
     * @brief Ajoute un périphérique I2C au bus et renvoie un handle de device
     *
     * @param address Adresse I2C 7 bits
     * @param clkHz Fréquence scl
     * @param outHandle Pointeur où stocker le handle du device créé
     */
    esp_err_t addDevice(uint8_t address, uint32_t clkHz, i2c_master_dev_handle_t *outHandle);

    /**
     * @brief Probe une adresse I2C (ACK/NACK) pour vérifier la présence d'un device
     *
     * @param address Adresse I2C
     * @param timeout_ms Timeout total (ms) pour acquisition du mutex et/ou sonde
     */
    esp_err_t probeAddress(uint8_t address, int timeout_ms = 50) const;

    /**
     * @brief Lit des octets depuis un registre
     *
     * @param handle Handle du périphérique I2C
     * @param reg Adresse du registre à lire
     * @param data Buffer de sortie qui reçoit les octets lus
     * @param len Nombre d'octet à lire
     */
    esp_err_t readBytes(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len) const;

    /**
     * @brief Écrit plusieurs octects dans un registre
     *
     * @param handle Handle du prériphérique I2C
     * @param ref Adresse du registre de départ
     * @param data Données à écrire
     * @param len Nombre d'octets à écrire depuis data
     */
    esp_err_t writeBytes(i2c_master_dev_handle_t handle, uint8_t reg, const uint8_t *data, size_t len) const;

    /**
     * @brief Écrit un seul octet dans un registre
     *
     * @param handle Handle du prériphérique I2C
     * @param ref Adresse du registre
     * @param value Valeur à écrire
     */
    esp_err_t writeByte(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t value) const;

    /**
     * @brief Retire un périphérique du bus
     *
     * @param handle Handle du périphérique à retirer
     */
    esp_err_t removeDevice(i2c_master_dev_handle_t handle);

private:
    i2c_master_bus_handle_t bus_;
    bool initialized_;
    mutable SemaphoreHandle_t mutex_;

    static constexpr int kDefaultLockTimeoutMs = 50;
    static constexpr int kDefaultIoTimeoutMs = 50;

    /**
     * @brief Prend le mutex
     *
     * @param timeout_ms Temps d'attente en ms pour obtenir le lock
     */
    esp_err_t lock(int timeout_ms = kDefaultLockTimeoutMs) const;

    /**
     * @brief Relâche le mutex
     */
    void unlock() const;
};
