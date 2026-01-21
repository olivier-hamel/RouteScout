#include "orientation/mpu6050.h"

#include <cmath>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "MPU6050";

namespace
{
    constexpr float ACCEL_SENS = 16384.0f; // - Accel +/-2g => 16384 LSB/g
    constexpr float GYRO_SENS = 131.0f;    // - Gyro  +/-250 dps => 131 LSB/(deg/s)
    constexpr float COMPLEMENTARY_ALPHA = 0.98f;
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;   // Gestion d'alimentation
    constexpr uint8_t REG_SMPLRT_DIV = 0x19;   // Diviseur du sample rate
    constexpr uint8_t REG_CONFIG = 0x1A;       // DLPF (filtres) / config générale
    constexpr uint8_t REG_GYRO_CONFIG = 0x1B;  // Plage gyroscope
    constexpr uint8_t REG_ACCEL_CONFIG = 0x1C; // Plage accéléromètre
    constexpr uint8_t REG_INT_ENABLE = 0x38;   // Activation des interruptions
    constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B; // Début des registres accel (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
    constexpr uint8_t REG_GYRO_XOUT_H = 0x43;  // Début des registres gyro
}

Mpu6050::Mpu6050(I2CBus &bus, uint8_t address, gpio_num_t intPin)
    : bus_(bus), address_(address), int_pin_(intPin), device_(nullptr) {}

esp_err_t Mpu6050::init()
{
    uint8_t primary = address_;
    uint8_t alt = (address_ == 0x68) ? 0x69 : 0x68;
    uint8_t candidates[2] = {primary, alt};

    esp_err_t last_err = ESP_FAIL;
    for (uint8_t addr : candidates)
    {
        last_err = addOrReconnect(addr);
        if (last_err != ESP_OK)
        {
            ESP_LOGE(TAG, "Device add failed for address 0x%02X: %s", addr, esp_err_to_name(last_err));
            continue;
        }

        last_err = configure();
        if (last_err == ESP_OK)
        {
            address_ = addr;
            ESP_LOGI(TAG, "MPU6050 configured at address 0x%02X", addr);
            break;
        }
        else
        {
            ESP_LOGE(TAG, "Configure failed at address 0x%02X: %s", addr, esp_err_to_name(last_err));
        }
    }

    ESP_RETURN_ON_ERROR(last_err, TAG, "configure failed");
    ESP_RETURN_ON_ERROR(setupInterruptPin(), TAG, "int pin config failed");
    return ESP_OK;
}

esp_err_t Mpu6050::configure()
{
    // Réveil, sort du mode sleep (PWR_MGMT_1 =0 )
    ESP_RETURN_ON_ERROR(bus_.writeByte(device_, REG_PWR_MGMT_1, 0x00), TAG, "wake failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    // Sample rate : sample_rate = gyro_output_rate / (1 + SMPLRT_DIV)
    // Gyro output rate = 1kHz quand DLPF activé
    // Ici: 1000 / (1+7) = 125 Hz
    ESP_RETURN_ON_ERROR(bus_.writeByte(device_, REG_SMPLRT_DIV, 0x07), TAG, "smplrt failed");

    // CONFIG (DLPF) : 0x03 => DLPF_CFG=3 (filtre passe-bas numérique, fréquence selon datasheet)
    // Réduit bruit, augmente latence (compromis)
    ESP_RETURN_ON_ERROR(bus_.writeByte(device_, REG_CONFIG, 0x03), TAG, "config failed");

    // Gyro range : 0x00 => +/-250 dps
    // Correspond à GYRO_SENS = 131 LSB/(deg/s)
    ESP_RETURN_ON_ERROR(bus_.writeByte(device_, REG_GYRO_CONFIG, 0x00), TAG, "gyro cfg failed");

    // Accel range : 0x00 => +/-2g
    // Correspond à ACCEL_SENS = 16384 LSB/g
    ESP_RETURN_ON_ERROR(bus_.writeByte(device_, REG_ACCEL_CONFIG, 0x00), TAG, "accel cfg failed");

    // active l'interruption "DATA_RDY_EN"
    ESP_RETURN_ON_ERROR(bus_.writeByte(device_, REG_INT_ENABLE, 0x01), TAG, "int enable failed");

    return ESP_OK;
}

esp_err_t Mpu6050::addOrReconnect(uint8_t address)
{
    if (device_)
    {
        (void)bus_.removeDevice(device_);
        device_ = nullptr;
    }
    return bus_.addDevice(address, CONFIG_I2C_MASTER_FREQUENCY, &device_);
}

esp_err_t Mpu6050::setupInterruptPin()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << int_pin_), // Selectionne GPIO int
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    return gpio_config(&io_conf);
}

esp_err_t Mpu6050::readAcceleration(AccelReading &out)
{
    uint8_t buffer[6];
    ESP_RETURN_ON_ERROR(bus_.readBytes(device_, REG_ACCEL_XOUT_H, buffer, sizeof(buffer)), TAG, "acc read failed");

    int16_t raw_x = (buffer[0] << 8) | buffer[1];
    int16_t raw_y = (buffer[2] << 8) | buffer[3];
    int16_t raw_z = (buffer[4] << 8) | buffer[5];

    out.x_g = static_cast<float>(raw_x) / ACCEL_SENS;
    out.y_g = static_cast<float>(raw_y) / ACCEL_SENS;
    out.z_g = static_cast<float>(raw_z) / ACCEL_SENS;
    return ESP_OK;
}

esp_err_t Mpu6050::readGyroscope(GyroReading &out)
{
    uint8_t buffer[6];
    ESP_RETURN_ON_ERROR(bus_.readBytes(device_, REG_GYRO_XOUT_H, buffer, sizeof(buffer)), TAG, "gyro read failed");

    int16_t raw_x = (buffer[0] << 8) | buffer[1];
    int16_t raw_y = (buffer[2] << 8) | buffer[3];
    int16_t raw_z = (buffer[4] << 8) | buffer[5];

    out.x_dps = static_cast<float>(raw_x) / GYRO_SENS;
    out.y_dps = static_cast<float>(raw_y) / GYRO_SENS;
    out.z_dps = static_cast<float>(raw_z) / GYRO_SENS;
    return ESP_OK;
}

esp_err_t Mpu6050::readAccelGyro(AccelReading &accel_out, GyroReading &gyro_out)
{
    // The MPU6050 data registers are contiguous from 0x3B:
    // Accel(6) + Temp(2) + Gyro(6) = 14 bytes.
    uint8_t buffer[14];
    ESP_RETURN_ON_ERROR(bus_.readBytes(device_, REG_ACCEL_XOUT_H, buffer, sizeof(buffer)), TAG, "accel+gyro read failed");

    const int16_t ax = (buffer[0] << 8) | buffer[1];
    const int16_t ay = (buffer[2] << 8) | buffer[3];
    const int16_t az = (buffer[4] << 8) | buffer[5];
    const int16_t gx = (buffer[8] << 8) | buffer[9];
    const int16_t gy = (buffer[10] << 8) | buffer[11];
    const int16_t gz = (buffer[12] << 8) | buffer[13];

    accel_out.x_g = static_cast<float>(ax) / ACCEL_SENS;
    accel_out.y_g = static_cast<float>(ay) / ACCEL_SENS;
    accel_out.z_g = static_cast<float>(az) / ACCEL_SENS;

    gyro_out.x_dps = static_cast<float>(gx) / GYRO_SENS;
    gyro_out.y_dps = static_cast<float>(gy) / GYRO_SENS;
    gyro_out.z_dps = static_cast<float>(gz) / GYRO_SENS;
    return ESP_OK;
}
