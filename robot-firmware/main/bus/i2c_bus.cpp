#include "bus/i2c_bus.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include <vector>

static const char *TAG = "I2CBus";

I2CBus::I2CBus() : bus_(nullptr), initialized_(false), mutex_(nullptr) {}

I2CBus::~I2CBus()
{
    if (bus_)
    {
        (void)i2c_del_master_bus(bus_);
        bus_ = nullptr;
        initialized_ = false;
    }

    if (mutex_)
    {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

esp_err_t I2CBus::lock(int timeout_ms) const
{
    if (!mutex_)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void I2CBus::unlock() const
{
    if (mutex_)
    {
        (void)xSemaphoreGive(mutex_);
    }
}

esp_err_t I2CBus::begin()
{
    if (initialized_)
    {
        return ESP_OK;
    }

    if (!mutex_)
    {
        mutex_ = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(mutex_ != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create I2C mutex");
    }

    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = static_cast<gpio_num_t>(CONFIG_I2C_MASTER_SDA); // Pin SDA/SCL définies dans menuconif (sdkconfig)
    bus_config.scl_io_num = static_cast<gpio_num_t>(CONFIG_I2C_MASTER_SCL);
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &bus_), TAG, "Failed to init bus");
    initialized_ = true;
    return ESP_OK;
}

esp_err_t I2CBus::addDevice(uint8_t address, uint32_t clkHz, i2c_master_dev_handle_t *outHandle)
{
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Bus not initialized");

    ESP_RETURN_ON_ERROR(lock(), TAG, "I2C lock timeout");

    i2c_device_config_t dev_cfg{};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = address;
    dev_cfg.scl_speed_hz = clkHz;
    dev_cfg.scl_wait_us = 0;

    esp_err_t err = i2c_master_bus_add_device(bus_, &dev_cfg, outHandle);
    unlock();
    return err;
}

esp_err_t I2CBus::probeAddress(uint8_t address, int timeout_ms) const
{
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Bus not initialized");
    ESP_RETURN_ON_ERROR(lock(timeout_ms), TAG, "I2C lock timeout");
    esp_err_t err = i2c_master_probe(bus_, address, timeout_ms);
    unlock();
    return err;
}

esp_err_t I2CBus::readBytes(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len) const
{
    ESP_RETURN_ON_ERROR(lock(), TAG, "I2C lock timeout");

    // 1) TX: envoie le reg pour placer le pointeur du device
    // 2) RX: lit len octets dans data
    esp_err_t err = i2c_master_transmit_receive(
        handle,
        &reg, 1,   // TX buffer = &reg, taille = 1 octet
        data, len, // RX buffer = data, taille = len
        kDefaultIoTimeoutMs);
    unlock();
    return err;
}

esp_err_t I2CBus::writeBytes(i2c_master_dev_handle_t handle, uint8_t reg, const uint8_t *data, size_t len) const
{
    ESP_RETURN_ON_ERROR(lock(), TAG, "I2C lock timeout");

    // [REG][D0][D1]...[Dn]
    // On construit un buffer de taille 1+len.
    std::vector<uint8_t> buffer(1 + len);
    buffer[0] = reg;
    for (size_t i = 0; i < len; ++i)
    {
        buffer[1 + i] = data[i];
    }
    esp_err_t err = i2c_master_transmit(handle, buffer.data(), buffer.size(), kDefaultIoTimeoutMs);
    unlock();
    return err;
}

esp_err_t I2CBus::writeByte(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t value) const
{
    ESP_RETURN_ON_ERROR(lock(), TAG, "I2C lock timeout");

    // payload[0] = reg, payload[1] = value
    uint8_t payload[2] = {reg, value};
    esp_err_t err = i2c_master_transmit(handle, payload, sizeof(payload), kDefaultIoTimeoutMs);
    unlock();
    return err;
}

esp_err_t I2CBus::removeDevice(i2c_master_dev_handle_t handle)
{
    ESP_RETURN_ON_ERROR(lock(), TAG, "I2C lock timeout");
    esp_err_t err = i2c_master_bus_rm_device(handle);
    unlock();
    return err;
}
