#include "yahboom_motor_driver.h"

#include <array>
#include <cmath>
#include <cstring>
#include <utility>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
    static const char *TAG = "YahboomMotor";

    struct Reg
    {
        static constexpr uint8_t kMotorType = 0x01;
        static constexpr uint8_t kDeadzone = 0x02;
        static constexpr uint8_t kLines = 0x03;
        static constexpr uint8_t kReductionRatio = 0x04;
        static constexpr uint8_t kWheelDiameter = 0x06;
        static constexpr uint8_t kSpeed = 0x06;
        static constexpr uint8_t kPwm = 0x07;
        static constexpr uint8_t kBattery = 0x08;

        static constexpr uint8_t kDeltaM1 = 0x10;
        static constexpr uint8_t kDeltaM2 = 0x11;
        static constexpr uint8_t kDeltaM3 = 0x12;
        static constexpr uint8_t kDeltaM4 = 0x13;

        static constexpr uint8_t kM1High = 0x20;
        static constexpr uint8_t kM1Low = 0x21;
        static constexpr uint8_t kM2High = 0x22;
        static constexpr uint8_t kM2Low = 0x23;
        static constexpr uint8_t kM3High = 0x24;
        static constexpr uint8_t kM3Low = 0x25;
        static constexpr uint8_t kM4High = 0x26;
        static constexpr uint8_t kM4Low = 0x27;
    };

    constexpr float kPi = 3.14159265358979323846f;
    constexpr TickType_t kConfigDelayTicks = pdMS_TO_TICKS(50);
} // namespace

MotorConfig MotorConfig::Default520()
{
    MotorConfig cfg{};
    cfg.motor_type = 1;
    cfg.deadzone = 1900;
    cfg.encoder_lines = 11;
    cfg.reduction_ratio = 40;
    cfg.wheel_diameter_mm = 67.0f;
    return cfg;
}

YahboomMotorDriver::YahboomMotorDriver(I2CBus &bus, uint8_t address, uint32_t clk_hz)
    : bus_(bus), address_(address), clk_hz_(clk_hz), handle_(nullptr), initialized_(false) {}

esp_err_t YahboomMotorDriver::init()
{
    if (initialized_)
    {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(bus_.addDevice(address_, clk_hz_, &handle_), TAG, "Failed to add device 0x%02X", address_);
    initialized_ = true;
    return ESP_OK;
}

esp_err_t YahboomMotorDriver::applyConfig(const MotorConfig &config, bool apply_delays) const
{
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Driver not initialized");
    ESP_RETURN_ON_ERROR(writeU8(Reg::kMotorType, config.motor_type), TAG, "Write motor type failed");
    if (apply_delays)
        vTaskDelay(kConfigDelayTicks);

    ESP_RETURN_ON_ERROR(writeU16Be(Reg::kReductionRatio, config.reduction_ratio), TAG, "Write ratio failed");
    if (apply_delays)
        vTaskDelay(kConfigDelayTicks);

    ESP_RETURN_ON_ERROR(writeU16Be(Reg::kLines, config.encoder_lines), TAG, "Write lines failed");
    if (apply_delays)
        vTaskDelay(kConfigDelayTicks);

    ESP_RETURN_ON_ERROR(writeFloatLe(Reg::kWheelDiameter, config.wheel_diameter_mm), TAG, "Write diameter failed");
    if (apply_delays)
        vTaskDelay(kConfigDelayTicks);

    ESP_RETURN_ON_ERROR(writeU16Be(Reg::kDeadzone, config.deadzone), TAG, "Write deadzone failed");
    if (apply_delays)
        vTaskDelay(kConfigDelayTicks);

    ESP_LOGI(TAG, "Applied Yahboom config: type=%u, ratio=%u, lines=%u, wheel=%.2f mm, deadzone=%u",
             config.motor_type, config.reduction_ratio, config.encoder_lines, config.wheel_diameter_mm, config.deadzone);
    return ESP_OK;
}

esp_err_t YahboomMotorDriver::setSpeed(const std::array<int16_t, 4> &speed_mm) const
{
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Driver not initialized");
    return writeI16x4Be(Reg::kSpeed, speed_mm);
}

esp_err_t YahboomMotorDriver::setPwm(const std::array<int16_t, 4> &pwm) const
{
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Driver not initialized");
    return writeI16x4Be(Reg::kPwm, pwm);
}

esp_err_t YahboomMotorDriver::readBatteryVoltage(float &voltage_v) const
{
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Driver not initialized");
    uint8_t buf[2] = {0};
    ESP_RETURN_ON_ERROR(bus_.readBytes(handle_, Reg::kBattery, buf, sizeof(buf)), TAG, "Read battery failed");
    uint16_t raw = static_cast<uint16_t>(buf[0] << 8 | buf[1]);
    voltage_v = static_cast<float>(raw) / 10.0f;
    return ESP_OK;
}

esp_err_t YahboomMotorDriver::readEncoderDelta(std::array<int16_t, 4> &delta_counts) const
{
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Driver not initialized");

    const std::array<uint8_t, 4> regs{Reg::kDeltaM1, Reg::kDeltaM2, Reg::kDeltaM3, Reg::kDeltaM4};
    for (size_t i = 0; i < regs.size(); ++i)
    {
        ESP_RETURN_ON_ERROR(readI16Be(regs[i], delta_counts[i]), TAG, "Read delta failed idx=%u", static_cast<unsigned>(i));
    }
    return ESP_OK;
}

esp_err_t YahboomMotorDriver::readEncoderTotals(std::array<int32_t, 4> &total_counts) const
{
    ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG, "Driver not initialized");

    const std::array<std::pair<uint8_t, uint8_t>, 4> regs{{
        {Reg::kM1High, Reg::kM1Low},
        {Reg::kM2High, Reg::kM2Low},
        {Reg::kM3High, Reg::kM3Low},
        {Reg::kM4High, Reg::kM4Low},
    }};

    for (size_t i = 0; i < regs.size(); ++i)
    {
        ESP_RETURN_ON_ERROR(readTotal32(regs[i].first, regs[i].second, total_counts[i]), TAG, "Read total failed idx=%u", static_cast<unsigned>(i));
    }
    return ESP_OK;
}

float YahboomMotorDriver::countsPerWheelRevolution(const MotorConfig &config)
{
    return static_cast<float>(config.encoder_lines) * 4.0f * static_cast<float>(config.reduction_ratio);
}

float YahboomMotorDriver::countsPerMillimeter(const MotorConfig &config)
{
    const float counts = countsPerWheelRevolution(config);
    const float circumference_mm = kPi * config.wheel_diameter_mm;
    return counts / circumference_mm;
}

float YahboomMotorDriver::deltaCountsToLinearSpeedMmPerSec(int16_t delta_counts, const MotorConfig &config, float dt_seconds)
{
    const float counts = static_cast<float>(delta_counts);
    const float revs = counts / countsPerWheelRevolution(config);
    const float circumference_mm = kPi * config.wheel_diameter_mm;
    const float distance_mm = revs * circumference_mm;
    return distance_mm / dt_seconds;
}

esp_err_t YahboomMotorDriver::writeU8(uint8_t reg, uint8_t value) const
{
    return bus_.writeByte(handle_, reg, value);
}

esp_err_t YahboomMotorDriver::writeU16Be(uint8_t reg, uint16_t value) const
{
    uint8_t payload[2];
    payload[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    payload[1] = static_cast<uint8_t>(value & 0xFF);
    return bus_.writeBytes(handle_, reg, payload, sizeof(payload));
}

esp_err_t YahboomMotorDriver::writeFloatLe(uint8_t reg, float value) const
{
    uint8_t payload[4];
    std::memcpy(payload, &value, sizeof(value));
    return bus_.writeBytes(handle_, reg, payload, sizeof(payload));
}

esp_err_t YahboomMotorDriver::writeI16x4Be(uint8_t reg, const std::array<int16_t, 4> &values) const
{
    uint8_t payload[8];
    for (size_t i = 0; i < values.size(); ++i)
    {
        payload[2 * i] = static_cast<uint8_t>((values[i] >> 8) & 0xFF);
        payload[2 * i + 1] = static_cast<uint8_t>(values[i] & 0xFF);
    }
    return bus_.writeBytes(handle_, reg, payload, sizeof(payload));
}

esp_err_t YahboomMotorDriver::readI16Be(uint8_t reg, int16_t &out) const
{
    uint8_t buf[2] = {0};
    ESP_RETURN_ON_ERROR(bus_.readBytes(handle_, reg, buf, sizeof(buf)), TAG, "Read int16 failed");
    out = static_cast<int16_t>(static_cast<uint16_t>(buf[0] << 8 | buf[1]));
    return ESP_OK;
}

esp_err_t YahboomMotorDriver::readTotal32(uint8_t reg_high, uint8_t reg_low, int32_t &out) const
{
    uint8_t hi[2] = {0};
    uint8_t lo[2] = {0};
    ESP_RETURN_ON_ERROR(bus_.readBytes(handle_, reg_high, hi, sizeof(hi)), TAG, "Read high failed");
    ESP_RETURN_ON_ERROR(bus_.readBytes(handle_, reg_low, lo, sizeof(lo)), TAG, "Read low failed");
    out = static_cast<int32_t>((static_cast<uint32_t>(hi[0]) << 24) |
                               (static_cast<uint32_t>(hi[1]) << 16) |
                               (static_cast<uint32_t>(lo[0]) << 8) |
                               static_cast<uint32_t>(lo[1]));
    return ESP_OK;
}