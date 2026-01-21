#include "orientation/orientation_task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "freertos/portmacro.h"

static const char *TAG = "OrientationTask";

OrientationTask::OrientationTask(Mpu6050 &imu, OrientationFilter &filter, TickType_t period_ticks, size_t queue_len)
    : imu_(imu), filter_(filter), period_(period_ticks), queue_(xQueueCreate(queue_len, sizeof(OrientationSample))), task_(nullptr) {}

esp_err_t OrientationTask::start(const char *task_name, UBaseType_t priority, uint32_t stack_words, BaseType_t core_id)
{
    if (task_ != nullptr || queue_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t created = xTaskCreatePinnedToCore(taskEntry, task_name, stack_words, this, priority, &task_, core_id);
    if (created != pdPASS)
    {
        return ESP_FAIL;
    }
    esp_err_t intr_err = attachInterrupt();
    if (intr_err != ESP_OK)
    {
        vTaskDelete(task_);
        task_ = nullptr;
        return intr_err;
    }
    return ESP_OK;
}

void OrientationTask::taskEntry(void *arg)
{
    auto *self = static_cast<OrientationTask *>(arg);
    self->run();
    vTaskDelete(nullptr);
}

void IRAM_ATTR OrientationTask::gpioIsrThunk(void *arg)
{
    auto *self = static_cast<OrientationTask *>(arg);
    if (!self || !self->task_)
    {
        return;
    }
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(self->task_, &hp);
    if (hp == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

esp_err_t OrientationTask::attachInterrupt()
{
    gpio_num_t pin = imu_.intPin();

    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE)
    {
        return isr_err;
    }

    esp_err_t err = gpio_set_intr_type(pin, GPIO_INTR_POSEDGE);
    if (err != ESP_OK)
    {
        return err;
    }
    err = gpio_isr_handler_add(pin, gpioIsrThunk, this);
    return err;
}

void OrientationTask::run()
{
    int64_t last = 0;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        OrientationSample sample{};
        sample.timestamp_us = esp_timer_get_time();

        AccelReading accel{};
        GyroReading gyro{};
        if (imu_.readAccelGyro(accel, gyro) == ESP_OK)
        {
            int64_t now = sample.timestamp_us;
            float dt = (last == 0) ? (period_ / 1000.0f) : (static_cast<float>(now - last) / 1e6f);
            last = now;

            filter_.update(accel, gyro, dt);
            sample.orientation = filter_.getOrientation();

            if (xQueueSend(queue_, &sample, 0) != pdTRUE)
            {
                (void)xQueueReceive(queue_, &sample, 0);
                (void)xQueueSend(queue_, &sample, 0);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read accel/gyro");
        }
    }
}
