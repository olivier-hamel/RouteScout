#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "orientation/mpu6050.h"
#include "orientation/orientation_filter.h"

struct OrientationSample
{
    Orientation orientation;
    int64_t timestamp_us;
};

/**
 * @brief Tache qui attends l'interruption "data ready du mpu6050"
 */
class OrientationTask
{
public:
    /**
     * @brief Constructeur.
     *
     * @param imu          Référence vers le driver MPU6050
     * @param filter       Filtre d'orientation
     * @param period_ticks Période "attendue" utilisée uniquement pour la première itération. (Par défaut 200ms).
     * @param queue_len    Taille de la queue de sortie
     */
    OrientationTask(Mpu6050 &imu, OrientationFilter &filter, TickType_t period_ticks = pdMS_TO_TICKS(200), size_t queue_len = 8);

    /**
     * @brief Démarre la tâche FreeRTOS et attache l'interruption GPIO du MPU6050.
     *
     * @param task_name   Nom de la tâche
     * @param priority    Priorité
     * @param stack_words Taille de pile en "mots" .
     * @param core_id     Affinité CPU (tskNO_AFFINITY ou 0/1 sur ESP32)
     */
    esp_err_t start(const char *task_name = "imu_task", UBaseType_t priority = tskIDLE_PRIORITY + 1, uint32_t stack_words = 4096, BaseType_t core_id = tskNO_AFFINITY);

    /**
     * @brief Queue de sortie
     */
    QueueHandle_t queue() const { return queue_; }

private:
    static void taskEntry(void *arg);
    static void IRAM_ATTR gpioIsrThunk(void *arg);
    void run();
    esp_err_t attachInterrupt();

    Mpu6050 &imu_;
    OrientationFilter &filter_;
    TickType_t period_;
    QueueHandle_t queue_;
    TaskHandle_t task_;
};
