#include "die_temp.h"

#include <math.h>
#include <stdbool.h>

#include "driver/temperature_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "die_temp";
static temperature_sensor_handle_t s_tsens = NULL;
static bool s_failed = false;

// app_main and the HTTP task can both make the first call. Two installs race,
// and the chip has one sensor, so the loser's install fails and would mark it
// unavailable for good. The mutex exists before app_main runs.
static StaticSemaphore_t s_lock_buf;
static SemaphoreHandle_t s_lock;

__attribute__((constructor)) static void die_temp_lock_init(void)
{
    s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
}

static float read_locked(void)
{
    if (s_failed) {
        return NAN;
    }
    if (s_tsens == NULL) {
        temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
        esp_err_t err = temperature_sensor_install(&cfg, &s_tsens);
        if (err == ESP_OK) {
            err = temperature_sensor_enable(s_tsens);
            if (err != ESP_OK) {
                temperature_sensor_uninstall(s_tsens);
            }
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "die temperature sensor unavailable: %s", esp_err_to_name(err));
            s_tsens = NULL;
            s_failed = true;
            return NAN;
        }
    }
    float c;
    esp_err_t err = temperature_sensor_get_celsius(s_tsens, &c);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "die temperature read failed: %s", esp_err_to_name(err));
        return NAN;
    }
    return c;
}

float die_temp_read(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    float c = read_locked();
    xSemaphoreGive(s_lock);
    return c;
}
