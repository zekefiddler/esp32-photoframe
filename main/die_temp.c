#include "die_temp.h"

#include <math.h>

#include "driver/temperature_sensor.h"
#include "esp_log.h"

static const char *TAG = "die_temp";
static temperature_sensor_handle_t s_tsens = NULL;
static bool s_failed = false;

float die_temp_read(void)
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
