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
        if (temperature_sensor_install(&cfg, &s_tsens) != ESP_OK ||
            temperature_sensor_enable(s_tsens) != ESP_OK) {
            ESP_LOGW(TAG, "die temperature sensor unavailable");
            s_tsens = NULL;
            s_failed = true;
            return NAN;
        }
    }
    float c;
    if (temperature_sensor_get_celsius(s_tsens, &c) != ESP_OK) {
        return NAN;
    }
    return c;
}
