#include "board_hal.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "epaper.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "board_hal_gd133c02";

// Good Display ESP32-133C02. Everything the board can do for this firmware is
// drive the panel and sleep: no charger IC is documented, the battery input is
// not sensed, and the CH340 USB-UART gives no VBUS signal, so power source and
// battery state are reported as unknown rather than guessed.

esp_err_t board_hal_init(void)
{
    ESP_LOGI(TAG, "Initializing Good Display ESP32-133C02 Board HAL");

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BOARD_HAL_SPI_MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = BOARD_HAL_SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1200 * 1600 / 2 + 100,  // one packed 4bpp frame
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    epaper_config_t ep_cfg = {
        .spi_host = SPI2_HOST,
        .pin_cs = BOARD_HAL_EPD_CS_PIN,
        .pin_dc = BOARD_HAL_EPD_DC_PIN,
        .pin_rst = BOARD_HAL_EPD_RST_PIN,
        .pin_busy = BOARD_HAL_EPD_BUSY_PIN,
        .pin_cs1 = BOARD_HAL_EPD_CS1_PIN,
        .pin_enable = BOARD_HAL_EPD_ENABLE_PIN,
    };
    epaper_init(&ep_cfg);

    return ESP_OK;
}

esp_err_t board_hal_prepare_for_sleep(void)
{
    ESP_LOGI(TAG, "Preparing ESP32-133C02 for sleep");
    // Puts the panel to sleep, then cuts its rail via the enable pin and latches
    // every panel-facing pad LOW through deep sleep. The vendor's own notes say a
    // panel left powered without refreshing is damaged, so the rail cut matters.
    epaper_enter_deepsleep();
    return ESP_OK;
}

bool board_hal_is_battery_connected(void)
{
    return false;
}

int board_hal_get_battery_voltage(void)
{
    return -1;
}

int board_hal_get_battery_percent(void)
{
    return -1;
}

bool board_hal_is_charging(void)
{
    return false;
}

bool board_hal_is_usb_connected(void)
{
    // Not detectable: the CH340 bridge does not route VBUS to a GPIO. Reporting
    // "not connected" keeps the deep-sleep schedule in charge; the web UI's
    // deep-sleep switch is the way to keep the board awake on a desk.
    return false;
}

void board_hal_shutdown(void)
{
    ESP_LOGI(TAG, "No power switch under firmware control, entering deep sleep instead");
    board_hal_prepare_for_sleep();
    esp_deep_sleep_start();
}

esp_err_t board_hal_rtc_init(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t board_hal_rtc_get_time(time_t *t)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t board_hal_rtc_set_time(time_t t)
{
    return ESP_ERR_NOT_SUPPORTED;
}

bool board_hal_rtc_is_available(void)
{
    return false;
}

esp_err_t board_hal_get_temperature(float *t)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t board_hal_get_humidity(float *h)
{
    return ESP_ERR_NOT_SUPPORTED;
}

void board_hal_led_set(board_hal_led_t led, bool on)
{
    // LED2/LED3 exist on the board but their GPIOs are undocumented.
    (void) led;
    (void) on;
}
