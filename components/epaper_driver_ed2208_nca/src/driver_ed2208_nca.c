#include <assert.h>

#if CONFIG_EP_ED2208_NCA_PANEL_GDEB0709E01 && !CONFIG_BOARD_DRIVER_GOODDISPLAY_ESP32_133C02
#error "GDEB0709E01 panel tuning enabled for a board without that panel (stale sdkconfig?)"
#endif
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "epaper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

static const char *TAG = "epaper_ed2208_nca";

static epaper_config_t g_cfg;
static spi_device_handle_t spi;

#ifdef CONFIG_PM_ENABLE
static esp_pm_lock_handle_t pm_lock = NULL;
#endif

#define EPD_WIDTH 1200
#define EPD_HEIGHT 1600
// Packed pixel buffer size: 2 pixels per byte (4-bit color depth)
#define EPD_BUF_SIZE (EPD_WIDTH / 2 * EPD_HEIGHT)

// --- Initialization Data ---
// Two panels share this controller family and command flow but not their tuning.
// Seeed T133A01 values come from T133A01_Defines.h; Good Display GDEB0709E01
// values come from the vendor's GDEB0709E01 ESP32 code example (2026-08).
#if CONFIG_EP_ED2208_NCA_PANEL_GDEB0709E01
static const uint8_t PSR_V[] = {0xDF, 0x6B};
static const uint8_t POF_V[] = {0x01};
static const uint8_t DRF_V[] = {0x00};
static const uint8_t BTST_P_V[] = {0xE0, 0x20};
static const uint8_t BTST_N_V[] = {0xE0, 0x20};
static const uint8_t r74DataBuf[] = {0x00, 0x0C, 0x0C, 0xD9, 0xDD, 0xDD, 0x15, 0x15, 0x55};
// Commands the GDEB0709E01 init sends that the T133A01 init does not.
static const uint8_t DCDC_V[] = {0x44, 0x54, 0x00};
static const uint8_t PLL_V[] = {0x08};
static const uint8_t POFS_M_V[] = {0x00, 0xC0, 0x03, 0xA8};  // master (left) controller
static const uint8_t POFS_S_V[] = {0x00, 0xC0, 0x03, 0x9A};  // slave (right) controller
static const uint8_t CMDA4_V[] = {0x03, 0x00, 0x01, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00};
#else
static const uint8_t PSR_V[] = {0xDF, 0x69};
static const uint8_t POF_V[] = {0x00};
static const uint8_t DRF_V[] = {0x01};
static const uint8_t BTST_P_V[] = {0xD8, 0x18};
static const uint8_t BTST_N_V[] = {0xD8, 0x18};
static const uint8_t r74DataBuf[] = {0xC0, 0x1C, 0x1C, 0xCC, 0xCC, 0xCC, 0x15, 0x15, 0x55};
#endif
static const uint8_t PWR_V[] = {0x0F, 0x00, 0x28, 0x2C, 0x28, 0x38};
static const uint8_t CDI_V[] = {0x37};
static const uint8_t TRES_V[] = {0x04, 0xB0, 0x03, 0x20};
static const uint8_t CCSET_V_CUR[] = {0x01};
static const uint8_t PWS_V[] = {0x22};

static const uint8_t rf0DataBuf[] = {0x49, 0x55, 0x13, 0x5D, 0x05, 0x10};
static const uint8_t r60DataBuf[] = {0x03, 0x03};
static const uint8_t r86DataBuf[] = {0x10};
static const uint8_t rb6DataBuf[] = {0x07};
static const uint8_t rb7DataBuf[] = {0x01};
static const uint8_t rb0DataBuf[] = {0x01};
static const uint8_t rb1DataBuf[] = {0x02};

// --- Low-level SPI helpers ---

// Send a command with optional data bytes in a single chip-select window on `cs`.
// The pin stays LOW for the entire command+data sequence.
static void cmd_data_on(gpio_num_t cs, uint8_t cmd, const uint8_t *data, size_t len)
{
    gpio_set_level(cs, 0);  // CS low

    // Send Command
    gpio_set_level(g_cfg.pin_dc, 0);  // DC low = command
    spi_transaction_t t_cmd = {.length = 8, .tx_buffer = &cmd};
    spi_device_transmit(spi, &t_cmd);

    // Send Data
    if (len > 0) {
        gpio_set_level(g_cfg.pin_dc, 1);  // DC high = data
        spi_transaction_t t_data = {.length = len * 8, .tx_buffer = data};
        spi_device_transmit(spi, &t_data);
    }

    gpio_set_level(cs, 1);  // CS high
}

// Left (master) controller only: CS LOW, CS1 stays HIGH.
static void cmd_data(uint8_t cmd, const uint8_t *data, size_t len)
{
    cmd_data_on(g_cfg.pin_cs, cmd, data, len);
}

// Send a command targeting both controllers (CS1 LOW during cmd_data)
static void cmd_data_both(uint8_t cmd, const uint8_t *data, size_t len)
{
    gpio_set_level(g_cfg.pin_cs1, 0);
    cmd_data(cmd, data, len);
    gpio_set_level(g_cfg.pin_cs1, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

#if CONFIG_EP_ED2208_NCA_PANEL_GDEB0709E01
// Right (slave) controller only: CS1 LOW, CS stays HIGH.
static void cmd_data_cs1(uint8_t cmd, const uint8_t *data, size_t len)
{
    cmd_data_on(g_cfg.pin_cs1, cmd, data, len);
}
#endif

// Whether the controller is up and initialised. It is only ever up inside
// epaper_display: every draw resets and re-initialises it, and ends by putting
// it in its own deep-sleep state (panel_sleep). The rail is cut for the
// board's deep sleep and when the controller stops answering (panel_rail_off).
static bool g_powered = false;

// BUSY limits. The short one is for phases a healthy controller finishes in
// well under a second, so a stuck line there means a dead or hung controller.
// The refresh is different: a full refresh of the GDEB0709E01 takes ~45 s on
// the bench at room temperature, and longer as the glass cools. The old single
// 40 s limit was within seconds of sending the power-off command to a healthy
// refresh still in progress.
#define BUSY_SHORT_MS 40000
#define BUSY_REFRESH_MS 120000

static bool is_busy(void)
{
    return gpio_get_level(g_cfg.pin_busy) == 0;
}

// Wait for BUSY to release, up to `max_ms`. Returns false on timeout; the
// caller decides what a silent controller means. In epaper_display it means
// the rail gets cut - never "carry on and hope".
static bool wait_busy(const char *label, uint32_t max_ms)
{
    vTaskDelay(pdMS_TO_TICKS(10));
    uint32_t waited_ms = 10;
    while (is_busy()) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
        if (waited_ms > max_ms) {
            ESP_LOGE(TAG, "[%s] BUSY timeout after %lu ms", label, (unsigned long) max_ms);
            return false;
        }
    }
    return true;
}

// --- Hardware setup ---

// Claim the panel's pins and hold its rail OFF for 250 ms. Called once at
// boot; the rail comes up for the first draw (panel_power_up). The dark period
// is the point: a controller left mid-waveform by a reset, a crash or a
// brownout keeps its charge pumps running and answers nothing, and no command
// clears that - only a rail that actually drops. On the XIAO EE02 the enable
// pin is GPIO43 (U0TXD), which the boot ROM drives high through reset, so a
// reset alone never gives that panel a power cycle; this does.
static void gpio_init(void)
{
    // Release any pad holds latched by a previous deep-sleep cycle
    // (see epaper_enter_deepsleep) so gpio_config + gpio_set_level
    // below can re-drive these pins.
    gpio_hold_dis(g_cfg.pin_cs);
    gpio_hold_dis(g_cfg.pin_cs1);
    gpio_hold_dis(g_cfg.pin_dc);
    gpio_hold_dis(g_cfg.pin_rst);

    // Panel-facing lines LOW while the rail is down, so nothing back-feeds
    // through the panel's ESD diodes (the same rule epaper_enter_deepsleep
    // follows). Levels first, then drivers, so nothing glitches high.
    gpio_set_level(g_cfg.pin_cs, 0);
    gpio_set_level(g_cfg.pin_cs1, 0);
    gpio_set_level(g_cfg.pin_dc, 0);
    gpio_set_level(g_cfg.pin_rst, 0);

    gpio_config_t out_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << g_cfg.pin_rst) | (1ULL << g_cfg.pin_dc) | (1ULL << g_cfg.pin_cs) |
                        (1ULL << g_cfg.pin_cs1),
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&out_conf));

    gpio_config_t in_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << g_cfg.pin_busy),
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&in_conf));

    if (g_cfg.pin_enable >= 0) {
        gpio_hold_dis(g_cfg.pin_enable);
        gpio_set_level(g_cfg.pin_enable, 0);
        gpio_config_t en_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << g_cfg.pin_enable),
        };
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&en_conf));
        gpio_set_level(g_cfg.pin_enable, 0);
        vTaskDelay(pdMS_TO_TICKS(250));  // long enough for the panel's rails to collapse
    }
    g_powered = false;
}

static void hw_reset(void);
static bool send_init_sequence(void);
static void panel_rail_off(bool latch);

// Rail up, reset, init sequence: everything a draw needs. Idempotent by way
// of g_powered, so a caller can ask for it before every draw. Returns false if
// the controller never released BUSY after reset; the rail is already cut then,
// so the caller must not send it anything.
static bool panel_power_up(void)
{
    if (g_powered) {
        return true;
    }
    if (g_cfg.pin_enable >= 0) {
        gpio_set_level(g_cfg.pin_enable, 1);
        vTaskDelay(pdMS_TO_TICKS(100));  // allow display power to stabilize
    }
    // Idle levels now the rail is up: deselected, command mode, out of reset.
    gpio_set_level(g_cfg.pin_cs, 1);
    gpio_set_level(g_cfg.pin_cs1, 1);
    gpio_set_level(g_cfg.pin_dc, 0);
    gpio_set_level(g_cfg.pin_rst, 1);
    hw_reset();
    if (!send_init_sequence()) {
        ESP_LOGE(TAG, "Controller stuck after reset; cutting the panel rail");
        panel_rail_off(false);
        return false;
    }
    g_powered = true;
    return true;
}

// Put the controller into its own deep-sleep state between draws. The rail
// stays up: SPI2 is shared with the SD card on some boards, and clocking a bus
// into an unpowered controller back-feeds it through its ESD diodes. The next
// draw resets and re-initialises it (panel_power_up).
static void panel_sleep(void)
{
    if (!g_powered) {
        return;
    }
    cmd_data(0x07, (uint8_t[]){0xA5}, 1);  // DEEP_SLEEP
    vTaskDelay(pdMS_TO_TICKS(1));
    if (!wait_busy("deepsleep", BUSY_SHORT_MS)) {
        // It did not take the sleep command, so its charge pumps may still be
        // running with the rail held up. Cut it, as for any silent controller.
        ESP_LOGE(TAG, "Controller did not sleep; cutting the panel rail");
        panel_rail_off(false);
        return;
    }
    g_powered = false;
}

// Cut the panel's rail. Used for deep sleep (`latch` holds every panel-facing
// pad low through it) and for a controller that stopped answering, where this
// is the only thing guaranteed to stop its charge pumps. Boards without a rail
// to cut (pin_enable < 0) only get panel_sleep.
static void panel_rail_off(bool latch)
{
    g_powered = false;
    if (g_cfg.pin_enable < 0) {
        return;
    }
    // Drive panel-facing GPIOs LOW before cutting VDD so they don't
    // back-feed through the panel's ESD diodes once its rail drops to
    // 0V. SPI peripheral pads (MOSI/SCLK) become Hi-Z in deep sleep
    // and don't need handling.
    gpio_set_level(g_cfg.pin_cs, 0);
    gpio_set_level(g_cfg.pin_dc, 0);
    gpio_set_level(g_cfg.pin_rst, 0);
    if (g_cfg.pin_cs1 >= 0) {
        gpio_set_level(g_cfg.pin_cs1, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(100));       // Ensure display enters sleep before cutting power
    gpio_set_level(g_cfg.pin_enable, 0);  // Cut power
    if (latch) {
        // Latch the pads low so the rail stays cut and panel-facing
        // signals stay grounded once the digital IO domain powers down
        // during deep sleep, and arm deep-sleep hold so the latch
        // survives into deep sleep. All calls are idempotent.
        gpio_hold_en(g_cfg.pin_enable);
        gpio_hold_en(g_cfg.pin_cs);
        gpio_hold_en(g_cfg.pin_dc);
        gpio_hold_en(g_cfg.pin_rst);
        if (g_cfg.pin_cs1 >= 0) {
            gpio_hold_en(g_cfg.pin_cs1);
        }
        gpio_deep_sleep_hold_en();
    }
}

static void spi_add_device(void)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,  // CS is manually controlled
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(g_cfg.spi_host, &devcfg, &spi));
}

static void hw_reset(void)
{
    gpio_set_level(g_cfg.pin_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(g_cfg.pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

// --- Display operations ---

// The 13.3" panel has two gate driver ICs: CS selects the left half,
// CS1 selects the right half. LOW = selected.
// Commands 0xF0 through TRES go to both controllers (CS1 LOW).
// Commands 0x74 and PWR through 0xB1 go to left controller only (CS1 HIGH).
static bool send_init_sequence(void)
{
    if (!wait_busy("init", BUSY_SHORT_MS)) {
        return false;  // a controller that never came out of reset hears nothing
    }

    // 0x74 — CS1 stays HIGH (left controller only)
    cmd_data(0x74, r74DataBuf, sizeof(r74DataBuf));

    // 0xF0 through TRES — both controllers
    cmd_data_both(0xF0, rf0DataBuf, sizeof(rf0DataBuf));
    cmd_data_both(0x00, PSR_V, sizeof(PSR_V));
#if CONFIG_EP_ED2208_NCA_PANEL_GDEB0709E01
    // Vendor order for the GDEB0709E01: DCDC (left only) and PLL right after PSR.
    cmd_data(0xA5, DCDC_V, sizeof(DCDC_V));
    cmd_data_both(0x30, PLL_V, sizeof(PLL_V));
#endif
    cmd_data_both(0x50, CDI_V, sizeof(CDI_V));
    cmd_data_both(0x60, r60DataBuf, sizeof(r60DataBuf));
#if CONFIG_EP_ED2208_NCA_PANEL_GDEB0709E01
    // POFS is per controller and the two halves take different values.
    cmd_data(0x03, POFS_M_V, sizeof(POFS_M_V));
    cmd_data_cs1(0x03, POFS_S_V, sizeof(POFS_S_V));
#endif
    cmd_data_both(0x86, r86DataBuf, sizeof(r86DataBuf));
    cmd_data_both(0xE3, PWS_V, sizeof(PWS_V));
#if CONFIG_EP_ED2208_NCA_PANEL_GDEB0709E01
    cmd_data_both(0xE0, CCSET_V_CUR, sizeof(CCSET_V_CUR));
#endif
    cmd_data_both(0x61, TRES_V, sizeof(TRES_V));

    // PWR through 0xB1 — left controller only (CS1 stays HIGH)
#if CONFIG_EP_ED2208_NCA_PANEL_GDEB0709E01
    cmd_data(0xA4, CMDA4_V, sizeof(CMDA4_V));
#endif
    cmd_data(0x01, PWR_V, sizeof(PWR_V));
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd_data(0xB6, rb6DataBuf, sizeof(rb6DataBuf));
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd_data(0x06, BTST_P_V, sizeof(BTST_P_V));
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd_data(0xB7, rb7DataBuf, sizeof(rb7DataBuf));
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd_data(0x05, BTST_N_V, sizeof(BTST_N_V));
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd_data(0xB0, rb0DataBuf, sizeof(rb0DataBuf));
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd_data(0xB1, rb1DataBuf, sizeof(rb1DataBuf));
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

// Convert 4-bit color index to hardware color value
static uint8_t color_get(uint8_t c)
{
    switch (c) {
    case 0:
        return 0x00;  // Black
    case 1:
        return 0x01;  // White
    case 2:
        return 0x02;  // Yellow
    case 3:
        return 0x03;  // Red
    case 5:
        return 0x05;  // Blue
    case 6:
        return 0x06;  // Green
    default:
        return 0x01;  // Default to White
    }
}

// --- Public API ---

uint16_t epaper_get_width(void)
{
    return EPD_WIDTH;
}

uint16_t epaper_get_height(void)
{
    return EPD_HEIGHT;
}

void epaper_init(const epaper_config_t *cfg)
{
    g_cfg = *cfg;

#if CONFIG_EP_ED2208_NCA_PANEL_GDEB0709E01
    ESP_LOGI(TAG, "Initializing GDEB0709E01 (Spectra 6, 7.09\") on the ED2208-NCA driver");
#else
    ESP_LOGI(TAG, "Initializing ED2208-NCA (Spectra 6, 13.3\") E-Paper Driver");
#endif

    spi_add_device();
    gpio_init();   // pins claimed, rail OFF: the panel comes on for the first draw

#ifdef CONFIG_PM_ENABLE
    esp_err_t ret = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "epd_update", &pm_lock);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PM lock: %s", esp_err_to_name(ret));
    }
#endif
}

void epaper_clear(uint8_t *image, uint8_t color)
{
    uint8_t c = color_get(color);
    uint8_t packed = (c << 4) | c;
    memset(image, packed, EPD_BUF_SIZE);

    ESP_LOGI(TAG, "Clearing display with color 0x%02x", color);
    epaper_display(image);
    ESP_LOGI(TAG, "Clear complete");
}

void epaper_display(uint8_t *image)
{
#ifdef CONFIG_PM_ENABLE
    if (pm_lock) {
        esp_pm_lock_acquire(pm_lock);
    }
#endif

    ESP_LOGI(TAG, "Starting display update: %d bytes", EPD_BUF_SIZE);

    if (!panel_power_up()) {
        goto out;  // rail already cut; nothing to talk to
    }

    // CCSET — both controllers
    cmd_data_both(0xE0, CCSET_V_CUR, sizeof(CCSET_V_CUR));

    if (!wait_busy("ccset", BUSY_SHORT_MS)) {
        goto fault;
    }

    // Split transfer: left half via CS, right half via CS1
    uint16_t block_w_bytes = EPD_WIDTH / 2 / 2;  // 300 bytes (packed)
    uint16_t row_stride_bytes = EPD_WIDTH / 2;   // 600 bytes

    uint8_t *line_buf = heap_caps_malloc(block_w_bytes, MALLOC_CAP_DMA);
    if (!line_buf) {
        ESP_LOGE(TAG, "Malloc failed");
        panel_sleep();
        goto out;
    }

    // Hold the SPI bus exclusively for the whole pixel transfer. CS/CS1 stay LOW
    // across every per-row transmit below, so any other transaction on this
    // shared bus (the SD card is on the same SPI2_HOST — e.g. a thumbnail being
    // written, or the debug-log writer task) would be clocked into the panel too,
    // desyncing the pixel stream: broken rows at the injection point and every
    // row after it shifted. Acquiring the bus makes those SD accesses wait their
    // turn instead of corrupting the frame. Released right after the transfer so
    // the (multi-second) refresh wait below doesn't block the SD.
    spi_device_acquire_bus(spi, portMAX_DELAY);

    // --- Phase 1: CS (Left half) ---
    gpio_set_level(g_cfg.pin_cs, 0);

    gpio_set_level(g_cfg.pin_dc, 0);  // CMD
    uint8_t dtm_cmd = 0x10;
    spi_transaction_t t_dtm = {.length = 8, .tx_buffer = &dtm_cmd};
    spi_device_transmit(spi, &t_dtm);

    gpio_set_level(g_cfg.pin_dc, 1);  // DATA

    for (uint16_t row = 0; row < EPD_HEIGHT; row++) {
        for (uint16_t col = 0; col < block_w_bytes; col++) {
            uint8_t b = image[row * row_stride_bytes + col];
            uint8_t p1 = (b >> 4) & 0x0F;
            uint8_t p2 = b & 0x0F;
            line_buf[col] = (color_get(p1) << 4) | color_get(p2);
        }
        spi_transaction_t t_line = {.length = block_w_bytes * 8, .tx_buffer = line_buf};
        spi_device_transmit(spi, &t_line);
    }
    gpio_set_level(g_cfg.pin_cs, 1);

    // --- Phase 2: CS1 (Right half) ---
    gpio_set_level(g_cfg.pin_cs1, 0);

    gpio_set_level(g_cfg.pin_dc, 0);  // CMD
    spi_device_transmit(spi, &t_dtm);

    gpio_set_level(g_cfg.pin_dc, 1);  // DATA

    for (uint16_t row = 0; row < EPD_HEIGHT; row++) {
        for (uint16_t col = 0; col < block_w_bytes; col++) {
            uint8_t b = image[row * row_stride_bytes + col + block_w_bytes];
            uint8_t p1 = (b >> 4) & 0x0F;
            uint8_t p2 = b & 0x0F;
            line_buf[col] = (color_get(p1) << 4) | color_get(p2);
        }
        spi_transaction_t t_line = {.length = block_w_bytes * 8, .tx_buffer = line_buf};
        spi_device_transmit(spi, &t_line);
    }
    gpio_set_level(g_cfg.pin_cs1, 1);

    spi_device_release_bus(spi);

    free(line_buf);

    // PON — both controllers
    cmd_data_both(0x04, NULL, 0);
    if (!wait_busy("power_on", BUSY_SHORT_MS)) {
        goto fault;
    }
#if CONFIG_EP_ED2208_NCA_PANEL_GDEB0709E01
    vTaskDelay(pdMS_TO_TICKS(50));  // vendor settles 50 ms between PON and DRF
#endif

    // DRF — both controllers
    cmd_data_both(0x12, DRF_V, sizeof(DRF_V));
    if (!wait_busy("refresh", BUSY_REFRESH_MS)) {
        goto fault;
    }

    // POF — both controllers
    cmd_data_both(0x02, POF_V, sizeof(POF_V));
    if (!wait_busy("power_off", BUSY_SHORT_MS)) {
        goto fault;
    }

    ESP_LOGI(TAG, "Display update complete");
    panel_sleep();
    goto out;

fault:
    // The controller stopped answering (wait_busy logged which phase). Its
    // charge pumps may be running with nobody driving a waveform; the one
    // thing guaranteed to end that is the rail. No sleep command first - it
    // would not be heard.
    ESP_LOGE(TAG, "Controller not answering; cutting the panel rail");
    panel_rail_off(false);

out:
#ifdef CONFIG_PM_ENABLE
    if (pm_lock) {
        esp_pm_lock_release(pm_lock);
    }
#endif
    return;
}

void epaper_enter_deepsleep(void)
{
    ESP_LOGI(TAG, "Entering deep sleep");

#ifdef CONFIG_PM_ENABLE
    if (pm_lock) {
        esp_pm_lock_acquire(pm_lock);
    }
#endif

    panel_sleep();        // no-op unless a draw was cut short and left it up
    panel_rail_off(true);

#ifdef CONFIG_PM_ENABLE
    if (pm_lock) {
        esp_pm_lock_release(pm_lock);
    }
#endif
}
