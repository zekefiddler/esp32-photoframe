#ifndef BOARD_GOODDISPLAY_ESP32_133C02_H
#define BOARD_GOODDISPLAY_ESP32_133C02_H

#include "driver/gpio.h"

// Good Display ESP32-133C02: ESP32-S3-WROOM-1 (N16R8) driver board sold with the
// 7.09" GDEB0709E01 and the 13.3" GDEP133C02 Spectra 6 panels. Pin map from the
// vendor's ESP32-133C02 specification rev 2.0 (2026-03) §3 and their GDEB0709E01
// ESP32 code example. USB is a CH340 on UART0 (no native USB, so USB power is
// not detectable), there is no battery sense, no RTC, no LED under firmware control.

// Board Info
#define BOARD_HAL_NAME "gooddisplay_esp32_133c02"
#define BOARD_HAL_TYPE BOARD_TYPE_GOODDISPLAY_ESP32_133C02

// Buttons: SW2/SW3/SW4 are on IO12/IO13/IO14 and ACTIVE-HIGH with external
// pull-downs (spec §3.1). The button task and EXT1 wake path assume active-low
// with internal pull-ups, so the buttons stay unwired until that polarity is
// supported. Wake is timer-only; a power cycle brings up the web UI.
#define BOARD_HAL_WAKEUP_KEY GPIO_NUM_NC
#define BOARD_HAL_WAKEUP_KEY_NAME "None"
#define BOARD_HAL_ROTATE_KEY GPIO_NUM_NC
#define BOARD_HAL_CLEAR_KEY GPIO_NUM_NC

// SPI Pins (panel bus). The microSD slot is on its own bus — CS 15, MOSI 3,
// MISO 5, SCK 8 — and is not enabled by this board definition.
#define BOARD_HAL_SPI_SCLK_PIN GPIO_NUM_9
#define BOARD_HAL_SPI_MOSI_PIN GPIO_NUM_41
#define BOARD_HAL_SPI_MISO_PIN GPIO_NUM_NC

// E-Paper Display Pins (dual-controller panel, 4-wire SPI with a DC line)
#define BOARD_HAL_EPD_DC_PIN GPIO_NUM_2
#define BOARD_HAL_EPD_CS_PIN GPIO_NUM_18   // EPD_CS_M: master, left 600 columns
#define BOARD_HAL_EPD_CS1_PIN GPIO_NUM_17  // EPD_CS_S: slave, right 600 columns
#define BOARD_HAL_EPD_RST_PIN GPIO_NUM_6
#define BOARD_HAL_EPD_BUSY_PIN GPIO_NUM_7
// Panel load switch (vendor PWR / LOAD_SW). GPIO45 is a strapping pin; LOW is
// its safe boot level and is also what the driver holds it at through sleep.
#define BOARD_HAL_EPD_ENABLE_PIN GPIO_NUM_45

// Display Configuration
#define BOARD_HAL_DISPLAY_ROTATION_DEG 0

#endif  // BOARD_GOODDISPLAY_ESP32_133C02_H
