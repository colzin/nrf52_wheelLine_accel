/*
 * version.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#ifdef __cplusplus
extern "C" {
#endif

#define DEVICE_NAME      "WLR"  /**< Name of device. Will be included in the advertising data. */

#define VERSION_MAJOR    0 // 1 byte
#define VERSION_MINOR    0 // 1 byte
#define VERSION_SUBMINOR 1 // 1 byte
#define COMPILE_FOR_PCA10040 1
#if (!COMPILE_FOR_PCA10040)
#define COMPILE_FOR_FEATHER 1
#endif // #if (!COMPILE_FOR_PCA10040)

#if (COMPILE_FOR_FEATHER&& COMPILE_FOR_PCA10040)
#error "Define one board to run on"
#endif // #if (COMPILE_FOR_FEATHER&& COMPILE_FOR_PCA10040)

#define COMPILE_RADIO_CC1101 0
#if COMPILE_RADIO_CC1101
#warning "Compiling for CC1101 radio"
#else
#define COMPILE_RADIO_900T20D 1
#if COMPILE_RADIO_900T20D
#warning "Compiling for 900T20D radio"
#else
#error "define radio"
#endif // #if COMPILE_RADIO_900T20D
#endif // #if COMPILE_RADIO_CC1101

#include "pindefs.h"
////////////////////////// pins NAMES THAT WE USE, mapping to each MCU in pindefs.h:

#if COMPILE_FOR_PCA10040
#warning "Compiling for PCA10040"

#define HEARTBEAT_LED_GPIO_NUM PCA10040_GPIO17_LED_1

// Use the pins for either EV1527 format
#define SPI2_SCK_PIN    PCA10040_GPIO7_UART_CTS // Use an un-useable pin, we don't care about this signal.
//#define SPI2_MOSI_PIN   PCA10040_GPIO4

#if COMPILE_RADIO_CC1101
#define CC1101_GDO2_PIN   PCA10040_GPIO5_UART_RTS

// SPI0 for e-ink, CC1101, etc...
#define SPI0_MISO_PIN   PCA10040_GPIO12
#define SPI0_MOSI_PIN   PCA10040_GPIO6_UART_TXD
#define SPI0_SCK_PIN    PCA10040_GPIO11

/* CC1101 pinout:
 * GD0 goes to ?
 * GD1 not pinned out on Solu 8-pin module
 * GD2 goes to ?
 * CS goes to ?
 * MOSI, MISO, SCK pins. So use on "SPI0"
 */
#define SPI0_CC1101_CS_GPIO PCA10040_GPIO3
#endif // #if COMPILE_RADIO_CC1101

#if COMPILE_RADIO_900T20D
#define _900T20D_AUX_PIN  PCA10040_GPIO7_UART_CTS

#define _900T20D_M0_PIN   PCA10040_GPIO5_UART_RTS
#define _900T20D_M1_PIN   PCA10040_GPIO6_UART_TXD
#define _900T20D_VCC_CTRL_PIN PCA10040_GPIO3
#define _900T20D_UART_TO_MODULE    PCA10040_GPIO11
#define _900T20D_UART_FROM_MODULE   PCA10040_GPIO12
#endif // #if COMPILE_RADIO_CC1101

/* I2C1:
 * Has LIS2DH12 on it, LED screen 7-segment
 */
#define I2C1_SCL_PIN    PCA10040_GPIO28
#define I2C1_SDA_PIN    PCA10040_GPIO29

// GPIOs for buttons
#define BUTTON_START_PIN    PCA10040_GPIO13_BUTTON_1
#define BUTTON_REV_PIN    PCA10040_GPIO14_BUTTON_2
#define BUTTON_FWD_PIN    PCA10040_GPIO15_BUTTON_3

#elif COMPILE_FOR_FEATHER
#warning "Compiling for nRF52 Bluefruit Feather"
#define HEARTBEAT_LED_GPIO_NUM FEATHER_ONBOARD_GPIO17_LED1

// For EV1527SPI
#define SPI2_SCK_PIN    FEATHER_GPIO16_16
#define SPI2_MOSI_PIN   FEATHER_GPIO15_15

#define CC1101_GDO2_PIN   FEATHER_GPIO28_A4

// SPI0 for e-ink, CC1101, etc...
#define SPI0_MISO_PIN   FEATHER_GPIO14_MISO
#define SPI0_MOSI_PIN   FEATHER_GPIO13_MOSI
#define SPI0_SCK_PIN    FEATHER_GPIO12_SCK

/* E-ink pinout:
 * SD card CS to Pin D5 = 27
 * SRAM CS to Pin D6 = 30
 * EINK CS to Pin D9 = 31
 * EINK DC to Pin D10 = 11
 * E-ink also goes to MOSI, MISO, SCK pins. So use on "SPI0"
 */
#define SPI0_SDCARD_CS_GPIO     FEATHER_GPIO27_27 // connects to featherWing board
#define SPI0_EINK_SRAM_CS_GPIO  FEATHER_GPIO30_30 // connects to featherWing board
#define SPI0_EINK_CS_GPIO       FEATHER_GPIO31_A7 // connects to featherWing board
#define EINK_DC_GPIO            FEATHER_GPIO11_11 // connects to featherWing board

/* CC1101 pinout:
 * GD0 goes to ?
 * GD1 not pinned out on Solu 8-pin module
 * GD2 goes to ?
 * CS goes to ?
 * MOSI, MISO, SCK pins. So use on "SPI0"
 */
#if COMPILE_RADIO_CC1101
#define SPI0_CC1101_CS_GPIO FEATHER_GPIO29_A5
#elif COMPILE_RADIO_900T20D
#define _900T20D_AUX_PIN FEATHER_GPIO29_A5
#endif // #if COMPILE_RADIO_CC1101

/* I2C1:
 * Has LIS2DH12 on it, LED screen 7-segment
 */
#define I2C1_SCL_PIN    FEATHER_GPIO26_SCL
#define I2C1_SDA_PIN    FEATHER_GPIO25_SDA

// GPIOs for buttons
#define BUTTON_START_PIN    FEATHER_OLED_BUTTONB_PIN
#define BUTTON_REV_PIN    FEATHER_OLED_BUTTONA_PIN
#define BUTTON_FWD_PIN    FEATHER_OLED_BUTTONC_PIN

#define UART_RX_PIN FEATHER_GPIO8_USB_UART_RX
#define UART_TX_PIN FEATHER_GPIO6_USB_UART_TX

#else
#error "define a board please"
#endif // #if COMPILE_FOR_PCA10040

#ifdef __cplusplus
}
#endif

#endif /* SRC_VERSION_H_ */
