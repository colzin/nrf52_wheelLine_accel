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

#include "sdk_config.h"

#define DEVICE_NAME      "WLD"  /**< Name of device. Will be included in the advertising data. */

#define VERSION_MAJOR_BYTE  0 // 1 byte
#define VERSION_MINOR_BYTE  0 // 1 byte
#define VERSION_PATCH_BYTE  1 // 1 byte

// choose radio
#define COMPILE_RADIO_CC1101 0
#define COMPILE_EV1527 0
#define COMPILE_RADIO_900T20D 1

#if COMPILE_RADIO_CC1101
#warning "Compiling for CC1101 radio"
#define RADIO_NAME "CC1101"
#if COMPILE_EV1527
#warning "EV1527 for CC1101"
#endif // #if COMPILE_EV1527
#elif COMPILE_RADIO_900T20D
#warning "Compiling for 900T20D radio"
#define RADIO_NAME "900T20D"
#else
#error "define radio"
#endif // #if COMPILE_RADIO_CC1101

// choose display
#define COMPILE_SH1107 0
#define COMPILE_4DIGIT7SEG 0
#define COMPILE_EINK 0

// choose board

#if defined(NRF52832_XXAA)
#define COMPILE_FOR_PCA10040 1 // Turn this on to use PCA10040, dev board for nRF52832
#define COMPILE_FOR_FEATHER 0 // Turn this on for feather
#define COMPILE_FOR_PCA10056 0 // Dev board for nRF52840
#elif defined(NRF52840_XXAA)
#define COMPILE_FOR_PCA10040 0
#define  COMPILE_FOR_FEATHER 0
#define COMPILE_FOR_PCA10056 1 // Dev board for nRF52840
#endif // #if defined(NRF52832_XXAA)

#if (COMPILE_RADIO_CC1101||COMPILE_EINK)
#define COMPILE_SPI 1
#else
#define COMPILE_SPI 0
#endif // #if (COMPILE_RADIO_CC1101||COMPILE_EINK)

#if (COMPILE_LIS2DH12||COMPILE_4DIGIT7SEG)
#define COMPILE_I2C 1
#else
#define COMPILE_I2C 0
#endif // #if (COMPILE_LIS2DH12||COMPILE_4DIGIT7SEG)

#include "pindefs.h"
////////////////////////// pins NAMES THAT WE USE, mapping to each MCU in pindefs.h:

#if COMPILE_FOR_PCA10040
#warning "Compiling for PCA10040"
#define BOARD_NAME "PCA10040"

#define HEARTBEAT_LED_GPIO_NUM PCA10040_GPIO17_LED_1

#if COMPILE_EV1527
// Use the pins for either EV1527 format
#define SPI2_SCK_PIN    PCA10040_GPIO7_UART_CTS // Use an un-useable pin, we don't care about this signal.
#define SPI2_MOSI_PIN   PCA10040_GPIO4
#endif // #if COMPILE_EV1527

#if COMPILE_RADIO_CC1101
/* CC1101 pinout:
 * GD0 goes to ?
 * GD1 not pinned out on Solu 8-pin module
 * GD2 goes to ?
 * CS goes to ?
 * MOSI, MISO, SCK pins. So use on "SPI0"
 */
#define CC1101_GDO2_PIN   PCA10040_GPIO5_UART_RTS
#elif COMPILE_RADIO_900T20D
#define _900T20D_M0_PIN PCA10040_GPIO5_UART_RTS
#define _900T20D_M1_PIN PCA10040_GPIO6_UART_TXD
#define _900T20D_RXD_PIN PCA10040_GPIO11
#define _900T20D_TXD_PIN PCA10040_GPIO12
#define _900T20D_AUX_PIN   PCA10040_GPIO7_UART_CTS
#if COMPILE_SPI
#error "can't use these pins"
#endif // #if COMPILE_SPI
#endif // #if COMPILE_RADIO_CC1101

#if COMPILE_SPI
// SPI0 for e-ink, CC1101, etc...
#define SPI_MISO_PIN   PCA10040_GPIO12
#define SPI_MOSI_PIN   PCA10040_GPIO6_UART_TXD
#define SPI_SCK_PIN    PCA10040_GPIO11

#if COMPILE_RADIO_CC1101
#define SPI_CC1101_CS_GPIO PCA10040_GPIO3
#endif // #if COMPILE_RADIO_CC1101
#if COMPILE_EINK
#define SPI_EINK_CS_GPIO PCA10040_GPIO?
#endif // #if COMPILE_EINK

#endif // #if COMPILE_SPI

#if COMPILE_I2C
#define I2C1_SCL_PIN    PCA10040_GPIO28
#define I2C1_SDA_PIN    PCA10040_GPIO29
#endif // #if COMPILE_I2C

// GPIOs for Driver side
#define KILLOPEN_RELAY_PIN  PCA10040_GPIO30
#define START_RELAY_PIN PCA10040_GPIO18_LED_2
#define FWD_RELAY_PIN   PCA10040_GPIO19_LED_3
#define REV_RELAY_PIN   PCA10040_GPIO20_LED_4

#else
#error "define a board please"
#endif // #if COMPILE_FOR_PCA10040


#ifdef __cplusplus
}
#endif // #ifdef __cplusplus

#endif /* SRC_VERSION_H_ */
