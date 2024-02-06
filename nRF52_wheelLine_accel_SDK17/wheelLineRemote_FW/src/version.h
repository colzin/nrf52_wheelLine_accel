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

////////////////////////////   PINS on nRF52 DK PCA10040
#define XTAL_PIN_1 0 // p0.00 used for 32.768kHz XTAL on PCA10040
#define XTAL_PIN_2 1 // p0.01 used for 32.768kHz XTAL on PCA10040

#define PCA10040_GPIO2     2 // p0.02 routed to a pin on PCA10040, otherwise unused
#define PCA10040_GPIO3     3 // p0.03 routed to a pin on PCA10040, otherwise unused
#define PCA10040_GPIO4     4 // p0.05 routed to a pin on PCA10040, otherwise unused

#define PCA10040_GPIO5_UART_RTS   5
#define PCA10040_GPIO6_UART_TXD   6
#define PCA10040_GPIO7_UART_CTS   7
#define PCA10040_GPIO8_UART_RXD   8

#define PCA10040_GPIO9_NFC1  9 // Let the two NFC pins idle at the same state to save power
#define PCA10040_GPIO10_NFC2 10 // Let the two NFC pins idle at the same state to save power

#define PCA10040_GPIO11 11
#define PCA10040_GPIO12 12

// Buttons short to GND, pull them up to use
#define BUTTON_1       13
#define BUTTON_2       14
#define BUTTON_3       15
#define BUTTON_4       16

#define PCA10040_IOEXP_INT_PIN 17
// PCA10040 LEDs on 17, 18, 19, 29, active low, drive low to light
#define LED_1          17
#define LED_2          18
#define LED_3          19
#define LED_4          20

#define NRF52_RESET_PIN 21 // 21 is nRESET

#define PCA10040_GPIO22 22
#define PCA10040_GPIO23 23
#define PCA10040_GPIO24 24
#define PCA10040_GPIO25 25

#define PCA10040_IOEXP_SDA_PIN 26
#define PCA10040_IOEXP_SCL_PIN 27

#define PCA10040_GPIO28 28
#define PCA10040_GPIO29 29
#define PCA10040_GPIO30 30
#define PCA10040_GPIO31 31

////////////////////////// pins as we use them:

#define HEARTBEAT_LED_GPIO_NUM LED_1

#define ISR_DEBUG_GPIO LED_3

// Use the pins for either EV1527 format
#define I2S_LRCLK_PIN   UNUSED_PIN0 // Need a dummy pin
#define I2S_SCK_PIN     UNUSED_PIN1 // Need a dummy pin
#define I2S_SDOUT_PIN   PCA10040_GPIO4

#define SPI2_SCK_PIN    PCA10040_GPIO7_UART_CTS // Use an un-useable pin, we don't care about this signal.
#define SPI2_MOSI_PIN   PCA10040_GPIO4

// SPI0 for e-ink, CC1101, etc...
#define SPI0_MISO_PIN   PCA10040_GPIO12
#define SPI0_MOSI_PIN   PCA10040_GPIO6_UART_TXD
#define SPI0_SCK_PIN    PCA10040_GPIO11
#define SPI0_EINK_CS_GPIO   PCA10040_GPIO9_NFC1 // 9=NFC1
#define SPI0_CC1101_CS_GPIO PCA10040_GPIO10_NFC2 // 10=NFC2
#ifdef __cplusplus
}
#endif

#endif /* SRC_VERSION_H_ */
