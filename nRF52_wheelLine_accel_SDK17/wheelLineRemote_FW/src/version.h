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

#if COMPILE_FOR_PCA10040
////////////////////////////   PINS on nRF52 DK PCA10040

#define HAS_XTAL 1 // XTAL on PCA10040
#define XTAL_PIN_1 0 // p0.00 used for 32.768kHz XTAL on PCA10040
#define XTAL_PIN_2 1 // p0.01 used for 32.768kHz XTAL on PCA10040

#define PCA10040_GPIO2     2 // p0.02 routed to a pin on PCA10040, otherwise unused
#define PCA10040_GPIO3     3 // p0.03 routed to a pin on PCA10040, otherwise unused
#define PCA10040_GPIO4     4 // p0.05 routed to a pin on PCA10040, otherwise unused

#define PCA10040_GPIO5_UART_RTS   5
#define PCA10040_GPIO6_UART_TXD   6
#define PCA10040_GPIO7_UART_CTS   7
#define PCA10040_GPIO8_UART_RXD   8

//#define PCA10040_GPIO9_NFC1  9 // NEED TO SOLDER to use these, do not use
//#define PCA10040_GPIO10_NFC2 10 // NEED TO SOLDER to use these, do not use

#define PCA10040_GPIO11 11
#define PCA10040_GPIO12 12

// Buttons short to GND, pull them up to use
#define PCA10040_GPIO13_BUTTON_1    13
#define PCA10040_GPIO14_BUTTON_2    14
#define PCA10040_GPIO15_BUTTON_3    15
#define PCA10040_GPIO16_BUTTON_4    16

#define PCA10040_GPIO17_IOEXP_INT 17

// PCA10040 LEDs on 17, 18, 19, 29, active low, drive low to light
#define PCA10040_GPIO17_LED_1   17
#define PCA10040_GPIO18_LED_2   18
#define PCA10040_GPIO19_LED_3   19
#define PCA10040_GPIO20_LED_4   20

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

#elif COMPILE_FOR_FEATHER

// P0.00, p0.01 not broken out, used for XTAL?
#define HAS_XTAL 1 // TODO double check

// Pins on left side (if microUSB is up)
#define FEATHER_GPIO21_nRST_Rst 21 // p0.21 used as nRST usually
//#define FEATHER_3v3_1
//#define FEATHER_3v3_2
//#define FEATHER_GND
#define FEATHER_GPIO2_A0    2
#define FEATHER_GPIO3_A1    3
#define FEATHER_GPIO4_A2    4
#define FEATHER_GPIO5_A3    5
#define FEATHER_GPIO28_A4   28 // p28 recommend low-speed IO for radio interference
#define FEATHER_GPIO29_A5   29 // p29 recommend low-speed IO for radio interference
#define FEATHER_GPIO12_SCK  12
#define FEATHER_GPIO13_MOSI 13
#define FEATHER_GPIO14_MISO 14 // Also TD[3]
#define FEATHER_GPIO8_RX    8
#define FEATHER_GPIO6_TX    6
#define FEATHER_GPIO20_DFU  20

// Pins on right side (if microUSB is up)
//#define FEATHER_VBAT_JST_PLUS
//#define FEATHER_3V3REG_EN // Drive low to disable 3.3V regulator
//#define FEATHER_USB_VBUS
#define FEATHER_GPIO16_16   16 // Also TD[1]
#define FEATHER_GPIO15_15   15 // Also TD[2]
#define FEATHER_GPIO7_7     7
#define FEATHER_GPIO11_11   11
#define FEATHER_GPIO31_31   31 // AVOID using this pin externally, used for LiPo measurement
#define FEATHER_GPIO30_30   30 // AVOID using this pin externally, used for LiPo measurement
#define FEATHER_GPIO27_27   27
#define FEATHER_GPIO26_SCL  26
#define FEATHER_GPIO25_SDA  25

#define FEATHER_ONBOARD_GPIO9_NFC1  9
#define FEATHER_ONBOARD_GPIO10_NFC2 10
#define FEATHER_ONBOARD_GPIO17_LED1 17
#define FEATHER_ONBOARD_GPIO19_LED2 19

#else
#error "define pins for your board please"
#endif // #if COMPILE_FOR_PCA10040
////////////////////////// pins as we use them:

#if COMPILE_FOR_PCA10040
#warning "Compiling for PCA10040"

#define HEARTBEAT_LED_GPIO_NUM PCA10040_GPIO17_LED_1

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

/* CC1101 pinout:
 * GD0 goes to ?
 * GD1 not pinned out on Solu 8-pin module
 * GD2 goes to ?
 * CS goes to ?
 * MOSI, MISO, SCK pins. So use on "SPI0"
 */
#define SPI0_CC1101_CS_GPIO PCA10040_GPIO3

/* I2C1:
 * Has LIS2DH12 on it, LED screen 7-segment
 */
#define I2C1_SCL_PIN    PCA10040_GPIO28
#define I2C1_SDA_PIN    PCA10040_GPIO29

#elif COMPILE_FOR_FEATHER
#warning "Compiling for nRF52 Bluefruit Feather"
#define HEARTBEAT_LED_GPIO_NUM FEATHER_ONBOARD_GPIO17_LED1

// For EV1527Timer
#define RADIO_TX_GPIO   FEATHER_GPIO20_DFU
// For EV1527SPI
#define SPI2_SCK_PIN    FEATHER_GPIO15_15
#define SPI2_MOSI_PIN   FEATHER_GPIO16_16

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
#define SPI0_EINK_CS_GPIO       FEATHER_GPIO31_31 // connects to featherWing board
#define EINK_DC_GPIO            FEATHER_GPIO11_11 // connects to featherWing board

/* CC1101 pinout:
 * GD0 goes to ?
 * GD1 not pinned out on Solu 8-pin module
 * GD2 goes to ?
 * CS goes to ?
 * MOSI, MISO, SCK pins. So use on "SPI0"
 */
#define SPI0_CC1101_CS_GPIO FEATHER_GPIO29_A5

/* I2C1:
 * Has LIS2DH12 on it, LED screen 7-segment
 */
#define I2C1_SCL_PIN    FEATHER_GPIO26_SCL
#define I2C1_SDA_PIN    FEATHER_GPIO25_SDA

#else
#error "define a board please"
#endif // #if COMPILE_FOR_PCA10040

#ifdef __cplusplus
}
#endif

#endif /* SRC_VERSION_H_ */
