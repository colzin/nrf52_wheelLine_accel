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

// PCA10040 LEDs on 17, 18, 19, 29, active low
#define LED_1          17
#define LED_2          18
#define LED_3          19
#define LED_4          20

#define HEARTBEAT_LED_GPIO_NUM LED_1
#define RADIO_TX_GPIO LED_2
#define ISR_DEBUG_GPIO LED_3

// Buttons short to GND
#define BUTTON_1       13
#define BUTTON_2       14
#define BUTTON_3       15
#define BUTTON_4       16

#define UNUSED_PIN0     0 // p0.00, any physical unused pin
#define UNUSED_PIN1     1 // p0.01, any physical unused pin

#define I2S_LRCLK_PIN   UNUSED_PIN0 // Need a dummy pin

#define I2S_SCK_PIN     UNUSED_PIN1 // Need a dummy pin
#define I2S_SDOUT_PIN   RADIO_TX_GPIO // p0.02, WE DO USE THIS
#ifdef __cplusplus
}
#endif

#endif /* SRC_VERSION_H_ */
