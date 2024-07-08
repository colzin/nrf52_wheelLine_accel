/*
 * uarte0.h
 *
 *  Created on: Feb 6, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_UARTE0_H_
#define SRC_UARTE0_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sdk_errors.h>

#define UARTE0_MAX_UART_DATA_LEN 128

#include <stdbool.h>
#include <stdint.h>

ret_code_t uarte0_enqueue(const uint8_t* pBytes, uint32_t numBytes);
uint32_t uarte0_rxBytes(void);
// check if done sending
bool uarte0_isTxDone(void);
// Returns true if RX byte is read out
bool uarte0_tryReadByte(uint8_t* pByte);
bool uarte0_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_UARTE0_H_ */
