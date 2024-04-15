/*
 * uartTerminal.h
 *
 *  Created on: Feb 6, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_UARTTERMINAL_H_
#define SRC_UARTTERMINAL_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <sdk_errors.h>

ret_code_t uartTerminal_enqueueToUSB(const uint8_t* pBytes, uint32_t numBytes);
void uartTerminal_init(void);


#ifdef __cplusplus
}
#endif

#endif /* SRC_UARTTERMINAL_H_ */
