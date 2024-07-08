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

#include "version.h"
#if COMPILE_RADIO_900T20D
#warning "can't use terminal with 900T20D"
#else
#include <sdk_errors.h>

void uartTerminal_init(void);

#endif // #if COMPILE_FOR_PCA10040

#ifdef __cplusplus
}
#endif

#endif /* SRC_UARTTERMINAL_H_ */
