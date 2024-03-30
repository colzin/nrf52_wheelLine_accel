/*
 * ev1527SPI.h
 *
 *  Created on: Feb 5, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_EV1527SPI_H_
#define SRC_EV1527SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "globalInts.h"
#include <stdint.h>

void ev1527_setSendState(machineState_t currentState);

void ev1527SPI_init(uint8_t txPin);

#ifdef __cplusplus
}
#endif

#endif /* SRC_EV1527SPI_H_ */
