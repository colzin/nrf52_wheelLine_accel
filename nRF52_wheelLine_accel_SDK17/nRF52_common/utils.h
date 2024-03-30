/*
 * utils.h
 *
 *  Created on: Jan 19, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_UTILS_H_
#define SRC_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint8_t* utils_writeLEUint32(uint8_t* pBuffer, uint32_t data);
uint8_t* utils_writeLEUint16(uint8_t* pBuffer, uint16_t data);
uint8_t* utils_readBEUint32(uint8_t* pBuffer, uint32_t* data);
uint8_t* utils_writeBEUint32(uint8_t* pBuffer, uint32_t data);

uint32_t utils_elapsedU32Ticks(uint32_t start, uint32_t end);

#ifdef __cplusplus
}
#endif

#endif /* SRC_UTILS_H_ */
