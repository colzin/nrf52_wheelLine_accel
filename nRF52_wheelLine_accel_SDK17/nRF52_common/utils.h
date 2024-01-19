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

uint32_t utils_elapsedU32Ticks(uint32_t start, uint32_t end);

#ifdef __cplusplus
}
#endif

#endif /* SRC_UTILS_H_ */
