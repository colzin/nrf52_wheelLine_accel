/*
 * uptimeCounter.h
 *
 *  Created on: Jan 18, 2024
 *      Author: Collin
 */

#ifndef SRC_UPTIMECOUNTER_H_
#define SRC_UPTIMECOUNTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t uptimeCounter_getUptimeMs(void);

void uptimeCounter_setTimeoutItvl(uint32_t desiredItvl_ms);

void uptimeCounter_zero(void);
uint32_t uptimeCounter_elapsedSince(uint32_t timestamp_ms);

void uptimeCounter_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_UPTIMECOUNTER_H_ */
