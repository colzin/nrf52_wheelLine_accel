/*
 * bleStuff.h
 *
 *  Created on: Jan 19, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_BLESTUFF_H_
#define SRC_BLESTUFF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sdk_config.h"

#if NRF_SDH_ENABLED
void bleStuff_init();
void bleStuff_printBLEVersion(void);

#endif // #if NRF_SDH_ENABLED

#ifdef __cplusplus
}
#endif

#endif /* SRC_BLESTUFF_H_ */
