/*
 * sh1107I2C.h
 *
 *  Created on: Feb 7, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_SH1107I2C_H_
#define SRC_SH1107I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "version.h"
#if COMPILE_SH1107
void sh1107I2C_init(void);

#endif // #if COMPILE_SH1107

#ifdef __cplusplus
}
#endif

#endif /* SRC_SH1107I2C_H_ */
