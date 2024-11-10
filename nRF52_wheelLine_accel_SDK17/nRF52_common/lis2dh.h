/*
 * lis2dh.h
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_LIS2DH_H_
#define SRC_LIS2DH_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "version.h"
#if COMPILE_LIS2DH
void lis2dh_poll(void);
void lis2dh_init(void);
#endif // #if COMPILE_LIS2DH

#ifdef __cplusplus
}
#endif

#endif /* SRC_LIS2DH_H_ */
