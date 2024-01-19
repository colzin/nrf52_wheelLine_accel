/*
 * pollers.h
 *
 *  Created on: Jan 19, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_POLLERS_H_
#define SRC_POLLERS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
/*************************************************************************************
 *  Definitions
 ************************************************************************************/
typedef void (*pollerFunction_t)(void);

/*************************************************************************************
 *  Functions
 ************************************************************************************/
bool pollers_registerPoller(pollerFunction_t pFn);

void pollers_runAll(void);

void pollers_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_POLLERS_H_ */
