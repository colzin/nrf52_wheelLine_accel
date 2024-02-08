/*
 * cc1101.h
 *
 *  Created on: Feb 5, 2024
 *      Author: collin Moore
 */

#ifndef cc1101_H_
#define cc1101_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// mid level utility

void cc1101_setIdle(bool flushFifos);

// Higher level functions

void cc1101_init(void);

#ifdef __cplusplus
}
#endif

#endif /* cc1101_H_ */
