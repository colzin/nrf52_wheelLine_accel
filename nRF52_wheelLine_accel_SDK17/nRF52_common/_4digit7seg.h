/*
 * _4digit7seg.h
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#ifndef SRC__4DIGIT7SEG_H_
#define SRC__4DIGIT7SEG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    dispState_off,
    dispState_onSolid,
    dispState_blink2Hz,
    dispState_blink1Hz,
    dispState_blink0_5Hz,
}dispState_t;

void _4digit7seg_setDisplayState(dispState_t desired);

void _4digit7seg_setBrightness(uint8_t zeroTo15);

void _4digit7seg_writeStr(const char* buffer, uint8_t size);

void _4digit7seg_writeDigitAscii(uint8_t d, uint8_t c, bool dot);

void _4digit7seg_init(void);

#ifdef __c
plusplus}
#endif
#endif /* SRC__4DIGIT7SEG_H_ */
