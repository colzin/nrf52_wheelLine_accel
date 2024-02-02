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

void _4digit7seg_writeDisplay(void);

void _4digit7seg_printU32(uint32_t n, int base);
void _4digit7seg_println(void);
void _4digit7seg_printlnStr(const char* c);
void _4digit7seg_printlnChar(char c);
void _4digit7seg_printlnU32(uint32_t b, int base);

        void _4digit7seg_writeDigitRaw(uint8_t d, uint8_t bitmask);
        void _4digit7seg_writeColon(void);

        uint32_t _4digit7seg_writeChar(char c);

        uint32_t _4digit7seg_writeStr(const char* buffer, uint32_t size);

        void _4digit7seg_writeDigitAscii(uint8_t d, uint8_t c, bool dot);

        void _4digit7seg_init(void);

#ifdef __cplusplus
    }
#endif
#endif /* SRC__4DIGIT7SEG_H_ */
