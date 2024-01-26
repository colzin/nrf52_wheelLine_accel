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

#include <stdint.h>

void _4digit7seg_clear(void);

void _4digit7seg_printU32(uint32_t n, int base);
void _4digit7seg_println(void);
void _4digit7seg_printlnStr(const char* c);
void _4digit7seg_printlnChar(char c);
void _4digit7seg_printlnU32(uint32_t b, int base);

uint32_t _4digit7seg_writeChar(char c);

uint32_t _4digit7seg_writeStr(const char* buffer, uint32_t size);

void _4digit7seg_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC__4DIGIT7SEG_H_ */
