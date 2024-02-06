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

typedef enum
{
    PKT_LEN_FIXED,
    PKT_LEN_VARIABLE,
    PKT_LEN_INFINITE
} pkt_format_t;

uint8_t* cc1101_receivePacket(uint32_t* pNumBytes);
void cc1101_resetRx();

uint8_t* cc1101_tryReceiveData(uint32_t* pNumBytes);

// mid level utility
pkt_format_t cc1101_getPacketConfigurationMode();

void cc1101_sidle(void);
void cc1101_flushRxFifo(void);
void cc1101_flushTxFifo(void);
bool cc1101_setTxState(void);
bool cc1101_setRxState(void);

// Higher level functions
bool cc1101_selfTest();
void cc1101_initASKTx_myStudio(void);
void cc1101_init(void);

#ifdef __cplusplus
}
#endif

#endif /* cc1101_H_ */
