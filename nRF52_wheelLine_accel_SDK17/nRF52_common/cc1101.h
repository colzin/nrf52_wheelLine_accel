/*
 * cc1101.h
 *
 *  Created on: Feb 5, 2024
 *      Author: collin Moore
 */

#ifndef cc1101_H_
#define cc1101_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/*************************************************************************************
 *  Definitions
 ************************************************************************************/
typedef enum
{
    statusByteState_idle = 0,
    statusByteState_rx,
    statusByteState_tx,
    statusByteState_fastTxReady,
    statusByteState_cal,
    statusByteState_settling,
    statusByteState_rxOverflow,
    statusByteState_txUnderflow,
} chipStatusByteState_t;

typedef enum
{
    cc1101_unknownMode = 0,
    cc1101_asyncTX,
    cc1101_packetTX,
    cc1101_packetRX,
} cc1101Mode_t;

/*************************************************************************************
 * Functions
 ************************************************************************************/
chipStatusByteState_t cc1101_getLastState(void);

// Low-level functions.
void cc1101_setIdle(bool flushFifos);

bool cc1101_sendPacket(uint8_t* pBytes, uint8_t len);

cc1101Mode_t cc1101_readMode(void);
void cc1101_setOutputPower(int8_t power_dBm);

void cc1101_init(cc1101Mode_t desired);

#ifdef __cplusplus
}
#endif

#endif /* cc1101_H_ */
