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

#define ASK_433 0
#if !ASK_433
#define SENSITIVE_433 1
#if !SENSITIVE_433
#define FORUM_433 0
#if !FORUM_433
#define SENSITIVE_915 0
#if !SENSITIVE_915
#define FORUM_915 0
#if !FORUM_915
#define locustcox_433 0
#endif // !FORUM_915
#endif // #if !SENSITIVE_915
#endif // #if !FORUM_433
#endif // #if SENSITIVE_433
#endif // #if !ASK_433

#define PACKET_ON_AIR_MS (80+9) // Seems to take 80ms or so

#define TX_TIMEOUT_MS (PACKET_ON_AIR_MS*2+3) // Should send in this time
#define RX_AFTER_TX_TIMEOUT_MS (PACKET_ON_AIR_MS+TX_TIMEOUT_MS+3) // Should hear back in this much time

#define CC1101_MIN_SEND_ITVL_MS (TX_TIMEOUT_MS+RX_AFTER_TX_TIMEOUT_MS+20) //

#define GDO2_STATUS 0 // 1 to use pin
#if GDO2_STATUS
#else
#define STATUS_POLL_ITVL_MS (100) // Try not to poll while RXing or TXing a packet
#endif // #if GDO2_STATUS

/*************************************************************************************
 * Functions
 ************************************************************************************/
chipStatusByteState_t cc1101_getLastState(void);

// Low-level functions.
void cc1101_setIdle(bool flushFifos);

bool cc1101_sendPacket(uint8_t byte);

cc1101Mode_t cc1101_readMode(void);
void cc1101_setOutputPower(int8_t power_dBm);

void cc1101_init(cc1101Mode_t desired);

#ifdef __cplusplus
}
#endif

#endif /* cc1101_H_ */
