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

#include "version.h"
#if COMPILE_RADIO_CC1101
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

#define BASIC_ASK_433 0
#if BASIC_ASK_433
#define ASK_PATABLE 1
#else
#define SENSITIVE_433 0
#if !SENSITIVE_433
#define FORUM_433 0
#if !FORUM_433
#define SENSITIVE_915 0
#if !SENSITIVE_915
#define FORUM_915 0
#if !FORUM_915
#define locustcox_433 0
#if !locustcox_433
#define SENSITIVE_ASK433 1
#if SENSITIVE_ASK433
#define ASK_PATABLE 1
#else

#endif // #if SENSITIVE_ASK433
#endif // #if locustcox_433
#endif // #if FORUM_915
#endif // #if SENSITIVE_915
#endif // #if FORUM_433
#endif // #if SENSITIVE_433
#endif // #if BASIC_ASK_433

#define PACKET_ON_AIR_MS (80+9) // Seems to take 80ms or so

#define TX_TIMEOUT_MS (PACKET_ON_AIR_MS*2+3) // Should send in this time
#define RX_AFTER_TX_TIMEOUT_MS (PACKET_ON_AIR_MS+TX_TIMEOUT_MS+3) // Should hear back in this much time

#define CC1101_MIN_SEND_ITVL_MS (TX_TIMEOUT_MS+RX_AFTER_TX_TIMEOUT_MS+20) //

#define TX_TEST_ITVL_MS  0 // non-zero to run TX test poll

#define PKT_SIZE_FIXED 1 // TODO switch to variable later to save power.
#if PKT_SIZE_FIXED
#define PKT_LEN 3
#else
#define PKT_LEN_VARIABLE 1
#endif // #if PKT_SIZE_FIXED

#define GDO2_STATUS 0 // 1 to use pin
#if GDO2_STATUS
#else
#define STATUS_POLL_ITVL_MS (100) // Try not to poll while RXing or TXing a packet
#endif // #if GDO2_STATUS

typedef enum
{
    closeInRx_0dB = 0,
    closeInRx_6dB,
    closeInRx_12dB,
    closeInRx_18dB
} fifothr_closeInRx_t;

/*************************************************************************************
 * Functions
 ************************************************************************************/
chipStatusByteState_t cc1101_getLastState(void);

// Low-level functions.
void cc1101_setIdle(bool flushFifos);

bool cc1101_sendPacket(uint8_t byte);

cc1101Mode_t cc1101_readMode(void);
void cc1101_setOutputPower(int8_t power_dBm);

bool cc1101_setCloseInRx(fifothr_closeInRx_t atten);

void cc1101_init(cc1101Mode_t desired);

#endif // #if COMPILE_RADIO_CC1101

#ifdef __cplusplus
}
#endif

#endif /* cc1101_H_ */
