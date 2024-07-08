/*
 * _900t20d.h
 *
 *  Created on: Feb 5, 2024
 *      Author: collin Moore
 */

#ifndef _900t20d_H_
#define _900t20d_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "version.h"
#if COMPILE_RADIO_900T20D
#include <stdbool.h>
#include <stdint.h>

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define PACKET_ON_AIR_MS (80+9) // Seems to take 80ms or so

#define TX_TIMEOUT_MS (PACKET_ON_AIR_MS*2+3) // Should send in this time
#define RX_AFTER_TX_TIMEOUT_MS (PACKET_ON_AIR_MS+TX_TIMEOUT_MS+3) // Should hear back in this much time

#define _900t20d_MIN_SEND_ITVL_MS (TX_TIMEOUT_MS+RX_AFTER_TX_TIMEOUT_MS+20) //

#define TX_TEST_ITVL_MS  0 // non-zero to run TX test poll

#define PKT_SIZE_FIXED 1 // TODO switch to variable later to save power.
#if PKT_SIZE_FIXED
#define PKT_LEN 3
#else
#define PKT_LEN_VARIABLE 1
#endif // #if PKT_SIZE_FIXED

/*************************************************************************************
 * Functions
 ************************************************************************************/

void _900t20d_setOutputPower(int8_t power_dBm);

bool _900t20d_sendPacket(uint8_t byte);
void _900t20d_init(void);

#endif // #if COMPILE_RADIO_900T20D

#ifdef __cplusplus
}
#endif

#endif /* _900t20d_H_ */
