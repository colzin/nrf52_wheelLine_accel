/*
 * rttTerminal.c
 *
 *  Created on: Feb 6, 2024
 *      Author: Collin Moore
 */

#include "rttTerminal.h"

#include "cc1101.h"
#include "nrf_delay.h"
#include "pollers.h"
#include "SEGGER_RTT.h"

#define NRF_LOG_MODULE_NAME rttTerm
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/*************************************************************************************
 *  Variables
 ************************************************************************************/

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static void rttPoll(void)
{
    int command = SEGGER_RTT_GetKey();
    switch (command)
    {
        case -1:
            // This what it returns when there is no character on terminal, ignore, return
            return;
        break;
        case 's':
            NRF_LOG_WARNING("Setting RX state")
            ;
            cc1101_setRxState();
        break;
        case 't':
            break;
        case 'r':
            NRF_LOG_WARNING("REbooting")
            ;
            // Delay for prints to finish
            nrf_delay_ms(2);
            NVIC_SystemReset();
        break;

    }
}

void rttTerminal_init(void)
{
    pollers_registerPoller(rttPoll);
}
