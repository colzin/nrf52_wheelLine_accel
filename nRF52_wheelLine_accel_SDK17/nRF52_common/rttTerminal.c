/*
 * rttTerminal.c
 *
 *  Created on: Feb 6, 2024
 *      Author: Collin Moore
 */

#include "rttTerminal.h"

#include "version.h"
#if COMPILE_RADIO_CC1101
#include "cc1101.h"
#endif // #if COMPILE_RADIO_CC1101
#if COMPILE_RADIO_900T20D
#include "_900t20d.h"
#endif // #if COMPILE_RADIO_900T20D

#include "globalInts.h"
#include "nrf_delay.h"
#include "pollers.h"
#include "SEGGER_RTT.h"

#define NRF_LOG_MODULE_NAME rttTerm
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define LF_CHAR 0xA
#define CR_CHAR 0xD

typedef enum
{
    parserState_default,
    parserState_receivingPower,
} parserState_t;
/*************************************************************************************
 *  Variables
 ************************************************************************************/
static parserState_t m_parserState;
static int32_t m_accumulator;
static bool m_negative;
/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static void parsePowerdigit(uint8_t byte)
{
    if ('-' == byte)
    {
        m_negative = true;
    }
    else if ('0' <= byte && '9' >= byte)
    {
        m_accumulator *= 10;
        m_accumulator += (byte - '0');
    }
    else if (LF_CHAR == byte || CR_CHAR == byte)
    {
        if (m_negative)
        {
            m_accumulator = -1 * m_accumulator;
        }
#if COMPILE_RADIO_CC1101
        cc1101_setOutputPower((int8_t)m_accumulator);
#endif // #if COMPILE_RADIO_CC1101
#if COMPILE_RADIO_900T20D
        _900t20d_setOutputPower((int8_t)m_accumulator);
#endif // #if COMPILE_RADIO_900T20D
        m_parserState = parserState_default;
    }
    else
    {
        NRF_LOG_WARNING("Error, moving to default");
        m_parserState = parserState_default;
    }
}

static void parseCloseIn(uint8_t byte)
{
    if ('0' <= byte && '3' >= byte)
    {
#if COMPILE_RADIO_CC1101
        cc1101_setCloseInRx(byte - '0');
#endif // #if COMPILE_RADIO_CC1101
    }
    else
    {
        NRF_LOG_WARNING("Error, moving to default");
        uartTerminal_enqueueToUSB((const uint8_t*)"Error, moving to default\n",
                                  strlen("Error, moving to default\n"));
    }
    m_parserState = parserState_default;
}

static void defaultParser(uint8_t byte)
{
    switch (byte)
    {
        break;
        case 'i':
            NRF_LOG_WARNING("Setting CC1101 to IDLE state")
            ;
#if COMPILE_RADIO_CC1101
            cc1101_setIdle(true);
#endif // #if COMPILE_RADIO_CC1101
        break;
        case 'o':
            NRF_LOG_WARNING("Setting engine ON idle mode")
            ;
            globalInts_setMachineState(machState_runEngineHydIdle);
        break;
        case 'r':
            NRF_LOG_WARNING("REbooting")
            ;
            // Delay for prints to finish
            nrf_delay_ms(2);
            NVIC_SystemReset();
        break;
        case 't':
            case 'T':
            NRF_LOG_WARNING("Enter Radio TX power (dBm)")
            ;
            m_accumulator = 0;
            m_negative = false;
            m_parserState = parserState_receivingPower;
        break;
        case LF_CHAR:
            break;
        case CR_CHAR:
            break;
        default:
            NRF_LOG_INFO("Enter i, o, p, r, ")
            ;
        break;
    }
}

static void rttPoll(void)
{
    int command = SEGGER_RTT_GetKey();
    if (-1 == command)
    { // This what it returns when there is no character on terminal, ignore, return
        return;
    }
//    NRF_LOG_INFO("Received 0x%x", command);
    switch (m_parserState)
    {
        case parserState_default:
            defaultParser((uint8_t)command);
        break;
        case parserState_receivingPower:
            parsePowerdigit((uint8_t)command);
        break;
    }
}

void rttTerminal_init(void)
{
    m_parserState = parserState_default;
    pollers_registerPoller(rttPoll);
}
