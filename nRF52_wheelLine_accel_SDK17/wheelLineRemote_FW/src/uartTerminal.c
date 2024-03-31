/*
 * uartTerminal.c
 *
 *  Created on: Feb 6, 2024
 *      Author: Collin Moore
 */

#include "uartTerminal.h"

#include "cc1101.h" // TO set output power
#include "globalInts.h"
#include "nrf_delay.h"
#include "nrfx_uarte.h"
#include "pollers.h"

#define NRF_LOG_MODULE_NAME uartTerm
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
        NRF_LOG_INFO("Setting power to %d dBm", m_accumulator);
        cc1101_setOutputPower((int8_t)m_accumulator);
        m_parserState = parserState_default;
    }
    else
    {
        NRF_LOG_WARNING("Error, moving to default");
        m_parserState = parserState_default;
    }
}

static void defaultParser(uint8_t byte)
{
    switch (byte)
    {
        break;
        case 'i':
            NRF_LOG_WARNING("Setting CC1101 to IDLE state")
            ;
            cc1101_setIdle(true);
        break;
        case 'o':
            NRF_LOG_WARNING("Setting engine ON idle mode")
            ;
            globalInts_setMachineState(machState_runEngineHydIdle);
        break;
        case 'p':
            NRF_LOG_WARNING("Enter PA power")
            ;
            m_accumulator = 0;
            m_negative = false;
            m_parserState = parserState_receivingPower;
        break;
        case 'r':
            NRF_LOG_WARNING("REbooting")
            ;
            // Delay for prints to finish
            nrf_delay_ms(2);
            NVIC_SystemReset();
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

static void uartPoll(void)
{
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

static nrfx_uarte_t m_uartInst = NRFX_UARTE_INSTANCE(0);

void uartTerminal_init(void)
{
    m_uartInst.
    nrfx_uarte_init(p_instance, p_config, event_handler);


    m_parserState = parserState_default;
    pollers_registerPoller(uartPoll);
}
