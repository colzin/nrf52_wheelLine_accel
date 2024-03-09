/*
 * remoteControlManager.c
 *
 *  Created on: Mar 8, 2024
 *      Author: Collin
 */

#include "remoteControlManager.h"

#include "cc1101.h"
#include "globalInts.h"
#include "pollers.h"
#include "uptimeCounter.h"

#define NRF_LOG_MODULE_NAME remoteMgr
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/
#define TX_ITVL_MS 500 // Send this often
/*************************************************************************************
 *  Variables
 ************************************************************************************/
static uint32_t m_lastTx_ms;
static machineState_t m_lastState;
/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static void runEnginePoll(machineState_t desiredState)
{
    if (uptimeCounter_elapsedSince(m_lastTx_ms) >= TX_ITVL_MS)
    {
        // Send run command, solenoids to neutral
        uint8_t byteToSend = (uint8_t)desiredState;
        cc1101_sendPacket(&byteToSend, 1);
        m_lastTx_ms = uptimeCounter_getUptimeMs();
    }
}

static void idlePoll(void)
{
    if (machState_killEngine != m_lastState || machState_justPoweredOn != m_lastState)
    {
        cc1101_setIdle(true); // Stop transmitting, don't transmit any more.
        m_lastTx_ms = uptimeCounter_getUptimeMs();
    }
    else if (uptimeCounter_elapsedSince(m_lastTx_ms) >= TX_ITVL_MS)
    {
        if (statusByteState_idle != cc1101_getLastState())
        {
            cc1101_setIdle(true); // Stop transmitting, don't transmit any more.
        }
        m_lastTx_ms = uptimeCounter_getUptimeMs();
    }
}

static void rmtCtlPoll(void)
{
    // TODO something should read the buttons and set the appropriate state before we call this function
    machineState_t currentState = globalInts_getMachineState();
    switch (currentState)
    {
        case machState_killEngine:
            case machState_justPoweredOn:   // Do nothing
            idlePoll();
        break;
        case machState_startEngine:
            case machState_runEngineHydFwd:
            case machState_runEngineHydIdle:
            case machState_runEngineHydRev:
            runEnginePoll(currentState);
        break;
        default:
            if (currentState != m_lastState)
            {
                NRF_LOG_WARNING("Don't know what to do in state %d", currentState);
            }
        break;
    }
    m_lastState = currentState;
}

void remoteControlManager_init()
{
    pollers_registerPoller(rmtCtlPoll);
}
