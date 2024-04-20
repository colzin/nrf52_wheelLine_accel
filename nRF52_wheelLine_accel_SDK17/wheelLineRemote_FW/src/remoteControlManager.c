/*
 * remoteControlManager.c
 *
 *  Created on: Mar 8, 2024
 *      Author: Collin
 */

#include "remoteControlManager.h"

#include "cc1101.h"
#include "ev1527SPI.h"
#include "globalInts.h"
#include "pollers.h"
#include "uptimeCounter.h"

#define NRF_LOG_MODULE_NAME remoteMgr
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/
#define TX_ITVL_MS 2000 // Send this often
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
    uint32_t msSinceLastTx = uptimeCounter_elapsedSince(m_lastTx_ms);
    if (TX_ITVL_MS < msSinceLastTx && CC1101_MIN_SEND_ITVL_MS < msSinceLastTx)
    {
        cc1101Mode_t mode = cc1101_readMode();
        switch (mode)
        {
            case cc1101_packetTX:
                {
                // Send byte with desired state
                cc1101_sendPacket((uint8_t)desiredState);
                m_lastTx_ms = uptimeCounter_getUptimeMs();
            }
            break;
            case cc1101_asyncTX:
                ev1527_setSendState(desiredState);
            break;
            default:
                break;
        }
    }
}

static void idlePoll(machineState_t currentState)
{
    if (currentState != m_lastState)
    {
        cc1101Mode_t mode = cc1101_readMode();
        switch (mode)
        {
            case cc1101_packetTX:
                cc1101_setIdle(true); // Stop transmitting, don't transmit any more.
            break;
            case cc1101_asyncTX:
                ev1527_setSendState(currentState);
                cc1101_setIdle(true); // Stop transmitting, don't transmit any more.
            break;
            default:
                break;
        }

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
            idlePoll(currentState);
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
