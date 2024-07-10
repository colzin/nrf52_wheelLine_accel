/*
 * globalInts.c
 *
 *  Created on: Feb 10, 2024
 *      Author: Collin Moore
 */

#include "globalInts.h"

#include "version.h" // To choose UART or not
#ifdef UART_TX_PIN
#include "uartTerminal.h"
#include <stdio.h> // for snprintf
#endif // #ifdef UART_TX_PIN

#define NRF_LOG_MODULE_NAME globalInts
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/*************************************************************************************
 *  Variables
 ************************************************************************************/

// static int32_t m_int32s[GLOBALINTs_NUM_INT32s];
static machineState_t m_machState;
static uint32_t m_machStateStart_ms;
const char* m_machineStateStrings[] = { "PON", "Start", "runIdle", "runFWD", "runREV", "kill" };

static uint64_t g_chipID;

static int8_t g_numRotations;
/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

//int32_t globalInts_getInt32(globalInt32s_t toGet)
//{
//    return m_int32s[toGet];
//}
//void globalInts_setIntInt32(globalInt32s_t toSet, int32_t val)
//{
//    m_int32s[toSet] = val;
//}
machineState_t globalInts_getMachineState(void)
{
    return m_machState;
}

const char* globalInts_getMachStateString(machineState_t st)
{
    if (st <= machState_killEngine)
    {
        return m_machineStateStrings[st];
    }
    return "invalid";
}

void globalInts_setMachineState(machineState_t st)
{
    if (st <= machState_killEngine)
    {
        if (st != m_machState)
        {
            NRF_LOG_INFO("Machine state from %s to %s\n", globalInts_getMachStateString(m_machState),
                         globalInts_getMachStateString(st));
#ifdef UART_TX_PIN
char strBuf[64];
int strLen = snprintf((char*)strBuf, sizeof(strBuf), "  Set state to %d\n", st);
if (0 < strLen)
{
    uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
}
#endif // #ifdef UART_TX_PIN
        }
        if (machState_runEngineHydFwd != st && machState_runEngineHydRev != st)
        {
            NRF_LOG_WARNING("Clearing numRots for machine state that shouldn't move.");
            globalInts_setNumRotations(0);
        }
        m_machState = st;
    }
    else
    {
        NRF_LOG_ERROR("Tried to set invalid state %d, setting to killEngine", st);
        m_machState = machState_killEngine;
    }
    m_machStateStart_ms = uptimeCounter_getUptimeMs();
}

uint32_t globalInts_getMachStateStart_ms(void)
{
    return m_machStateStart_ms;
}

uint64_t globalInts_getChipIDU64(void)
{
    return g_chipID;
}
void globalInts_setChipIDU64(uint64_t chipID)
{
    g_chipID = chipID;
}

int8_t globalInts_getNumRotations(void)
{
    return g_numRotations;
}
void globalInts_setNumRotations(int8_t num)
{
    if (num != g_numRotations)
    {
        NRF_LOG_INFO("\tChanged revs from %d to %d\n", g_numRotations, num);
    }
    g_numRotations = num;
}
