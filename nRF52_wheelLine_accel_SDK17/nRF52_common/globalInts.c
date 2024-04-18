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

//static int32_t m_int32s[GLOBALINTs_NUM_INT32s];
static machineState_t m_machState;
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
void globalInts_setMachineState(machineState_t st)
{
    if (st <= machState_killEngine)
    {
        if (st != m_machState)
        {
            NRF_LOG_INFO("Machine state from %d to %d", m_machState, st);
#ifdef UART_TX_PIN
            char strBuf[64];
            int strLen = snprintf((char*)strBuf, sizeof(strBuf), "  Set state to %d\n", st);
            if (0 < strLen)
            {
                uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
            }
#endif // #ifdef UART_TX_PIN

        }
        m_machState = st;
    }
    else
    {
        NRF_LOG_ERROR("Tried to set invalid state %d, setting to kill engine", st);
        m_machState = machState_killEngine;
    }

}
