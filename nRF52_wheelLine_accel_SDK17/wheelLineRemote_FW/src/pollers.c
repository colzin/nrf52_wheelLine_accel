/*
 * pollers.c
 *
 *  Created on: Jan 19, 2024
 *      Author: Collin Moore
 */

#include "pollers.h"

#include "nrf_log.h"

/*************************************************************************************
 *  Definitions
 ************************************************************************************/
#define MAX_NUM_POLL_FUNCTIONS 16
/*************************************************************************************
 *  Variables
 ************************************************************************************/

// Make sure all pointers are initted to zeros.
static pollerFunction_t m_pollFunctions[MAX_NUM_POLL_FUNCTIONS];
/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

bool pollers_registerPoller(pollerFunction_t pFn)
{
    for (uint32_t i = 0; i < MAX_NUM_POLL_FUNCTIONS; i++)
    {
        if (NULL == m_pollFunctions[i])
        {
            m_pollFunctions[i] = pFn;
            return true;
        }
    }
    NRF_LOG_ERROR("No room for poller function!");
    return false;
}

void pollers_runAll(void)
{
    for (uint32_t i = 0; i < MAX_NUM_POLL_FUNCTIONS; i++)
    {
        if (NULL != m_pollFunctions[i])
        { // Call each function
            m_pollFunctions[i]();
        }
    }
}

void pollers_init(void)
{
    for (uint32_t i = 0; i < MAX_NUM_POLL_FUNCTIONS; i++)
    {
        m_pollFunctions[i] = NULL;
    }
}
