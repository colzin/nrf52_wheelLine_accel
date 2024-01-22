/*
 * uptimeCounter.c
 *
 *  Created on: Jan 18, 2024
 *      Author: Collin Moore
 */

#include "uptimeCounter.h"

#include "app_timer.h"

#define NRF_LOG_MODULE_NAME uptimeCounter
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/
#define IDLE_TICK_ITVL_MS 500 // TODO we may not need to idle, if user shuts off power when done.
#define RUNNING_TICK_ITVL_MS 5

/*************************************************************************************
 *  Variables
 ************************************************************************************/

APP_TIMER_DEF(m_uptimeTimer); /**< App timer uses RTCs underneath. */

static volatile uint32_t m_roughUptime_ms;

static uint32_t m_nextPoll_ms;
/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static void msTickHandler(void* pContext)
{
    m_roughUptime_ms += m_nextPoll_ms;
}

uint32_t uptimeCounter_getUptimeMs(void)
{
    return m_roughUptime_ms;
}

void uptimeCounter_setTimeoutItvl(uint32_t desiredItvl_ms)
{
    if (desiredItvl_ms < 1)
    {
        desiredItvl_ms = 1;
        NRF_LOG_WARNING("Flooring interval at 1ms");
    }
    if (desiredItvl_ms != m_nextPoll_ms)
    {
        // re-set the timer
        app_timer_stop(m_uptimeTimer); // It's ok if it returns invalid state, if not running already.
        ret_code_t ret = app_timer_start(m_uptimeTimer, APP_TIMER_TICKS(desiredItvl_ms), NULL);
        if (NRF_SUCCESS != ret)
        {
            m_nextPoll_ms = 0; // Set to invalid
            NRF_LOG_ERROR("ERROR %d starting app timer", ret);
        }
        else
        {
            m_nextPoll_ms = desiredItvl_ms;
            NRF_LOG_DEBUG("Changed interval to %d ms", m_nextPoll_ms);
        }
    }
}

void uptimeCounter_zero(void)
{
    m_roughUptime_ms = 0; // no overflow counter yet.
}

uint32_t uptimeCounter_elapsedSince(uint32_t timestamp_ms)
{
    return utils_elapsedU32Ticks(timestamp_ms, m_roughUptime_ms);
}
void uptimeCounter_init(void)
{
    // Set up the timer
    ret_code_t err_code = app_timer_init();
    if (NRF_SUCCESS == err_code)
    {
        err_code = app_timer_create(&m_uptimeTimer, APP_TIMER_MODE_REPEATED, msTickHandler);
    }
    // Set the frequency
    m_nextPoll_ms = 0; // Set to invalid value so that it'll update
    uptimeCounter_setTimeoutItvl(RUNNING_TICK_ITVL_MS);
}
