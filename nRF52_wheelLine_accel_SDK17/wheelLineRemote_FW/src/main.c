/*
 * main.c
 *
 *  Created on: Jan 17, 2024
 *      Author: Collin Moore
 */

#include "sdk_config.h"

#if NRF_SDH_ENABLED
#include "bleStuff.h"
#endif // #if NRF_SDH_ENABLED

#include "pollers.h"

#include <stdint.h>
#include "uptimeCounter.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

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

static void initializeInputs(void)
{ // Inputs to our system

  // Init any input pins, ADC, etc so we have those inputs set up and polled early.
}

static void initializeOutputs(void)
{ // outputs from our system, may make decisions based on pollers run as inputs
// TODO Init output pin managers
    heartblink_init();
}
static void log_init(void)
{
    NRF_LOG_INIT(uptimeCounter_getUptimeMs);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

int main(void)
{
    uptimeCounter_zero();
    // Get logging up
    log_init();
    NRF_LOG_DEBUG("WheelLineRemote start");
    // Start uptime tick timer, so we know what time it is
    uptimeCounter_init();
    // Zero the pollers, so future calls can init
    pollers_init();
    // Run initialization functions as needed, they may register pollers now
    initializeInputs();
    initializeOutputs();
    // TODO start BLE for dropping to DFU, softDevice calls
#if NRF_SDH_ENABLED
    bleStuff_init();
    bleStuff_printBLEVersion();
#endif // #if NRF_SDH_ENABLED

    uint32_t lastPoll_ms = uptimeCounter_getUptimeMs();
    uint32_t ms_now;
    for (;;)
    {
        // TODO once softDevice is in place, check for and service events
        ms_now = uptimeCounter_getUptimeMs();
        if (lastPoll_ms != ms_now)
        {
            pollers_runAll();
        }
        // Go to low-power sleep between polls
#if NRF_SDH_ENABLED
        sd_app_evt_wait();
#endif // #if NRF_SDH_ENABLED

    }
}
