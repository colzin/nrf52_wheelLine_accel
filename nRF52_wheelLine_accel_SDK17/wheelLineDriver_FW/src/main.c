/*
 * main.c
 *
 *  Created on: Jan 17, 2024
 *      Author: Collin Moore
 */

#include "sdk_config.h"

#include "_4digit7seg.h"

#if NRF_SDH_ENABLED
#include "bleStuff.h"
#endif // #if NRF_SDH_ENABLED

#include "cc1101.h"
#include "ev1527SPI.h"

#include "heartbeatBlink.h"
#include "pollers.h"
#include "relayGpios.h"
#include "rttTerminal.h"
#include "sh1107I2C.h"
#include <stdint.h>
#include "uptimeCounter.h"
#include "version.h"

#define NRF_LOG_MODULE_NAME main
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*************************************************************************************
 *  Definitions
 ************************************************************************************/
#if NRF_SDH_BLE_ENABLED
#define RUN_BLE 1
#endif // #if NRF_SDH_BLE_ENABLED

#define READ_RESETREAS 0

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
    rttTerminal_init();

#ifdef UART_TX_PIN
    uartTerminal_init();
#endif // #ifdef UART_TX_PIN

    cc1101_init(cc1101_packetRX); // We listen by default
}

static void initializeOutputs(void)
{ // outputs from our system, may make decisions based on pollers run as inputs
// TODO Init output pin managers
    heartblink_init();
    _4digit7seg_init();
    sh1107I2C_init();
    relayGpios_init(); // Outputs to relays
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
#if COMPILE_FOR_PCA10040
    NRF_LOG_DEBUG("%s start, compiled for PCA10040", DEVICE_NAME);
#elif COMPILE_FOR_FEATHER
    NRF_LOG_DEBUG("%s start, compiled for FEATHER", DEVICE_NAME);
#endif //

    // Start uptime tick timer, so we know what time it is
    uptimeCounter_init();
    // Zero the pollers, so future calls can init
    pollers_init();

#if READ_RESETREAS
    NRF_LOG_INFO("Reset reason 0x%x", NRF_POWER->RESETREAS);
#endif // #if READ_RESETREAS
    // Run initialization functions as needed, they may register pollers now
    initializeInputs();
    initializeOutputs();
    // Put version into a string on the screen
    char strBuf[8]; // Could have dots
    int strLen = snprintf(strBuf, sizeof(strBuf), "v%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_SUBMINOR);
    if (0 < strLen)
    {
        _4digit7seg_writeStr(strBuf);
    }
    // TODO start BLE for dropping to DFU, softDevice calls
#if NRF_SDH_ENABLED && RUN_BLE
    bleStuff_init();
    bleStuff_printBLEVersion();
#endif // #if NRF_SDH_ENABLED && RUN_BLE

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
#if NRF_SDH_ENABLED && RUN_BLE
        sd_app_evt_wait();
#endif // #if NRF_SDH_ENABLED && RUN_BLE

    }
}
