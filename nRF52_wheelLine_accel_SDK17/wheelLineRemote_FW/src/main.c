/*
 * main.c
 *
 *  Created on: Jan 17, 2024
 *      Author: Collin Moore
 */

#include "sdk_config.h"

#include "version.h" // #defines for other #includes
#include "_4digit7seg.h"

#if NRF_SDH_ENABLED
#include "bleStuff.h"
#endif // #if NRF_SDH_ENABLED

#if COMPILE_RADIO_CC1101
#include "cc1101.h"
#endif // #if COMPILE_RADIO_CC1101

#if COMPILE_RADIO_900T20D
#include "_900t20d.h"
#endif // #if COMPILE_RADIO_900T20D

#include "globalInts.h"
#include "pinStuff.h"
#include "heartbeatBlink.h"
#include "lis2dh.h"
#include "pollers.h"
#include "remoteControlManager.h"
#include "rttTerminal.h"
#include "sh1107I2C.h"
#include <stdint.h>

#ifdef UART_TX_PIN
#include "uartTerminal.h"
#endif // #ifdef UART_TX_PIN

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
#define RUN_BLE 0
#endif // #if NRF_SDH_BLE_ENABLED

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

    pinStuff_init(); // Sets the machine state
}

static void initializeOutputs(void)
{ // outputs from our system, may make decisions based on pollers run as inputs

    remoteControlManager_init();
    heartblink_init();
    _4digit7seg_init();
#if COMPILE_RADIO_CC1101
    cc1101_init(cc1101_packetTX); // TX on Remote, Rx on driver
#endif // #if COMPILE_RADIO_CC1101
#if COMPILE_RADIO_900T20D
    _900t20d_init();
#endif // #if COMPILE_RADIO_900T20D
    sh1107I2C_init();

}
static void log_init(void)
{
    NRF_LOG_INIT(uptimeCounter_getUptimeMs);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

#if NRF_SDH_ENABLED && RUN_BLE
#else
/**@brief Function for the Power Management.
 */
static void power_manage(void)
{
    // Use directly __WFE and __SEV macros since the SoftDevice is not available.
    // Wait for event.
    __WFE();
    // Clear Event Register.
    __SEV();
    __WFE();
}
#endif //
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

//    NRF_LOG_DEBUG("DEVid 0x%x %x, addr 0x%x %x", NRF_FICR->DEVICEID[1],
//                  NRF_FICR->DEVICEID[0],
//                  NRF_FICR->DEVICEADDR[1], NRF_FICR->DEVICEADDR[0]);

    // Start uptime tick timer, so we know what time it is
    uptimeCounter_init();
    // Zero the pollers, so future calls can init
    pollers_init();
    // Run initialization functions as needed, they may register pollers now
    initializeInputs();
    initializeOutputs();
    // Put version into a string on the screen
    char strBuf[8]; // Could have dots
    int strLen = snprintf(strBuf, sizeof(strBuf), "v%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_SUBMINOR);
    if (0 < strLen)
    {
        _4digit7seg_writeStr(strBuf);
#ifdef UART_TX_PIN
        uartTerminal_enqueueToUSB((uint8_t*)strBuf, (uint32_t)strLen);
#endif // #ifdef UART_TX_PIN
    }
    // TODO start BLE for dropping to DFU, softDevice calls
#if NRF_SDH_ENABLED && RUN_BLE
    bleStuff_init();
    bleStuff_printBLEVersion();
#else
    nrf_drv_clock_lfclk_request(); // to keep timer running without softdevice
    nrfx_clock_lfclk_start();
    NRF_LOG_INFO("LFCLK is %s", nrfx_clock_lfclk_is_running() ? "Running" : "off");
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
#else
        power_manage();
#endif // #if NRF_SDH_ENABLED && RUN_BLE

    }
}
