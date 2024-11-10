/*
 * main.c
 *
 *  Created on: Jan 17, 2024
 *      Author: Collin Moore
 */

#include "sdk_config.h"
#include "version.h"

#if NRF_SDH_ENABLED
#include "bleStuff.h"
#endif // #if NRF_SDH_ENABLED

#if COMPILE_RADIO_CC1101
#include "cc1101.h"
#endif // #if COMPILE_RADIO_CC1101

#if COMPILE_RADIO_900T20D
#include "_900t20d.h"
#endif // #if COMPILE_RADIO_900T20D

#include "heartbeatBlink.h"

#if COMPILE_LIS2DH
#include "lis2dh.h"
#endif // #if COMPILE_LIS2DH

#include "pollers.h"
#include "relayGpios.h"
#include "rttTerminal.h"
#if COMPILE_SH1107
#include "sh1107I2C.h"
#endif // #if COMPILE_SH1107
#include <stdint.h>

#ifdef UART_TX_PIN
#include "uartTerminal.h"
#endif // #ifdef UART_TX_PIN

#include "uptimeCounter.h"

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
// Init radio
#if COMPILE_RADIO_CC1101
    cc1101_init(cc1101_packetRX); // We listen by default
#elif COMPILE_RADIO_900T20D
    _900t20d_init();
#endif // #if COMPILE_RADIO_CC1101
}

static void initializeOutputs(void)
{ // outputs from our system, may make decisions based on pollers run as inputs
// TODO Init output pin managers
    heartblink_init();
#if _4DIGIT7SEG
    _4digit7seg_init();
#endif // #if _4DIGIT7SEG
#if USE_SH1107
    sh1107I2C_init();
#endif // #if USE_SH1107
    relayGpios_init(); // Outputs to relays
}
static void log_init(void)
{
    NRF_LOG_INIT(uptimeCounter_getUptimeMs);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for the Power Management.
 */
static void power_manage(void)
{
#define FPU_EXCEPTION_MASK 0x0000009F
    __set_FPSCR(__get_FPSCR() & ~(FPU_EXCEPTION_MASK));
    (void)__get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);

#if NRF_SDH_ENABLED && RUN_BLE
    sd_app_evt_wait();
#else
    // Use directly __WFE and __SEV macros since the SoftDevice is not available.
    // Wait for event.
    __WFE();
    // Clear Event Register.
    __SEV();
    __WFE();
#endif // 		#if NRF_SDH_ENABLED && RUN_BLE

}

int main(void)
{
    uptimeCounter_zero();
    // Get logging up
    log_init();
    NRF_LOG_DEBUG("%s start, compiled for %s, radio %s", DEVICE_NAME, BOARD_NAME, RADIO_NAME);

//    NRF_LOG_DEBUG("DEVid 0x%x %x, addr 0x%x %x", NRF_FICR->DEVICEID[1],
//                  NRF_FICR->DEVICEID[0],
//                  NRF_FICR->DEVICEADDR[1], NRF_FICR->DEVICEADDR[0]);

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
	    #if COMPILE_4DIGIT7SEG
        _4digit7seg_writeStr(strBuf);
		#endif // #if COMPILE_4DIGIT7SEG
    }
    // TODO start BLE for dropping to DFU, softDevice calls
#if NRF_SDH_ENABLED && RUN_BLE
    bleStuff_init();
    bleStuff_printBLEVersion();
#else
#include "nrf_clock.h"
//    nrf_clock_lfclk_request(); // to keep timer running without softdevice
//    nrf_clock_lfclk_start();
    NRF_LOG_INFO("LFCLK is %s", nrf_clock_lf_is_running() ? "Running" : "off");
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
		power_manage();
#endif // #if NRF_SDH_ENABLED && RUN_BLE

    }
}
