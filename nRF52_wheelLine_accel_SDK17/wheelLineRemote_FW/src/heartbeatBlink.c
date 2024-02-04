/*
 * heartbeatBlink.c
 *
 *  Created on: Jan 19, 2024
 *      Author: Collin Moore
 */

#include "heartbeatBlink.h"

#include "nrf_gpio.h"
#include "nrf_log.h"
#include "pollers.h"
#include <stdint.h>
#include "uptimeCounter.h"

#include "_4digit7seg.h"

#include "version.h"

/*************************************************************************************
 *  Definitions
 ************************************************************************************/


#define  HEARTBEAT_LED_TOGGLE_MS 1000
/*************************************************************************************
 *  Variables
 ************************************************************************************/
static uint32_t m_lastToggle_ms;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static void heartBlinkPoll(void)
{
    if (uptimeCounter_elapsedSince(m_lastToggle_ms) >= HEARTBEAT_LED_TOGGLE_MS)
    {
        if (NRF_P0->OUT & (1UL << HEARTBEAT_LED_GPIO_NUM))
        {
            NRF_P0->OUTCLR = (1UL << HEARTBEAT_LED_GPIO_NUM);
        }
        else
        {
            NRF_P0->OUTSET = (1UL << HEARTBEAT_LED_GPIO_NUM);
        }
        m_lastToggle_ms = uptimeCounter_getUptimeMs();
//        NRF_LOG_DEBUG("Toggled heartbeat LED at %d ms", m_lastToggle_ms);
    }

}

void heartblink_init()
{
    m_lastToggle_ms = 0;
    // LEDs are active low
    NRF_P0->OUTSET = (1UL << HEARTBEAT_LED_GPIO_NUM);
    nrf_gpio_cfg_output(HEARTBEAT_LED_GPIO_NUM);
    pollers_registerPoller(heartBlinkPoll);
}
