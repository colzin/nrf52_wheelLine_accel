/*
 * relayGpios.c
 *
 *  Created on: Mar 9, 2024
 *      Author: Collin
 */

#include "relayGpios.h"

#include "cc1101.h"
#include "globalInts.h"
#include "nrf_gpio.h"
#include "pollers.h"

#include "version.h" // For pin info

#define NRF_LOG_MODULE_NAME relayGpios
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/*************************************************************************************
 *  Variables
 ************************************************************************************/
static machineState_t m_lastState;
/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static void setGpios(machineState_t state)
{
    switch (state)
    {
        case machState_justPoweredOn:
            case machState_killEngine:
            default:
            NRF_LOG_DEBUG("State %d, killing engine", state)
            ;
            NRF_P0->OUTCLR = (1U << KILLOPEN_RELAY_PIN)
                    | (1U << START_RELAY_PIN)
                    | (1U << FWD_RELAY_PIN)
                    | (1U << REV_RELAY_PIN);
        break;
        case machState_startEngine:
            NRF_P0->OUTCLR = (1U << FWD_RELAY_PIN)
                    | (1U << REV_RELAY_PIN);
            NRF_P0->OUTSET = (1U << KILLOPEN_RELAY_PIN)
                    | (1U << START_RELAY_PIN);
        break;
        case machState_runEngineHydFwd:
            NRF_P0->OUTCLR = (1U << START_RELAY_PIN)
                    | (1U << REV_RELAY_PIN);
            NRF_P0->OUTSET = (1U << KILLOPEN_RELAY_PIN)
                    | (1U << FWD_RELAY_PIN);
        break;
        case machState_runEngineHydIdle:
            NRF_P0->OUTCLR = (1U << START_RELAY_PIN)
                    | (1U << FWD_RELAY_PIN)
                    | (1U << REV_RELAY_PIN);
            NRF_P0->OUTSET = (1U << KILLOPEN_RELAY_PIN);
        break;
        case machState_runEngineHydRev:
            NRF_P0->OUTCLR = (1U << START_RELAY_PIN)
                    | (1U << FWD_RELAY_PIN);
            NRF_P0->OUTSET = (1U << KILLOPEN_RELAY_PIN)
                    | (1U << REV_RELAY_PIN);
        break;
    }
}
static void relayGpiosPoll(void)
{
    machineState_t currentState = globalInts_getMachineState();
    if (currentState != m_lastState)
    {
        setGpios(currentState);
        m_lastState = currentState;
    }

}

void relayGpios_init(void)
{
    // Init GPIOs off, LOW (TODO define states)
    NRF_P0->OUTCLR = (1U << START_RELAY_PIN)
            | (1U << REV_RELAY_PIN)
            | (1U << KILLOPEN_RELAY_PIN)
            | (1U << FWD_RELAY_PIN);
    nrf_gpio_cfg(START_RELAY_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(KILLOPEN_RELAY_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(FWD_RELAY_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(REV_RELAY_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
    pollers_registerPoller(relayGpiosPoll);
}
