/*
 * gpioDriver.c
 *
 *  Created on: Mar 8, 2024
 *      Author: Collin
 */

#include "gpioDriver.h"

#include "globalInts.h"
#include "nrf_gpio.h"
#include "pollers.h"
#include "uptimeCounter.h"

#include "version.h"

#define NRF_LOG_MODULE_NAME gpioDriver
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

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

static bool isPressed(uint32_t pinNo)
{
    if ((NRF_P0->IN >> pinNo) & 0x01U)
    { // High=false, pulled up
        return false;
    }
    return true;
}

static void gpioDriverPoll(void)
{
    // Get button states
    machineState_t currentState = globalInts_getMachineState();
    switch (currentState)
    {
        case machState_justPoweredOn:
            case machState_killEngine:
            if (isPressed(BUTTON_START_PIN))
            {
                NRF_LOG_INFO("Moving to START state");
                globalInts_setMachineState(machState_startEngine);
            }
        break;
        case machState_startEngine:
            if (!isPressed(BUTTON_START_PIN))
            {
                NRF_LOG_INFO("Start released, releasing starter, enging HYD idle");
                globalInts_setMachineState(machState_runEngineHydIdle);
            }
        break;
        case machState_runEngineHydIdle:
            if (isPressed(BUTTON_FWD_PIN))
            {
                NRF_LOG_INFO("FWD pressed, engine HYD FWD");
                globalInts_setMachineState(machState_runEngineHydFwd);
            }
            if (isPressed(BUTTON_REV_PIN))
            {
                NRF_LOG_INFO("REV pressed, engine HYD REV");
                globalInts_setMachineState(machState_runEngineHydRev);
            }
        break;
        case machState_runEngineHydFwd:
            if (!isPressed(BUTTON_FWD_PIN))
            {
                NRF_LOG_INFO("FWD released, engine HYD idle");
                globalInts_setMachineState(machState_runEngineHydIdle);
            }
        break;
        case machState_runEngineHydRev:
            if (!isPressed(BUTTON_REV_PIN))
            {
                NRF_LOG_INFO("REV released, engine HYD idle");
                globalInts_setMachineState(machState_runEngineHydIdle);
            }
        break;
        default:
            break;
    }

}

void gpioDriver_init(void)
{
    // Set up GPIOs
    nrf_gpio_cfg(BUTTON_START_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_S0D1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(BUTTON_FWD_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_S0D1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(BUTTON_REV_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_S0D1,
                 NRF_GPIO_PIN_NOSENSE);
    // Set initial machine state
    globalInts_setMachineState(machState_justPoweredOn);
    pollers_registerPoller(gpioDriverPoll);
}
