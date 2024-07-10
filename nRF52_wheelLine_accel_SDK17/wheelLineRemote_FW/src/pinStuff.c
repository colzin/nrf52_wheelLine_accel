/*
 * pinStuff.c
 *
 *  Created on: Mar 8, 2024
 *      Author: Collin Moore
 */

#include "pinStuff.h"

#include "globalInts.h"
#include "nrf_gpio.h"
#include "pollers.h"
#include "uptimeCounter.h"

#include "version.h"

#define NRF_LOG_MODULE_NAME pinStuff
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define BUTTONS_DEBOUNCE_PRESS_MS 68 // 50 was very short, 500 too long.
#define BUTTONS_DEBOUNCE_RELEASE_MS BUTTONS_DEBOUNCE_PRESS_MS

#define BUTTON_HOLD_MIN_MS 900 // todo TUNE

typedef struct
{
    uint32_t inState_ms;
    uint8_t pinNumber;
    bool pressedThisPoll, pressedLastPoll;
} buttonInfo_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

static buttonInfo_t g_startButtonInfo, g_fwdButtonInfo, g_revButtonInfo;
static uint32_t g_lastButtonPoll_ms;

static bool m_fwdPressed, m_revPressed;
static uint32_t m_fwdPressStart_ms, m_revPressStart_ms;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static void pinStuff_initButtons(void)
{
    // Set up GPIOs
    // filter, function 0, enable input, pullup
//    pinMode(BUTTON_START_PIN, INPUT_PULLUP);
    nrf_gpio_cfg(BUTTON_START_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_S0D1,
                 NRF_GPIO_PIN_NOSENSE);

//    pinMode(BUTTON_FWD_PIN, INPUT_PULLUP);
    nrf_gpio_cfg(BUTTON_FWD_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_S0D1,
                 NRF_GPIO_PIN_NOSENSE);

//    pinMode(BUTTON_REV_PIN, INPUT_PULLUP);
    nrf_gpio_cfg(BUTTON_REV_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_S0D1,
                 NRF_GPIO_PIN_NOSENSE);
    // NRF_LOG_DEBUG("\nConfig for START: 0x%x, FWD: 0x%x, REV: 0x%x\n\n", *((uint32_t *)GPIO_PIN_REG_0), *((uint32_t *)GPIO_PIN_REG_33), *((uint32_t *)GPIO_PIN_REG_34));
    g_startButtonInfo.pinNumber = BUTTON_START_PIN;
    g_startButtonInfo.inState_ms = 0;
    g_startButtonInfo.pressedLastPoll = false;
    g_startButtonInfo.pressedThisPoll = false;
    g_fwdButtonInfo.pinNumber = BUTTON_FWD_PIN;
    g_fwdButtonInfo.inState_ms = 0;
    g_fwdButtonInfo.pressedLastPoll = false;
    g_fwdButtonInfo.pressedThisPoll = false;
    g_revButtonInfo.pinNumber = BUTTON_REV_PIN;
    g_revButtonInfo.inState_ms = 0;
    g_revButtonInfo.pressedLastPoll = false;
    g_revButtonInfo.pressedThisPoll = false;

    m_fwdPressed = false;
    m_revPressed = false;
}

static uint32_t digitalRead(uint8_t pinNumber)
{
    return (NRF_P0->IN >> pinNumber) & 0x01U;
}

static void updateButtonState(buttonInfo_t* p, uint32_t ms_now, uint32_t lastPoll_ms)
{
    // Move current to last
    p->pressedLastPoll = p->pressedThisPoll;
    // update current
    if (digitalRead(p->pinNumber))
    { // High, so not pressed (active low)
        p->pressedThisPoll = false;
    }
    else
    { // Active low
        p->pressedThisPoll = true;
    }
    if (p->pressedLastPoll == p->pressedThisPoll)
    {
        p->inState_ms += uptimeCounter_elapsedSince(lastPoll_ms);
    }
    else
    { // Zero the counter on state change
        NRF_LOG_DEBUG("Pin %d change to %d at %d\n", p->pinNumber, p->pressedThisPoll, ms_now);
        p->inState_ms = 0;
    }
}

static void checkToStart(void)
{
    if (g_startButtonInfo.pressedThisPoll && g_startButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_PRESS_MS)
    {
        NRF_LOG_INFO("Moving from %s to START state\n", globalInts_getMachStateString(globalInts_getMachineState()));
        globalInts_setMachineState(machState_startEngine);
    }
}

static void checkStartRelease(void)
{
    if (!g_startButtonInfo.pressedThisPoll && g_startButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_PRESS_MS)
    {
        NRF_LOG_INFO("Moving from %s to RunIdle state\n", globalInts_getMachStateString(globalInts_getMachineState()));
        globalInts_setMachineState(machState_runEngineHydIdle);
    }
}

static void checkFwdRelease(void)
{
    if (m_fwdPressed && !g_fwdButtonInfo.pressedThisPoll && (g_fwdButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_RELEASE_MS))
    {              // If released for debounce period, check time to see if we should increment count or release HYD now
        m_fwdPressed = false; // Mark released
        uint32_t pressed_ms = uptimeCounter_elapsedSince(m_fwdPressStart_ms);
        if (pressed_ms < BUTTON_HOLD_MIN_MS)
        { // On release of FWD button, increment the counter, we need to move in the negative direction
            NRF_LOG_INFO("FWD released after %d seconds, need %d to hold, increment rotations\n", pressed_ms,
                         BUTTON_HOLD_MIN_MS);
            globalInts_setNumRotations(globalInts_getNumRotations() + 1);
        }
        else
        {
            NRF_LOG_INFO("FWD released after %d, to HYD idle at %d\n", pressed_ms, uptimeCounter_getUptimeMs());
            globalInts_setNumRotations(0);
            globalInts_setMachineState(machState_runEngineHydIdle);
        }
    }
}

static void checkRevRelease(void)
{
    if (m_revPressed && !g_revButtonInfo.pressedThisPoll && (g_revButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_RELEASE_MS))
    {              // If released for debounce period, check time to see if we should increment count or release HYD now
        m_revPressed = false; // Mark released
        uint32_t pressed_ms = uptimeCounter_elapsedSince(m_revPressStart_ms);
        if (pressed_ms < BUTTON_HOLD_MIN_MS)
        { // On release of REV button, decrement the counter, we need to move in the positive direction
            NRF_LOG_INFO("REV released after %d seconds, need %d to hold, decrement rotations\n", pressed_ms,
                         BUTTON_HOLD_MIN_MS);
            globalInts_setNumRotations(globalInts_getNumRotations() - 1);
        }
        else
        {
            NRF_LOG_INFO("REV released after %d, to HYD idle at %d\n", pressed_ms, uptimeCounter_getUptimeMs());
            globalInts_setNumRotations(0);
            globalInts_setMachineState(machState_runEngineHydIdle);
        }
    }
}

static void checkFwdPress(void)
{
    // In this state, check for a debounced press of the FWD button
    if (!m_fwdPressed && g_fwdButtonInfo.pressedThisPoll && (g_fwdButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_PRESS_MS))
    {
        NRF_LOG_INFO("FWD pressed for %d, from state %s to FWD state at %d\n", g_fwdButtonInfo.inState_ms,
                     globalInts_getMachStateString(globalInts_getMachineState()),
                     uptimeCounter_getUptimeMs());
        m_fwdPressed = true;
        m_fwdPressStart_ms = uptimeCounter_getUptimeMs();
        globalInts_setMachineState(machState_runEngineHydFwd);
    }
}

static void checkRevPress(void)
{
    if (!m_revPressed && g_revButtonInfo.pressedThisPoll && (g_revButtonInfo.inState_ms >= BUTTONS_DEBOUNCE_PRESS_MS))
    {
        NRF_LOG_INFO("REV pressed for %d, from state %s to REV state at %d\n", g_revButtonInfo.inState_ms,
                     globalInts_getMachStateString(globalInts_getMachineState()),
                     uptimeCounter_getUptimeMs());
        m_revPressed = true;
        m_revPressStart_ms = uptimeCounter_getUptimeMs();
        globalInts_setMachineState(machState_runEngineHydRev);
    }
}

static void pollButtons(void)
{
    // Update button states
    uint32_t ms_now = uptimeCounter_getUptimeMs();
    updateButtonState(&g_startButtonInfo, ms_now, g_lastButtonPoll_ms);
    updateButtonState(&g_fwdButtonInfo, ms_now, g_lastButtonPoll_ms);
    updateButtonState(&g_revButtonInfo, ms_now, g_lastButtonPoll_ms);

    // Change state based on button pressed
    machineState_t currentState = globalInts_getMachineState();
    switch (currentState)
    {
        case machState_justPoweredOn:
            case machState_killEngine:
            // In these states, check for a debounced press of the Start button
            checkToStart();
        break;
        case machState_startEngine:
            // Check for user release of Start button
            checkStartRelease();
        break;
        case machState_runEngineHydIdle:
            // In this state, check for a debounced press of the Start button
            checkToStart();
            checkFwdPress();
            checkRevPress();
        break;
        case machState_runEngineHydFwd:
            checkFwdRelease();
            checkToStart();
            checkFwdPress(); // check for another press
            checkRevPress();
        break;
        case machState_runEngineHydRev:
            checkRevRelease();
            checkToStart();
            checkFwdPress();
            checkRevPress(); // check for another press
        break;
        default:
            NRF_LOG_WARNING("UNKNOWN state %d, going to justPoweredOn\n", currentState)
            ;
            globalInts_setMachineState(machState_justPoweredOn);
        break;
    }

    g_lastButtonPoll_ms = ms_now;
}

static void pinStuffPoll(void)
{
    pollButtons();
//    adcPoll();
}

void pinStuff_init(void)
{

    // Init the buttons for the Remote
    pinStuff_initButtons();

    pollers_registerPoller(pinStuffPoll);
}
