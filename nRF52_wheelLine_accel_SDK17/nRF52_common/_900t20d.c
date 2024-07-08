/*
 * _900t20d.c
 *
 *  Created on: July 5, 2024
 *      Author: Collin Moore
 */

#include "_900t20d.h"

#if COMPILE_RADIO_900T20D
#include "_4digit7seg.h"

#include "globalInts.h" // To set machine state

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "pollers.h"

#include "uarte0.h" // Comms to the module

#include "uptimeCounter.h"
#include "version.h"

#define NRF_LOG_MODULE_NAME _900t20d
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

typedef enum
{
    _900t20dMode_normal, // Normal RX and TX
    _900t20dMode_wakeUp, // Sends preamble to wake up receiver if receiver is in mode 2
    _900t20dMode_powerSave, // UART shut down. Monitors for preamble for RF RX.
    _900t20dMode_sleepConfig, // UART in 9600, 8n1 mode to set parameters.
    _900t20dMode_unknown,
} _900t20dMode_t;

#define STATUS_POLL_ITVL_MS 1200

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static _900t20dMode_t m_lastMode;

#if TX_TEST_ITVL_MS
static uint32_t m_lastTx_ms;
#endif // #if TX_TEST_ITVL_MS

static uint32_t m_inState_ms;
static uint32_t m_lastPoll_ms;

// For receiving packets
static uint8_t m_rxPacketBuffer[PKT_LEN];

static uint8_t m_sendState;

static bool m_newTxPower;
static int8_t m_desiredTxdBm;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static bool awaitAuxHigh(uint32_t maxWait_ms, uint32_t delayAfter_ms)
{
    uint32_t elapsed_ms = 0;
    if (NRF_P0->IN & (1U << _900T20D_AUX_PIN))
    {
        NRF_LOG_DEBUG("AUX high, skip delay.");
        return true;
    }
    // Wait for AUX to go high, if it isn't already
    while ((elapsed_ms < maxWait_ms) && (!(NRF_P0->IN & (1U << _900T20D_AUX_PIN))))
    {
        nrf_delay_ms(1);
        if (elapsed_ms < maxWait_ms)
        {
            elapsed_ms++;
        }
//        NRF_LOG_DEBUG("Waited with AUX low for %d ms", elapsed_ms);
    }
    if (elapsed_ms < maxWait_ms)
    {
        NRF_LOG_INFO("Detected AUX high after %d of %d ms. Reads %s now", elapsed_ms, maxWait_ms,
                     NRF_P0->IN & (1U << _900T20D_AUX_PIN)?"high":"low");
        if (delayAfter_ms)
        {
            nrf_delay_ms(delayAfter_ms);
            NRF_LOG_DEBUG("Delayed for %d ms after aux rise", delayAfter_ms);
        }
        return true;
    }
    else
    {
        NRF_LOG_ERROR("MaxWait %d ran out, AUX pin still reads %d!", maxWait_ms,
                      NRF_P0->IN & (1U << _900T20D_AUX_PIN));
        return false;
    }
}

static _900t20dMode_t getMode(void)
{
    NRF_LOG_DEBUG("getMode await aux high:");
    if (!awaitAuxHigh(100, 0))
    {
        NRF_LOG_ERROR("Failed to get AUX high to read state");
    }
    _900t20dMode_t state = 0;
    uint32_t mask = NRF_P0->IN;
    if (mask & (1U << _900T20D_M0_PIN))
    { // set bit 0
        state |= 0b1;
    }
    if (mask & (1U << _900T20D_M1_PIN))
    { // set bit 1
        state |= 0b10;
    }
    return state;
}

static void setMode(_900t20dMode_t state)
{
    NRF_LOG_DEBUG("setMode call getMode:");
    _900t20dMode_t currentMode = getMode();
    if (currentMode == state)
    { // Already in desired state, wait for AUX high to resume
        return;
    }
    // User manual recommends wait for 2ms after verifying that AUX is high to switch mode.
    NRF_LOG_DEBUG("setMode from mode %d to %d, await aux high:", currentMode, state);
    if (!awaitAuxHigh(100, 2))
    {
        NRF_LOG_ERROR("Failed to get AUX high before changing state");
    }
    switch (state)
    {
        case _900t20dMode_normal:
            NRF_P0->OUTCLR = (1U << _900T20D_M1_PIN) | (1U << _900T20D_M0_PIN);
        break;
        case _900t20dMode_wakeUp:
            NRF_P0->OUTCLR = 1U << _900T20D_M1_PIN;
            NRF_P0->OUTSET = 1U << _900T20D_M0_PIN;
        break;
        case _900t20dMode_powerSave:
            NRF_P0->OUTSET = 1U << _900T20D_M1_PIN;
            NRF_P0->OUTCLR = 1U << _900T20D_M0_PIN;
        break;
        case _900t20dMode_sleepConfig:
            NRF_P0->OUTSET = (1U << _900T20D_M1_PIN) | (1U << _900T20D_M0_PIN);
        break;
        default:
            NRF_LOG_ERROR("Can't set unknown mode %d", state)
            ;
            return;
    }
    m_lastMode = state;
    NRF_LOG_DEBUG("setMode await aux high after switch:");
    if (!awaitAuxHigh(25, 1))
    {
        NRF_LOG_ERROR("Failed to get AUX high after changing state");
    }
}

static bool sendBytes(uint8_t* pBytes, uint32_t len)
{
    if (NRF_SUCCESS != uarte0_enqueue(pBytes, len))
    {
        NRF_LOG_ERROR("sendBytes UART enqueue error");
        return false;
    }
    // Wait for UART to complete sending
    uint32_t elapsed_ms = 0;
    uint32_t maxWait_ms = len * 2 + 3; // Wait for 9600baud
    while (!uarte0_isTxDone() && elapsed_ms < maxWait_ms)
    {
        nrf_delay_ms(1);
        elapsed_ms++;
    }
    if (elapsed_ms < maxWait_ms)
    {
        NRF_LOG_DEBUG("Sent %d bytes in %d ms.", len, elapsed_ms);
        return true;

//        NRF_LOG_DEBUG("Sent %d bytes in %d ms. Now await AUX high:", len, elapsed_ms);
//        return awaitAuxHigh(500, 0);
    }
    else
    {
        NRF_LOG_ERROR("UART send waited for %d ms, but still not done", maxWait_ms);
        return false;
    }
}

static uint32_t readBytes(uint8_t* pBytes, uint32_t len, uint32_t maxWait_ms)
{
    uint32_t numRead = 0;
    uint32_t elapsed_ms = 0;
    while (numRead < len && elapsed_ms < maxWait_ms)
    {
        if (uarte0_tryReadByte(&pBytes[numRead]))
        {
            numRead++;
        }
        else
        { // Not successful in receiving a byte, wait a bit
            nrf_delay_ms(1);
            elapsed_ms++;
        }
    }
    if (elapsed_ms < maxWait_ms)
    {
//        NRF_LOG_DEBUG("Received %d bytes in %d ms", len, elapsed_ms);
    }
    else
    {
        NRF_LOG_ERROR("readBytes waited for %d ms, but only received %d bytes", maxWait_ms, numRead);
    }
    return numRead;
}

#if READ_ALL_REGS
static void readAllRegs(void)
{
    nrf_delay_ms(1);
    uint8_t buf[0x3D];
    _900t20d_readBurst(0x0, buf, sizeof(buf));
    for (uint32_t i = 0; i < sizeof(buf); i += 16)
    {
        NRF_LOG_HEXDUMP_DEBUG(&buf[i], 16);
        nrf_delay_ms(50);
    }
}
#endif // #if READ_ALL_REGS

//static bool _900t20d_hardReset(void)
//{
//// drive VCC OFF to start reset
//    NRF_P0->OUTCLR = 1U << _900T20D_VCC_CTRL_PIN;
//    nrf_delay_ms(100); // Wait for it to actually lose power
//    // turn VCC back ON, wait for AUX to go high
//    NRF_P0->OUTSET = 1U << _900T20D_VCC_CTRL_PIN;
//    bool auxAck = awaitAuxHigh(1200, 2);
//    if (!auxAck)
//    {
//        NRF_LOG_ERROR("AUX didn't go high after reset");
//    }
//    return true;
//}

static bool _900t20d_softReset(void)
{
    NRF_LOG_DEBUG("softReset, set config:");
    setMode(_900t20dMode_sleepConfig); // put in sleep mode to program it
    uint8_t bytes[3] = { 0xC4, 0xC4, 0xC4 };
    NRF_LOG_DEBUG("softReset sending SRES:");
    if (!sendBytes(bytes, sizeof(bytes)))
    {
        NRF_LOG_ERROR("Error sending SRES command");
    }
    uint32_t elapsedHigh_ms = 0;
    uint32_t maxTilReset_ms = 2000; // Can take up to 1.01 sec to reset.
    NRF_LOG_DEBUG("softReset await AUX low for %d ms (start of its reset):", maxTilReset_ms);
    while ((NRF_P0->IN & (1U << _900T20D_AUX_PIN)) && elapsedHigh_ms < maxTilReset_ms)
    {
        nrf_delay_ms(1);
        elapsedHigh_ms++;
    }
    if (elapsedHigh_ms > maxTilReset_ms)
    {
        NRF_LOG_ERROR("softReset never detected module driving AUX low, bail out");
        return false;
    }
    // Wait for AUX high again, usually about 180ms. Then wait for 3ms to be sure it's ready
    NRF_LOG_DEBUG("softReset detected AUX falling after %d ms for reset, await rise again:", elapsedHigh_ms);
    bool auxAck = awaitAuxHigh(1200, 3);
    if (!auxAck)
    {
        NRF_LOG_ERROR("AUX didn't go high after SRES ");
        return false;
    }
    NRF_LOG_DEBUG("softReset done.");
    return true;
}

static bool _900t20d_readOperatingParams(void)
{
    NRF_LOG_DEBUG("readOpParams start:");
    setMode(_900t20dMode_sleepConfig); // put in sleep mode to program it
    uint8_t txBytes[3] = { 0xC1, 0xC1, 0xC1 };
    if (!sendBytes(txBytes, sizeof(txBytes)))
    {
        NRF_LOG_ERROR("Error sending readOpParams command");
        return false;
    }
    // read back from UART: 6 bytes should come back.
    uint8_t rxBytes[6];
    uint32_t numBytesRead = readBytes(rxBytes, sizeof(rxBytes), 100);
    if (numBytesRead)
    {
        NRF_LOG_DEBUG("readOpParams got %d bytes: ", numBytesRead);
        NRF_LOG_HEXDUMP_DEBUG(rxBytes, numBytesRead);
    }
    if (6 != numBytesRead)
    {
        NRF_LOG_ERROR("readOpParams expected %d bytes, only got %d", 6, numBytesRead);
        return false;
    }
    // TODO store operating parameters that we found
    NRF_LOG_DEBUG("readOpParams done");
    return true;
}

static bool _900t20d_readVersionNumber(void)
{
    setMode(_900t20dMode_sleepConfig); // put in sleep mode to program it
    uint8_t txBytes[3] = { 0xC3, 0xC3, 0xC3 };
    if (!sendBytes(txBytes, sizeof(txBytes)))
    {
        NRF_LOG_ERROR("Error sending readVerNo command");
        return false;
    }
    // read back from UART: 8 bytes should come back.
    uint8_t rxBytes[8];
    uint32_t numBytesRead = readBytes(rxBytes, sizeof(rxBytes), 100);
    if (numBytesRead)
    {
        NRF_LOG_DEBUG("Read back %d bytes: ", numBytesRead);
        NRF_LOG_HEXDUMP_DEBUG(rxBytes, numBytesRead);
    }
    if (8 != numBytesRead)
    {
        NRF_LOG_ERROR("readVerNo expected %d bytes, only got %d", 8, numBytesRead);
        return false;
    }
    // TODO parse version number
    NRF_LOG_DEBUG("readVerNo success");
    return true;
}

static void packetTxPoll(_900t20dMode_t currentMode)
{
    switch (currentMode)
    {
        case _900t20dMode_normal:
            // Gets into this state from someone telling it to send a packet.
            if (m_inState_ms >= TX_TIMEOUT_MS)
            { // Shouldn't take a long time to send, TODO find time
                NRF_LOG_ERROR("Timed out in TX state, idling")
                ;
//                _900t20d_setIdle(true);
            }
        break;
        case _900t20dMode_wakeUp:
            if ((m_sendState && (RX_AFTER_TX_TIMEOUT_MS < m_inState_ms))
                    || (0 == m_sendState))
            {
                NRF_LOG_WARNING("packetTx RX state timeout after %d ms, sendState %d, idling", m_inState_ms,
                                m_sendState)
                ;
#ifdef UART_TX_PIN
                char strBuf[96];
                int strLen = snprintf(strBuf, sizeof(strBuf),
                                      "packetTx RX state timeout after %d ms, sendState %d, idling",
                                      m_inState_ms,
                                      m_sendState);
                if (0 < strLen)
                {
                    uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
                }
#endif // #ifdef UART_TX_PIN
//                _900t20d_setIdle(true);
                m_sendState = 0;
            }
        break;
        case _900t20dMode_powerSave:
            // Idle until someone calls sendPacket
            if (m_newTxPower)
            {
//                setTxPower(m_desiredTxdBm);
                m_newTxPower = false;
            }
            if (0 == m_inState_ms)
            { // Reset RX parser on entry, then try and receive packet
//                m_rxSettings.rxState = packetRxState_awaitingPacketStart;
//                m_rxSettings.packetBufIndex = 0;
//                // Usually here because we received a packet
//                tryReceivePacket(&m_rxSettings);
            }
            // Stay in idle until next TX command triggers TX, then TX to RX, then idle again.
        break;
        case _900t20dMode_sleepConfig:
            NRF_LOG_ERROR("Detected sleep state")
            ;
//            _900t20d_setIdle(true);
        break;
        default:
            if (m_inState_ms > 1000)
            {
                NRF_LOG_WARNING("State 0x%x, idling", currentMode);
//                _900t20d_setIdle(true);
            }
        break;
    }
}

#if READ_ALL_REGS
static uint32_t m_lastPrint_ms;
#endif // #if READ_ALL_REGS

static void _900t20dPoll(void)
{
    if (uptimeCounter_elapsedSince(m_lastPoll_ms) < STATUS_POLL_ITVL_MS)
    { // Don't spam SPI bus, especially when RXing
        return;
    }
// If here, we have expired the timer and can ask it what's up.
//    _900t20d_strobe(STROBE_NOP); // Strobe NOP to get status
// See if state has changed
    _900t20dMode_t currentMode = getMode();
    if (currentMode != m_lastMode)
    {
        m_inState_ms = 0;
    }
    else
    { // If in state, increment timer
        m_inState_ms += uptimeCounter_elapsedSince(m_lastPoll_ms);
    }
    switch (currentMode)
    {
        case _900t20dMode_normal:
            //            asyncTxPoll(currentMode);
        break;
        case _900t20dMode_wakeUp:
            //            packetRxPoll(currentMode);
        break;
        case _900t20dMode_powerSave:
            packetTxPoll(currentMode);
        break;
        default:
            break;
    }

#if TX_TEST_ITVL_MS
if (uptimeCounter_elapsedSince(m_lastTx_ms) >= TX_TEST_ITVL_MS)
{
    // TODO run a TX packet
    uint8_t txPacketBytes[PKT_LEN];
    for (uint8_t i = 0; i < PKT_LEN; i++)
    {
        txPacketBytes[i] = i;
    }
    _900t20d_sendPacket(txPacketBytes, PKT_LEN);
    m_lastTx_ms = uptimeCounter_getUptimeMs();
}
#endif // #if TX_TEST_ITVL_MS

#if READ_ALL_REGS
    if (uptimeCounter_elapsedSince(m_lastPrint_ms) > 2000)
    {
        readAllRegs();
        m_lastPrint_ms = uptimeCounter_getUptimeMs();
    }
#endif // #if READ_ALL_REGS
    m_lastMode = currentMode;
    m_lastPoll_ms = uptimeCounter_getUptimeMs();

}

bool _900t20d_sendPacket(uint8_t byte)
{
    uint8_t pktBuf[PKT_LEN];
    pktBuf[0] = (uint8_t)m_desiredTxdBm;
    pktBuf[1] = byte;
    bool ret = true;
//    ret &= _900t20d_writeBurst(TXFIFO_REGADDR, pktBuf, PKT_LEN);
//    ret &= _900t20d_readSingleByte(TXBYTES_REGADDR, pktBuf);
    if (PKT_LEN == pktBuf[0])
    {
//        NRF_LOG_DEBUG("Sending packet of %d bytes at %d dBm.", PKT_LEN, m_desiredTxdBm);
#if verbose_tx
        NRF_LOG_DEBUG("Sending packet of %d bytes at %d dBm.", PKT_LEN, m_desiredTxdBm);
#ifdef UART_TX_PIN
        uint8_t strBytes[256];
        int printlen = snprintf((char*)strBytes, sizeof(strBytes), "_900t20d sending %d bytes at %d dBm", PKT_LEN,
                                m_desiredTxdBm);
        if (0 < printlen)
        {
            uartTerminal_enqueueToUSB(strBytes, (uint32_t)printlen);
        }
#endif // #ifdef UART_TX_PIN
#endif // #if verbose_tx
    }
    else
    {
        NRF_LOG_WARNING("Sending packet of %d bytes at %d dBm to txfifo, BUT TXYBTES says %d", PKT_LEN,
                        m_desiredTxdBm,
                        pktBuf[0]);
    }
//    ret &= _900t20d_strobe(STROBE_STX);
    m_sendState++;
    return ret;

}

void _900t20d_setOutputPower(int8_t tx_dBm)
{
    if (m_desiredTxdBm != tx_dBm)
    {
        NRF_LOG_INFO("Desire change of power to %d dBm", tx_dBm);
#ifdef UART_TX_PIN
        char strBuf[96];
        int strLen = snprintf(strBuf, sizeof(strBuf), "Desire TX %d dBm", tx_dBm);
        if (0 < strLen)
        {
            uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
        }
#endif // #ifdef UART_TX_PIN
        m_desiredTxdBm = tx_dBm;
        m_newTxPower = true;
    }
}

void _900t20d_init(void)
{
    uarte0_init(); // Make sure UART is initted, ignore errors until we turn it back ON

// turn VCC to radio OFF, set state, then turn ON, then wait for AUX to be set high by radio
//    NRF_P0->OUTCLR = 1U << _900T20D_VCC_CTRL_PIN;
//    nrf_gpio_cfg(_900T20D_VCC_CTRL_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
//                 NRF_GPIO_PIN_INPUT_CONNECT,
//                 GPIO_PIN_CNF_PULL_Disabled,
//                 NRF_GPIO_PIN_S0S1,
//                 NRF_GPIO_PIN_NOSENSE);
    // AUX may be open-drain, pull it up.
    nrf_gpio_cfg(_900T20D_AUX_PIN, NRF_GPIO_PIN_DIR_INPUT,
                 NRF_GPIO_PIN_INPUT_CONNECT,
                 GPIO_PIN_CNF_PULL_Pullup,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_SENSE_HIGH);
// set up M0 and M1 states for radio mode
    nrf_gpio_cfg(_900T20D_M0_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_CONNECT,
                 GPIO_PIN_CNF_PULL_Disabled,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(_900T20D_M1_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_CONNECT,
                 GPIO_PIN_CNF_PULL_Disabled,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
    // Try a soft reset, see if it likes that.
    if (!_900t20d_softReset())
    {
        NRF_LOG_ERROR("Reset failed, bail out");
        return;
    }
// Ok, it has booted. Do any init over UART.
    if (!_900t20d_readOperatingParams())
    {
        NRF_LOG_ERROR("Error reading operating params, bail out");
    }
    if (!_900t20d_readVersionNumber())
    {
        NRF_LOG_ERROR("Error reading version number, bail out");
    }

//
//    m_lastChipStatusByte = 0x80; // Default to idle, no FIFO bytes
//#if READ_ALL_REGS
//    NRF_LOG_WARNING("Regs before reset:");
//    readAllRegs();
//#endif // #if READ_ALL_REGS
//    bool testPass = _900t20d_reset();
//#if READ_ALL_REGS
//    NRF_LOG_WARNING("Regs after reset:");
//    readAllRegs();
//#endif // #if READ_ALL_REGS
//    if (!testPass)
//    {
//        NRF_LOG_ERROR("Couldn't reboot _900t20d");
//        return;
//    }
//    testPass = _900t20d_selfTest();
//    if (!testPass)
//    {
//        NRF_LOG_ERROR("_900t20d self-test failed");
//        return;
//    }
//// If here, init it
////    _900t20d_initASKTx_myStudio();
//    m_opMode = _900t20dMode_unknown;
//
//    // TODO switch module OFF, then set up M0, M1, then turn ON.
//
//    switch (desired)
//    {
//        case _900t20d_packetRX:
//            setupGDO2Input(_900t20d_GDO2_PIN);
//            initPacketReceiver();
//            setupPacketRadio(-20); // Same settings for Rx side
//            m_opMode = _900t20d_packetRX;
//            m_sendState = 0;
//        break;
//        case _900t20d_packetTX:
//            setupGDO2Input(_900t20d_GDO2_PIN);
//            initPacketReceiver();
//            setupPacketRadio(-20); // Same settings for Rx side
//            m_opMode = _900t20d_packetTX;
//            m_sendState = 0;
//        break;
//        case _900t20d_asyncTX:
//            // do NOT set up the MOSI pin to drive yet, until we de-init the default _900t20d drive as output.
//            setupAsyncTx();
//            // Now set up our output pin, which should idle low to not send.
//            ev1527SPI_init(SPI2_MOSI_PIN); // init SPI, which idles low.
//            _900t20d_strobe(STROBE_STX); // Start TX of _900t20d, to send what comes out MOSI
//            m_opMode = _900t20d_asyncTX;
//            NRF_LOG_WARNING("Be sure to wire SPI2_MOSI pin %d to _900t20d GDO0", SPI2_MOSI_PIN)
//            ;
//        break;
//        default:
//            NRF_LOG_ERROR("Don't know how to set  up mode %d", desired)
//            ;
//        break;
//    }

    pollers_registerPoller(_900t20dPoll);
}

#endif // #if COMPILE_RADIO_900t20d
