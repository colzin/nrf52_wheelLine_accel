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

// 16 bits, MSByte is byte[1], LSByte is byte[2]
#define GET_ADDRESS(byte1,byte2) ((((uint16_t)byte1)<<8)|byte2)
// byte[3] (SPED) definitions:
// parity in b7:6
#define GET_UART_PARITY(byte3) (byte3>>6)
// baud rate in b5:3
#define GET_UART_BAUD(byte3) ((byte3>>3)&0x7)
// air data rate in b2:0
#define GET_AIR_DATA_RATE(byte3) (byte3&0x07)
// byte[4] (CHAN) definitions:
// CHAN in b4:0.
#define GET_CHAN(byte4) (byte4&0x1F)
// byte[5] (OPTION) definitions:
/* Fixed Transmission Enable bit is byte[5] b7
 * If SET: Fixed transmission mode, where the first three bytes of each user's data frame can be used
 * as high/low address and channel. The module changes its address and channel when it transmits, then reverts
 * to the normal setting when complete.
 * If CLEAR: Transparent transmission mode: passes the first three bytes through
 */
#define GET_FIXEDTRANSMODE(byte5)(byte5>>7)
// b6 is IO drive mode: 1 for push-pull AUX and TXD, 0 for open-collector outputs (need pullups)
#define GET_IODRIVEMODE(byte5)((byte5>>6)&0x01)//
/* b5:3 is Wireless wake-up time:
 *
 */
#define GET_WIRELESSWAKEUPTIME(byte5) ((byte5>>3)&0x07)
// b2 is FEC switch, 1 to enable forward Error Correction, 0 to disable
#define GET_FECENABLE(byte5)((byte5>>2)&0x01)
// b1:0 are transmit power. Lower power not recommended in manual
#define GET_TXPOWER(byte5)(byte5&0x03)
// Clear b1:0, then set value
#define SET_TXPOWER(byte5, x)((byte5&0xFC)|x)
typedef enum
{
    txPwr_20dBm = 0,
    txPwr_17dBm,
    txPwr_14dBm,
    txPwr_10dBm
} _900t20dTxPwr_t;

//typedef struct
//{
//    uint16_t address; // 16 bits, MSByte in byte[1], LSByte byte[2]
//    uint8_t uartParity :2; // b7:6 of byte[3]
//    uint8_t uartBaud :3; // b5:3 of byte[3]
//    uint8_t airDataRate :3; // b2:0 of byte[3]
//    // b7:5 of byte[4] are reserved, write zeros always
//    uint8_t channel :5; // b4:0 channel, (862MHz + channel*1MHz). Default 0x06. So values 0x00 to 0x45 are valid.
//
//    uint8_t fixedTransEnable :1;
//    uint8_t fixedTransEnable :1;
//
//} _900t20dConfig_t;

#define STATUS_POLL_ITVL_MS 1200

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static _900t20dMode_t m_lastMode;
static bool m_configBytesValid = false;
static uint8_t m_configBytes[6];

#if TX_TEST_ITVL_MS
static uint32_t m_lastTx_ms;
#endif // #if TX_TEST_ITVL_MS

static uint32_t m_inState_ms;
static uint32_t m_lastPoll_ms;

#if USE_PACKETS
// For receiving packets
static uint8_t m_rxPacketBuffer[128]; // TODO make the max packet size possible
static uint32_t m_rxWriteIndex;
#else

#endif // #if USE_PACKETS

static bool m_newTxPower;
static _900t20dTxPwr_t m_desiredTxSetting;

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
//        NRF_LOG_DEBUG("AUX high, skip delay.");
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
//            NRF_LOG_DEBUG("Delayed for %d ms after aux rise", delayAfter_ms);
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
//    NRF_LOG_DEBUG("getMode await aux high:");
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

static bool setMode(_900t20dMode_t state)
{
//    NRF_LOG_DEBUG("setMode call getMode:");
    _900t20dMode_t currentMode = getMode();
    if (currentMode == state)
    { // Already in desired state, wait for AUX high to resume
        return true;
    }
    // User manual recommends wait for 2ms after verifying that AUX is high to switch mode.
//    NRF_LOG_DEBUG("setMode from mode %d to %d, await aux high:", currentMode, state);
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
            return false;
    }
    m_lastMode = state;
    NRF_LOG_DEBUG("setMode await aux high after switch:");
    if (!awaitAuxHigh(25, 1))
    {
        NRF_LOG_ERROR("Failed to get AUX high after changing state");
        return false;
    }
    return true;
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
//        NRF_LOG_DEBUG("Sent %d bytes in %d ms.", len, elapsed_ms);
//        return true;

        NRF_LOG_DEBUG("Sent %d bytes in %d ms. Now await AUX high:", len, elapsed_ms);
        return awaitAuxHigh(500, 0);
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
    while (numRead < len)
    {
        if (uarte0_tryReadByte(&pBytes[numRead]))
        {
            numRead++;
        }
        else if (maxWait_ms)
        { // Not successful in receiving a byte, wait a bit
            nrf_delay_ms(1);
            elapsed_ms++;
            if (elapsed_ms >= maxWait_ms)
            { // Leave this while loop
                break;
            }
        }
    }
    if (elapsed_ms < maxWait_ms)
    {
//        NRF_LOG_DEBUG("Received %d bytes in %d ms", len, elapsed_ms);
    }
    else if (maxWait_ms)
    { // Print warning if we were waiting
        NRF_LOG_WARNING("readBytes waited for %d ms, but only received %d bytes", maxWait_ms, numRead);
    }
    return numRead;
}

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

static bool readConfigBytes(void)
{
    m_configBytesValid = false; // Mark old as invalid if we are trying to read
    NRF_LOG_DEBUG("readOpParams start:");
    setMode(_900t20dMode_sleepConfig); // put in sleep mode to program it
    uint8_t txBytes[3] = { 0xC1, 0xC1, 0xC1 };
    if (!sendBytes(txBytes, sizeof(txBytes)))
    {
        NRF_LOG_ERROR("Error sending readOpParams command");
        return false;
    }
    m_configBytesValid = false;
    // read back from UART: 6 bytes should come back.
    uint32_t numBytesRead = readBytes(m_configBytes, sizeof(m_configBytes), 100);
    if (numBytesRead)
    {
        NRF_LOG_DEBUG("readOpParams got %d bytes: ", numBytesRead);
        NRF_LOG_HEXDUMP_DEBUG(m_configBytes, numBytesRead);
    }
    if (6 != numBytesRead)
    {
        NRF_LOG_ERROR("readOpParams expected %d bytes, only got %d", 6, numBytesRead);
        return false;
    }
    // byte[0] should be 0xC0 or 0xC2: C0 to save params, C2 to not save
    if (0xC0 != m_configBytes[0] && 0xC2 != m_configBytes[0])
    {
        NRF_LOG_ERROR("byte[0] was Not C0 or C2, bail out");
        return false;
    }
    NRF_LOG_DEBUG("Address is 0x%04x", GET_ADDRESS(m_configBytes[1], m_configBytes[2]));
    NRF_LOG_DEBUG("Uart parity is 0x%x, baud rate 0x%x, air data rate 0x%x", GET_UART_PARITY(m_configBytes[3]),
                  GET_UART_BAUD(m_configBytes[3]),
                  GET_AIR_DATA_RATE(m_configBytes[3]));

    NRF_LOG_DEBUG("Channel byte 0x%x, value 0x%x", m_configBytes[4], GET_CHAN(m_configBytes[4]));

    NRF_LOG_DEBUG("FixedTrans: %s, IO mode: %s", GET_FIXEDTRANSMODE(m_configBytes[5])?"Fixed":"Transparent",
                  GET_IODRIVEMODE(m_configBytes[5])?"PushPull":"OpenColl");
    NRF_LOG_DEBUG("Wireless wakeup time 0x%x, FEC: %s, TxPower: 0x%x", GET_WIRELESSWAKEUPTIME(m_configBytes[5]),
                  GET_FECENABLE(m_configBytes[5])?"enabled":"disabled",
                  GET_TXPOWER(m_configBytes[5]));
//    NRF_LOG_DEBUG("readOpParams done");
    m_configBytesValid = true;
    return true;
}

static bool _900t20d_readVersionInfo(void)
{
    setMode(_900t20dMode_sleepConfig); // put in sleep mode to program it
    uint8_t txBytes[3] =
            { 0xC3, 0xC3, 0xC3 };
    if (!sendBytes(txBytes, sizeof(txBytes)))
    {
        NRF_LOG_ERROR("Error sending readVerNo command");
        return false;
    }
    // read back from UART: 8 bytes should come back, or maybe 4
    uint8_t rxBytes[8];
    uint32_t numBytesRead = readBytes(rxBytes, sizeof(rxBytes), 50);
    if (0 == numBytesRead)
    {
        NRF_LOG_ERROR("readVer didn't get any reply");
        return false;
    }
    else
    { // Check leading byte
        if (txBytes[0] != rxBytes[0])
        {
            NRF_LOG_ERROR("readVer sent 0x%x, but got back 0x%x, can't use.", txBytes[0], rxBytes[0]);
            return false;
        }
    }
    // I thought we'd get 8 bytes, but we may get 4 bytes
    if (8 == numBytesRead)
    {
        NRF_LOG_WARNING("readVerNo got the expected 8 bytes. TODO parse!");
        NRF_LOG_HEXDUMP_WARNING(rxBytes, numBytesRead);
        return true;
    }
    else if (4 == numBytesRead)
    {
        // Seems like it's Header, (already checked), Freq, Version, Features
        NRF_LOG_INFO("Read 3-byte version Freq 0x%x, Version 0x%x, Features 0x%x", rxBytes[1], rxBytes[2], rxBytes[3]);
        NRF_LOG_INFO("Read 3-byte version Freq %d, Version %d, Features %d", rxBytes[1], rxBytes[2], rxBytes[3]);
        return true;
    }
    // If here, we didn't know how to parse it. Error
    NRF_LOG_WARNING("readVerNo got %d bytes. TODO parse!", numBytesRead);
    NRF_LOG_HEXDUMP_WARNING(rxBytes, numBytesRead);
    return false;
}

static bool writeConfigBytes(uint8_t* pConfigBytes, bool verify)
{
    _900t20dMode_t lastMode = getMode(); // Save and restore mode
    setMode(_900t20dMode_sleepConfig); // Set to config mode
    // Now write the params, then read back to verify that they are as desired
    // Manual says to write 6 bytes, with 0xC0 leading to save params
    pConfigBytes[0] = 0xC0; // Save parameters when powering down
    bool ret = true;
    ret &= sendBytes(pConfigBytes, 6);
    // Now set back to the old mode
    if (verify)
    {
        if (!readConfigBytes())
        {
            NRF_LOG_ERROR("Readback failed");
            ret = false;
        }
        if (memcmp(m_configBytes, pConfigBytes, 6))
        {
            NRF_LOG_ERROR("Verify failed");
            ret = false;
        }
    }

    ret &= setMode(lastMode);
    return ret;
}

static bool trySetTxPower(_900t20dTxPwr_t desiredTxPwr)
{ // E32 900T20D can go up to 20dBm
    if (!m_configBytesValid)
    {
        readConfigBytes();
    }
    if (!m_configBytesValid)
    { // Failed to read config, don't set.
        return false;
    }
    uint8_t newParams[6];
    // copy current setttings for new parameters
    memcpy(newParams, m_configBytes, 6);
    newParams[5] = (uint8_t)SET_TXPOWER(newParams[5], m_desiredTxSetting);

    if (writeConfigBytes(newParams, true))
    {
        m_newTxPower = false;
        return true;
    }
    else
    {
        NRF_LOG_ERROR("setTxPower call to writeParams failed, retry");
    }
    return false;
}

static void packetRxPoll(void)
{
    uint8_t rxByte;
    uint32_t numRead = readBytes(&rxByte, 1, 0);
    while (numRead)
    {
        // Received a byte from the other module, enqueue it to the RX packet
#if USE_PACKETS
        m_rxPacketBuffer[m_rxWriteIndex++] = rxByte;
        loraStuff_tryParsePacket(m_rxPacketBuffer, m_rxWriteIndex);
#else
        globalInts_setMachineState(rxByte);
#endif // #if USE_PACKETS
        // See if there are any more in UART rxBuf
        numRead = readBytes(&rxByte, 1, 0);
    }
}

static void _900t20dPoll(void)
{
    if (uptimeCounter_elapsedSince(m_lastPoll_ms) < STATUS_POLL_ITVL_MS)
    { // Don't spam module, especially when RXing
        return;
    }
    if (m_newTxPower)
    {
        trySetTxPower(m_desiredTxSetting);
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
            case _900t20dMode_wakeUp:
            case _900t20dMode_powerSave:
            packetRxPoll();
        break;
        default:
            // Nothing in programming mode
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

    m_lastMode = currentMode;
    m_lastPoll_ms = uptimeCounter_getUptimeMs();
}

#if USE_PACKETS
#error "define send function"
#else
bool _900t20d_sendByte(uint8_t byte)
{
    setMode(_900t20dMode_wakeUp); // Set to normal UART mode, send preamble to receiver to wake it up
    // TODO should we just use normal mode for speed?

    if (GET_FIXEDTRANSMODE(m_configBytes[5]))
    { // Need 3 more bytes: addHigh, addLow, and channel
//        uint8_t pkt[4];
//        pkt[0] = m_destAddr >> 8;
//        pkt[1] = m_destAddr & 0xFF;
//        pkt[2] = m_destChan;
//        sendBytes(byte, 4);
        NRF_LOG_ERROR("Unsupported fixed mode");
        return false;
    }
    else
    { // Send the one data byte in transparent mode
        return sendBytes(&byte, 1);
    }
}
#endif // #if USE_PACKETS

int8_t _900t20d_setOutputPower(int8_t tx_dBm)
{
    _900t20dTxPwr_t newDesiredPwrSetting;
    // Parse into valid values
    if (tx_dBm <= 10)
    { // 900t20d min 10dBm
        tx_dBm = 10;
        newDesiredPwrSetting = txPwr_10dBm;
    }
    else if (tx_dBm <= 14)
    {
        tx_dBm = 14;
        newDesiredPwrSetting = txPwr_14dBm;
    }
    else if (tx_dBm <= 17)
    {
        tx_dBm = 17;
        newDesiredPwrSetting = txPwr_17dBm;
    }
    else
    { // Set to max of 20
        tx_dBm = 20;
        newDesiredPwrSetting = txPwr_20dBm;
    }
    if (m_desiredTxSetting != newDesiredPwrSetting)
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
        m_desiredTxSetting = newDesiredPwrSetting;
        m_newTxPower = true;
    }
    return tx_dBm; // Return the valid value
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
    if (!readConfigBytes())
    {
        NRF_LOG_ERROR("Error reading operating parameters!");
    }
    if (!_900t20d_readVersionInfo())
    {
        NRF_LOG_ERROR("Error reading version info!");
    }
    // TODO set min TX power here
    m_desiredTxSetting = txPwr_20dBm; // Init here so that our call changes it
    _900t20d_setOutputPower(-30); // Try to set as low as possible
    if (m_newTxPower)
    {
        trySetTxPower(m_desiredTxSetting);
    }
    setMode(_900t20dMode_powerSave);
    m_lastMode = getMode();
    m_inState_ms = 0;
    m_lastPoll_ms = uptimeCounter_getUptimeMs();
    pollers_registerPoller(_900t20dPoll);
}

#endif // #if COMPILE_RADIO_900t20d
