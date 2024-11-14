/*
 * _900t20d.c
 *
 *  Created on: July 5, 2024
 *      Author: Collin Moore
 */

#include "_900t20d.h"

#if COMPILE_RADIO_900T20D

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
    mode_normal, // Normal RX and TX
    mode_wakeUp, // Sends preamble to wake up receiver if receiver is in mode 2
    mode_powerSave, // UART shut down. Monitors for preamble for RF RX.
    mode_sleepConfig, // UART in 9600, 8n1 mode to set parameters.
    mode_unknown,
} _900t20dMode_t;

// 16 bits, MSByte is byte[1], LSByte is byte[2]
#define GET_16BIT_ADDRESS(byte1,byte2) ((((uint16_t)byte1)<<8)|byte2)
// byte[3] (SPED) definitions:
// parity in b7:6
typedef enum
{
    parity_8N1 = 0, // default
    parity_8O1,
    parity_8E1,
    parity_8N1_2, // 3 same as 0
} _900t20dParity_t;
#define GET_UART_PARITY(byte3) (byte3>>6)
#define SET_UART_PARITY(byte3) ((byte3<<6)&0xC0)
// baud rate in b5:3
typedef enum
{
    _900t20dBaud_1200 = 0,
    _900t20dBaud_2400,
    _900t20dBaud_4800,
    _900t20dBaud_9600, // default is 9600
    _900t20dBaud_19200,
    _900t20dBaud_38400,
    _900t20dBaud_57600,
    _900t20dBaud_115200
} _900t20dBaud_t;
#define GET_UART_BAUD(byte3) ((byte3>>3)&0x7)
#define SET_UART_BAUD(byte3) ((byte3<<3)&0x38)
// air data rate in b2:0
typedef enum
{
    airDataRate_0_3k = 0,
    airDataRate_1_2k,
    airDataRate_2_4k,
    airDataRate_4_8k,
    airDataRate_9_6k,
    airDataRate_19_2k
// 0b110 is also 19.2k
// 0b111 is also 19.2k
} _900t20dAirDataRate_t;
#define AIR_DATA_RATE(byte3) (byte3&0x07)
// byte[4] (CHAN) definitions:
// b7:5 reserved, write zeros
// CHAN in b4:0.
#define GET_CHAN(byte4) (byte4&0x1F)
#define CHAN_TO_MHZ(x) (GET_CHAN(x)+862)
#define MHZ_MIN 862
#define MHZ_TO_CHAN(x) (GET_CHAN((x-MHZ_MIN)))
// byte[5] (OPTION) definitions:
/* Fixed Transmission Enable bit is byte[5] b7
 * If SET: Fixed transmission mode, where the first three bytes of each user's data frame can be used
 * as high/low address and channel. The module changes its address and channel when it transmits, then reverts
 * to the normal setting when complete.
 * If CLEAR: Transparent transmission mode: passes the first three bytes through
 */
#define GET_FIXEDTRANSMODE(byte5)(byte5>>7)
// b6 is IO drive mode: 1 for push-pull AUX and TXD, 0 for open-collector outputs (need pullups)
#define GET_IODRIVEMODE(byte5)((byte5>>6)&0x01)
// b5:3 is Wireless wake-up time:
typedef enum
{
    wirelessWakeup_250ms = 0, // default 250ms
    wirelessWakeup_500ms,
    wirelessWakeup_750ms,
    wirelessWakeup_1000ms,
    wirelessWakeup_1250ms,
    wirelessWakeup_1500ms,
    wirelessWakeup_1750ms,
    wirelessWakeup_2000ms

} _900t20dWirelessWakeupTime_t;
#define GET_WIRELESSWAKEUPTIME(byte5) ((byte5>>3)&0x07)
#define SET_WIRELESSWAKEUPTIME(byte5) ((byte5<<3)&0x38)
// b2 is FEC switch, 1 to enable forward Error Correction, 0 to disable
#define GET_FECENABLE(byte5)((byte5>>2)&0x01)
// b1:0 are transmit power. Lower power not recommended in manual
#define GET_TXPOWER(byte5)(byte5&0x03)
// Clear b1:0, then set value
#define SET_TXPOWER(x)(x&0x03)
typedef enum
{
    txPwr_20dBm = 0,
    txPwr_17dBm,
    txPwr_14dBm,
    txPwr_10dBm
} _900t20dTxPwr_t;

typedef struct
{
    // Byte[0] 0xC0 to save, 0xC2 to not save to non-vol
    bool saveParams;
    // Byte[1-2] are 16-bit  address, MSByte first
    uint16_t address; // 16 bits, MSByte in byte[1], LSByte byte[2]
    // Byte[3] speed
    _900t20dParity_t uartParity; // b7:6 of byte[3]
    _900t20dBaud_t uartBaud; // b5:3 of byte[3]
    _900t20dAirDataRate_t airDataRate; // b2:0 of byte[3]
    // byte[4] is channel
    // b7:5 of byte[4] are reserved, write zeros always
    uint8_t channel; // b4:0 channel, (862MHz + channel*1MHz). Default 0x06. So values 0x00 to 0x45 are valid.
    // Byte[5] is options
    bool fixedTransEnable; // b7 fixed Trans Enable, 0 for transparent
    bool ioDrivePushPull; // b6 open-drain if 0, push-pull if 1 (default)
    _900t20dWirelessWakeupTime_t wwt; // b5:3 wireless wakeup time
    bool FEC; // b2 Forward Error Correcting if set.
    _900t20dTxPwr_t txPwr; // b1:0 Tx power

} _900t20dConfig_t;

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

static uint32_t m_inMode_ms;
static uint32_t m_lastPoll_ms;

#if USE_PACKETS
// For receiving packets
static uint8_t m_rxPacketBuffer[128]; // TODO make the max packet size possible
static uint32_t m_rxWriteIndex;
#else

#endif // #if USE_PACKETS

// For changing settings
static _900t20dConfig_t m_desiredSettings;

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
    if (elapsed_ms <= maxWait_ms)
    {
//        NRF_LOG_INFO("Detected AUX high after %d of %d ms. Reads %s now", elapsed_ms, maxWait_ms,
//                     NRF_P0->IN & (1U << _900T20D_AUX_PIN)?"high":"low");
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

static _900t20dMode_t getMode(uint32_t ms_wait)
{
//    NRF_LOG_DEBUG("getMode await aux high:");
    if (!awaitAuxHigh(ms_wait, 0))
    {
        NRF_LOG_ERROR("Failed to get AUX high to read mode");
    }
    _900t20dMode_t mode = 0;
    uint32_t mask = NRF_P0->IN;
    if (mask & (1U << _900T20D_M0_PIN))
    { // set bit 0
        mode |= 0b1;
    }
    if (mask & (1U << _900T20D_M1_PIN))
    { // set bit 1
        mode |= 0b10;
    }
    NRF_LOG_DEBUG("getMode read %d", mode);
    return mode;
}

static bool setMode(_900t20dMode_t mode)
{
//    NRF_LOG_DEBUG("setMode call getMode:");
    _900t20dMode_t currentMode = getMode(0);
    if (currentMode == mode)
    { // Already in desired mode, wait for AUX high to resume
        return true;
    }
// User manual recommends wait for 2ms after verifying that AUX is high to switch mode.
//    NRF_LOG_DEBUG("setMode from mode %d to %d, await aux high:", currentMode, mode);
    if (!awaitAuxHigh(100, 2))
    {
        NRF_LOG_ERROR("Failed to get AUX high before changing mode");
    }
    switch (mode)
    {
        case mode_normal:
            NRF_P0->OUTCLR = (1U << _900T20D_M1_PIN) | (1U << _900T20D_M0_PIN);
        break;
        case mode_wakeUp:
            NRF_P0->OUTCLR = 1U << _900T20D_M1_PIN;
            NRF_P0->OUTSET = 1U << _900T20D_M0_PIN;
        break;
        case mode_powerSave:
            NRF_P0->OUTSET = 1U << _900T20D_M1_PIN;
            NRF_P0->OUTCLR = 1U << _900T20D_M0_PIN;
        break;
        case mode_sleepConfig:
            NRF_P0->OUTSET = (1U << _900T20D_M1_PIN) | (1U << _900T20D_M0_PIN);
        break;
        default:
            NRF_LOG_ERROR("Can't set unknown mode %d", mode)
            ;
            return false;
    }
    m_lastMode = mode;
//    NRF_LOG_DEBUG("setMode await aux high after switch:");
    if (!awaitAuxHigh(25, 1))
    {
        NRF_LOG_ERROR("Failed to get AUX high after changing mode");
        return false;
    }
// TODO make sure to set UART back to 9600 when in mode sleep/config
    return true;
}

static bool sendBytes(uint8_t* pBytes, uint32_t len)
{
// Send bytes on UART
    if (NRF_SUCCESS != uarte0_enqueue(pBytes, len))
    {
        NRF_LOG_ERROR("sendBytes UART enqueue error");
        return false;
    }
// Wait for UART to complete sending
    uint32_t elapsed_us = 0;
    uint32_t maxWait_us = (len * 2 + 3) * 1000; // Wait for 9600baud
    while (!uarte0_isTxDone() && elapsed_us < maxWait_us)
    {
        nrf_delay_us(1);
        elapsed_us++;
    }
    if (elapsed_us < maxWait_us)
    {
//        NRF_LOG_DEBUG("Sent %d bytes in %d ms.", len, elapsed_ms);
//        return true;

//        NRF_LOG_DEBUG("Sent %d bytes in %d ms. Now await AUX high:", len, elapsed_ms);
        return awaitAuxHigh(500, 0);
    }
    else
    {
        NRF_LOG_ERROR("UART send waited for %d us, but still not done", maxWait_us);
        return false;
    }
}

static uint32_t readBytes(uint8_t* pBytes, uint32_t len, uint32_t maxWait_ms)
{
    uint32_t numRead = 0;
    uint32_t elapsed_us = 0;
    while (numRead < len)
    {
        if (uarte0_tryReadByte(&pBytes[numRead]))
        {
            numRead++;
        }
        else if (maxWait_ms)
        { // Not successful in receiving a byte, wait a bit
            nrf_delay_us(1);
            elapsed_us++;
        }
        if (elapsed_us >= (maxWait_ms * 1000))
        { // Leave this while loop
            break;
        }
    }
    if (elapsed_us < (maxWait_ms * 1000))
    {
//        NRF_LOG_DEBUG("Received %d bytes in %d us", len, elapsed_us);
    }
    else if (maxWait_ms)
    { // Print warning if we were waiting
        NRF_LOG_WARNING("readBytes waited for %d ms, but only received %d bytes", maxWait_ms, numRead);
    }
    return numRead;
}

static bool _900t20d_softReset(void)
{
//    NRF_LOG_DEBUG("softReset, set config mode:");
    setMode(mode_sleepConfig); // put in sleep mode to program it
    uint8_t bytes[3] = { 0xC4, 0xC4, 0xC4 };
//    NRF_LOG_DEBUG("softReset sending SRES:");
    if (!sendBytes(bytes, sizeof(bytes)))
    {
        NRF_LOG_ERROR("Error sending SRES command");
    }
    uint32_t elapsedHigh_ms = 0;
    uint32_t maxTilReset_ms = 2000; // Can take up to 1.01 sec to reset.
//    NRF_LOG_DEBUG("softReset await AUX low for %d ms (start of its reset):", maxTilReset_ms);
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
//    NRF_LOG_DEBUG("softReset detected AUX falling after %d ms for reset, await rise again:", elapsedHigh_ms);
    bool auxAck = awaitAuxHigh(1200, 3);
    if (!auxAck)
    {
        NRF_LOG_ERROR("AUX didn't go high after SRES ");
        return false;
    }
//    NRF_LOG_DEBUG("softReset done.");
    return true;
}

static bool readConfigBytes(void)
{
    m_configBytesValid = false; // Mark old as invalid if we are trying to read
//    NRF_LOG_DEBUG("readOpParams start:");
    setMode(mode_sleepConfig); // put in sleep mode to program it
// TODO make sure to set UART back to 9600 when in mode sleep/config
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
//        NRF_LOG_DEBUG("readOpParams got %d bytes: ", numBytesRead);
//        NRF_LOG_HEXDUMP_DEBUG(m_configBytes, numBytesRead);
    }
    if (6 != numBytesRead)
    {
        NRF_LOG_ERROR("readConfigBytes expected %d bytes, only got %d", 6, numBytesRead);
        return false;
    }
// byte[0] should be 0xC0 or 0xC2: C0 to save params, C2 to not save
    if (0xC0 != m_configBytes[0] && 0xC2 != m_configBytes[0])
    {
        NRF_LOG_ERROR("byte[0] was Not C0 or C2, bail out");
        return false;
    }
    NRF_LOG_DEBUG("Address is 0x%04x", GET_16BIT_ADDRESS(m_configBytes[1], m_configBytes[2]));
    NRF_LOG_DEBUG("Uart parity is 0x%x, baud rate 0x%x, air data rate 0x%x", GET_UART_PARITY(m_configBytes[3]),
                  GET_UART_BAUD(m_configBytes[3]),
                  AIR_DATA_RATE(m_configBytes[3]));

    NRF_LOG_DEBUG("Channel byte 0x%x, value 0x%x", m_configBytes[4], GET_CHAN(m_configBytes[4]));

    NRF_LOG_DEBUG("FixedTrans: %s, IO mode: %s", GET_FIXEDTRANSMODE(m_configBytes[5])?"Fixed":"Transparent",
                  GET_IODRIVEMODE(m_configBytes[5])?"PushPull":"OpenColl");
    NRF_LOG_DEBUG("Wireless wakeup time 0x%x, FEC: %s, TxPower: 0x%x", GET_WIRELESSWAKEUPTIME(m_configBytes[5]),
                  GET_FECENABLE(m_configBytes[5])?"enabled":"disabled",
                  GET_TXPOWER(m_configBytes[5]));
//    NRF_LOG_DEBUG("readConfigBytes done");
    m_configBytesValid = true;
    return true;
}

static bool writeConfigBytes(uint8_t* pConfigBytes, bool save, bool verify)
{
    _900t20dMode_t lastMode = getMode(0); // Save and restore mode
    setMode(mode_sleepConfig); // Set to config mode
// Now write the params, then read back to verify that they are as desired
// Manual says to write 6 bytes, with 0xC0 leading to save params
    pConfigBytes[0] = 0xC2; // Don't save
    if (save)
    {
        pConfigBytes[0] = 0xC0; // Save parameters when powering down
    }
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

static bool _900t20d_readVersionInfo(void)
{
    setMode(mode_sleepConfig); // put in sleep mode to program it
    uint8_t txBytes[3] = { 0xC3, 0xC3, 0xC3 };
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

static bool trySendConfig(void)
{
    if (!m_configBytesValid)
    { // Read if we don't have a config
        readConfigBytes();
    }
    if (!m_configBytesValid)
    { // Failed to read config, don't set anything
        return false;
    }
// Make a new array then edit it, see if it has any changes to send
    uint8_t newParams[sizeof(m_configBytes)];
    // Byte[0] 0xC0 to save, 0xC2 to not save to non-vol
    newParams[0] = m_desiredSettings.saveParams ? 0xC0 : 0xC2;
    // Byte[1-2] are 16-bit  address, MSByte first
    newParams[1] = (uint8_t)(m_desiredSettings.address >> 8);
    newParams[2] = (uint8_t)(m_desiredSettings.address & 0xFF);
    // Byte[3] speed settings
    newParams[3] = 0;
    newParams[3] |= SET_UART_PARITY(m_desiredSettings.uartParity);
    newParams[3] |= SET_UART_BAUD(m_desiredSettings.uartBaud);
    newParams[3] |= AIR_DATA_RATE(m_desiredSettings.airDataRate);
    // byte[4] is channel
    // b7:5 of byte[4] are reserved, write zeros always
    // b4:0 channel, (862MHz + channel*1MHz). Default 0x06. So values 0x00 to 0x45 are valid.
    newParams[4] = m_desiredSettings.channel & 0x1F;
    // Byte[5] is options
    newParams[5] = 0;
    // b7 fixed Trans Enable, 0 for transparent
    newParams[5] |= m_desiredSettings.fixedTransEnable ? 0x80 : 0x00;
    // b6 open-drain if 0, push-pull if 1 (default)
    newParams[5] |= m_desiredSettings.ioDrivePushPull ? 0x40 : 0x00;
    // b5:3 wireless wakeup time
    newParams[5] |= SET_WIRELESSWAKEUPTIME(m_desiredSettings.wwt);
    // b2 Forward Error Correcting if set.
    newParams[5] |= m_desiredSettings.FEC ? 0x04 : 0x00;
    // b1:0 Tx power
    newParams[5] |= SET_TXPOWER(m_desiredSettings.txPwr);
    // check if any changed
    if (0 == memcmp(m_configBytes, newParams, sizeof(m_configBytes)))
    { // Config same, ignore
        return true;
    }
    if (writeConfigBytes(newParams, true, true))
    {
        NRF_LOG_DEBUG("Wrote new config params");
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
        NRF_LOG_INFO("Read byte 0x%x", rxByte);
//        globalInts_setMachineState(rxByte);
#endif // #if USE_PACKETS
        // See if there are any more in UART rxBuf
        numRead = readBytes(&rxByte, 1, 0);
    }
}

static void _900t20dPoll(void)
{
// Make sure it is at desired TX power
    trySendConfig();
// See if state has changed via 2 GPIOs
    _900t20dMode_t currentMode = getMode(0);
    if (currentMode != m_lastMode)
    {
        m_inMode_ms = 0;
    }
    else
    { // If in state, increment timer
        m_inMode_ms += uptimeCounter_elapsedSince(m_lastPoll_ms);
    }
    switch (currentMode)
    {
        case mode_normal:
            case mode_wakeUp:
            case mode_powerSave:
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
// Set to wakeUp mode so that it sends preamble to sleeping receiver
    setMode(mode_wakeUp); // UART TX available in this mode, but not in sleep mode
// TODO any other delays?
// TODO set baud rate

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

int8_t _900t20d_setOutputPower(int8_t desired_dBm)
{
    _900t20dTxPwr_t newDesiredPwrSetting;
    int8_t tx_dBm = desired_dBm;
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
    if (m_desiredSettings.txPwr != newDesiredPwrSetting)
    {
        m_desiredSettings.txPwr = newDesiredPwrSetting;
        NRF_LOG_INFO("Desire change of power to %d dBm, can do %d", desired_dBm, tx_dBm);
#ifdef UART_TX_PIN
        char strBuf[96];
        int strLen = snprintf(strBuf, sizeof(strBuf), "Desire TX %d dBm", tx_dBm);
        if (0 < strLen)
        {
            uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
        }
#endif // #ifdef UART_TX_PIN
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
    if (!_900t20d_readVersionInfo())
    {
        NRF_LOG_ERROR("Error reading version info!");
    }

    // Set desired parameters in our struct, then call function to read current and overwrite if needed.
    m_desiredSettings.FEC = true;
    m_desiredSettings.address = 0x0000; // TODO set an address based on chipID or something?
    m_desiredSettings.airDataRate = airDataRate_2_4k; // 2.4k is default
    m_desiredSettings.channel = MHZ_TO_CHAN(915); // Set channel from MHz desired.
    m_desiredSettings.fixedTransEnable = false; // Transparent
    m_desiredSettings.ioDrivePushPull = true;
    m_desiredSettings.txPwr = txPwr_10dBm; // Lowest to start with, higher to test.
    m_desiredSettings.uartBaud = _900t20dBaud_9600; // TODO speed up
    m_desiredSettings.uartParity = parity_8N1; // Default
    m_desiredSettings.wwt = wirelessWakeup_250ms;

    m_configBytesValid = false; // Trigger a get of settings
    trySendConfig(); // Set our desired settings

    setMode(mode_powerSave); // Shut down config UART, wait for preamble on air.
    m_lastMode = getMode(0);
    m_inMode_ms = 0;
    m_lastPoll_ms = uptimeCounter_getUptimeMs();
    pollers_registerPoller(_900t20dPoll);
}

#endif // #if COMPILE_RADIO_900t20d
