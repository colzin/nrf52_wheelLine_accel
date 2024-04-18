/*
 * ht16k33.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#include "_4digit7seg.h"

#include "globalInts.h"
#include "i2c1.h"

#include "version.h" // for startup print

#define NRF_LOG_MODULE_NAME _4digit7seg
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define HT16K33_DEVADDR 0x70

/* Display data address pointer, bits 3:0 are address bits. R/W
 * So, addresses 0x00 to 0x0F are data addresses, to write bytes into
 */
#define HT16K33_ADDR_DDAP 0x00

// System Setup, write only. b0 of system setup enables oscillator
#define HT16K33_CMD_SYS_SETUP_OSC_OFF   0x20
#define HT16K33_CMD_SYS_SETUP_OSC_ON    0x21
// Key address data pointer, bits 3:0 are address bits. Read only.
#define HT16K33_ADDR_KADP 0x40
// INT flag address pointer, defines INT flag address. Reads INT flag status. Read only.
#define HT16K33_ADDR_INTFLAGS 0x60
/* Display setup, write only.
 *  b0=1 for display on, 0 for display off.
 *  b2:1 are blinking frequency:
 *  0b0000 = blinking off, 0b0010 = blink at 2Hz, 0b0100 = blink at 1Hz,  0b0110 = blink at 0.5Hz
 */
#define HT16K33_CMD_DISP_SETUP_OFF 0x80
#define HT16K33_CMD_DISP_SETUP_ON_SOLID 0x81
#define HT16K33_CMD_DISP_SETUP_BLINK_2HZ 0x83
#define HT16K33_CMD_DISP_SETUP_BLINK_1HZ 0x85
#define HT16K33_CMD_DISP_SETUP_BLINK_0_5HZ 0x87

// Row/INT setting, b1=act, b0=row/int. Write Only
#define HT16K33_CMD_ROW_INT 0xC0
// Dimming setting, 0xE0, b3:0 are brightness, Write only.
#define HT16K33_CMD_BRIGHTNESS 0xE0

#define SEVENSEG_DIGITS 5 ///< # Digits in 7-seg displays: 4 chars, don't count colon in middle

static const uint8_t sevensegfonttable[] =
        {

        0b00000000, // (space)
          0b10000110, // !
          0b00100010, // "
          0b01111110, // #
          0b01101101, // $
          0b11010010, // %
          0b01000110, // &
          0b00100000, // '
          0b00101001, // (
          0b00001011, // )
          0b00100001, // *
          0b01110000, // +
          0b00010000, // ,
          0b01000000, // -
          0b10000000, // .
          0b01010010, // /
          0b00111111, // 0
          0b00000110, // 1
          0b01011011, // 2
          0b01001111, // 3
          0b01100110, // 4
          0b01101101, // 5
          0b01111101, // 6
          0b00000111, // 7
          0b01111111, // 8
          0b01101111, // 9
          0b00001001, // :
          0b00001101, // ;
          0b01100001, // <
          0b01001000, // =
          0b01000011, // >
          0b11010011, // ?
          0b01011111, // @
          0b01110111, // A
          0b01111100, // B
          0b00111001, // C
          0b01011110, // D
          0b01111001, // E
          0b01110001, // F
          0b00111101, // G
          0b01110110, // H
          0b00110000, // I
          0b00011110, // J
          0b01110101, // K
          0b00111000, // L
          0b00010101, // M
          0b00110111, // N
          0b00111111, // O
          0b01110011, // P
          0b01101011, // Q
          0b00110011, // R
          0b01101101, // S
          0b01111000, // T
          0b00111110, // U
          0b00111110, // V
          0b00101010, // W
          0b01110110, // X
          0b01101110, // Y
          0b01011011, // Z
          0b00111001, // [
          0b01100100, //
          0b00001111, // ]
          0b00100011, // ^
          0b00001000, // _
          0b00000010, // `
          0b01011111, // a
          0b01111100, // b
          0b01011000, // c
          0b01011110, // d
          0b01111011, // e
          0b01110001, // f
          0b01101111, // g
          0b01110100, // h
          0b00010000, // i
          0b00001100, // j
          0b01110101, // k
          0b00110000, // l
          0b00010100, // m
          0b01010100, // n
          0b01011100, // o
          0b01110011, // p
          0b01100111, // q
          0b01010000, // r
          0b01101101, // s
          0b01111000, // t
          0b00011100, // u
          0b00011100, // v
          0b00010100, // w
          0b01110110, // x
          0b01101110, // y
          0b01011011, // z
          0b01000110, // {
          0b00110000, // |
          0b01110000, // }
          0b00000001, // ~
          0b00000000, // del
        };

#define TEST_POLL_ITVL_MS 0 // 1500 // non-zero to poll
#if TEST_POLL_ITVL_MS
#include "pollers.h"
#include "uptimeCounter.h"
#endif // #if TEST_POLL_ITVL_MS

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static bool m_initted = false;

#if TEST_POLL_ITVL_MS
static uint32_t m_lastPoll_ms;
#else
static machineState_t m_lastMachState;
#endif // #if TEST_POLL_ITVL_MS

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

void _4digit7seg_setDisplayState(dispState_t desired)
{
    if (!m_initted)
    {
        return;
    }
    uint8_t buffer;
    switch (desired)
    {
        case dispState_off:
            buffer = HT16K33_CMD_DISP_SETUP_OFF;
        break;
        case dispState_onSolid:
            buffer = HT16K33_CMD_DISP_SETUP_ON_SOLID;
        break;
        case dispState_blink2Hz:
            buffer = HT16K33_CMD_DISP_SETUP_BLINK_2HZ;
        break;
        case dispState_blink1Hz:
            buffer = HT16K33_CMD_DISP_SETUP_BLINK_1HZ;
        break;
        case dispState_blink0_5Hz:
            buffer = HT16K33_CMD_DISP_SETUP_BLINK_0_5HZ;
        break;
        default:
            NRF_LOG_ERROR("Unknown display state %d, ignoring", desired)
            ;
            return;
        break;
    }
    i2c1_writeBytes(HT16K33_DEVADDR, &buffer, 1);
}

void _4digit7seg_setBrightness(uint8_t zeroTo15)
{
    if (!m_initted)
    {
        return;
    }
    uint8_t buffer = (uint8_t)(HT16K33_CMD_BRIGHTNESS | (zeroTo15 & 0x0F));
    i2c1_writeBytes(HT16K33_DEVADDR, &buffer, 1);
}

static uint16_t m_displaybuffer[8]; ///< Raw display data

static void writeDisplay(void)
{
    uint8_t buffer[17]; // Send 1 address and 16 bytes of data.
    buffer[0] = HT16K33_ADDR_DDAP; // start at address 0, write them all
    for (uint8_t i = 0; i < 8; i++)
    { // Read in data from our m_displayBuffer
        buffer[1 + 2 * i] = (uint8_t)(m_displaybuffer[i] & 0xFF);
        buffer[2 + 2 * i] = (uint8_t)(m_displaybuffer[i] >> 8);
    }
    i2c1_writeBytes(HT16K33_DEVADDR, buffer, 17);
}

/******************************* 7 SEGMENT OBJECT */

static void writeDigitRaw(uint8_t d, uint8_t bitmask)
{
    if (d >= SEVENSEG_DIGITS)
    {
        NRF_LOG_WARNING("no room for digit at %d", d);
        return;
    }
    m_displaybuffer[d] = bitmask;
}

static void writeDigitAscii(uint8_t pos, uint8_t c, bool dot)
{
    if (pos >= SEVENSEG_DIGITS)
    { // We don't do colon, just the 4 digits. This is probably the \0 on a string, ignore anyway
//        NRF_LOG_WARNING("no room for char %c at %d", c, pos);
        return;
    }
    if ((c >= ' ') && (c <= 127))
    {
//        NRF_LOG_DEBUG("Writing ASCII char %c to pos %d, %s", c, pos, dot ? "with dot" : "");
        writeDigitRaw(pos, (uint8_t)((sevensegfonttable[c - 32]) | (dot << 7)));
    }
}

static void writeColon(void)
{
    uint8_t buffer[3];
    buffer[0] = HT16K33_ADDR_DDAP + 0x04; // Start at this address
    buffer[1] = (uint8_t)(m_displaybuffer[2] & 0xFF);
    buffer[2] = (uint8_t)(m_displaybuffer[2] >> 8);
    i2c1_writeBytes(HT16K33_DEVADDR, buffer, 3);
}

static void drawColon(bool state)
{
    if (state)
    {
        m_displaybuffer[2] |= 0x2;
//        NRF_LOG_DEBUG("Set colon ON");
    }
    else
    {
        m_displaybuffer[2] &= (uint16_t)(~0x02);
//        NRF_LOG_DEBUG("Set colon OFF");
    }
}

static void clearBuffer(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        m_displaybuffer[i] = 0;
    }
}

void _4digit7seg_writeStr(const char* buffer)
{ // Max we can handle is 4 digits, 1 colon, and 4 dots, so 9 total
    if (!m_initted)
    {
        return;
    }
    clearBuffer();
    // Increment through the full digits
    uint8_t bufIdx = 0, screenIdx = 0;
    while (screenIdx <= SEVENSEG_DIGITS)
    {
        if (2 == screenIdx)
        { // Digit[2] can only be a colon
            if (':' == buffer[bufIdx])
            { // Index 2 could be colon, or normal digit after the colon
                drawColon(true);
                bufIdx++;
            }
            // Always increment the screen index, so next char will be a full char
            screenIdx++;
        }
        else if ('.' == buffer[bufIdx + 1])
        { // If there is a dot following, use the dot, and increment bufIndex twice
            writeDigitAscii(screenIdx, buffer[bufIdx], true);
            bufIdx += 2; // skip the dot next time, it's in the last char printed
            screenIdx++;
        }
        else
        {
            if (':' == buffer[bufIdx])
            { // Skip over colon
                if (bufIdx != 2)
                {
                    NRF_LOG_ERROR("Can't print : except in middle");
                }
                bufIdx++; // skip this char, don't increment the screenIndex
            }
            else
            {
                writeDigitAscii(screenIdx, buffer[bufIdx], false);
                bufIdx++;
                screenIdx++;
            }
        }
    }
    // Send to the display
    writeDisplay();
}

#if TEST_POLL_ITVL_MS
static uint32_t m_printerState;
static void testPoll(void)
{
    if (uptimeCounter_elapsedSince(m_lastPoll_ms) > TEST_POLL_ITVL_MS)
    {
////        _4digit7seg_setBrightness((uint8_t)((m_lastPoll_ms / 1000) % 15));
//        NRF_LOG_DEBUG("Set brightness to %d", (m_lastPoll_ms / 1000) % 15);

//        _4digit7seg_drawColon(true);
//        _4digit7seg_writeColon();
        switch (m_printerState)
        {
            case 1:
                _4digit7seg_writeStr("off");
            break;
            case 2:
                _4digit7seg_writeStr("RT 3");
            break;
            case 3:
                _4digit7seg_writeStr("lt: 4");
            break;
            case 4:
                _4digit7seg_writeStr("LT:56");
            break;
            default:
                m_printerState = 0;
            break;
        }
        m_printerState++;

//        _4digit7seg_writeDigitAscii(1,'a',false);
//        _4digit7seg_writeDigitAscii(1,'a',false);
//        _4digit7seg_writeDigitAscii(1,'a',false);

        m_lastPoll_ms = uptimeCounter_getUptimeMs();
    }
}

#else
static void machineStatePoll(void)
{
    machineState_t stateNow = globalInts_getMachineState();
    if (stateNow != m_lastMachState)
    {
        switch (stateNow)
        {
            // TODO update the display
            case machState_justPoweredOn:
                _4digit7seg_writeStr("P ON");
            break;
            case machState_startEngine:
                _4digit7seg_writeStr("STRT");
            break;
            case machState_runEngineHydIdle:
                _4digit7seg_writeStr("IdLE");
            break;
            case machState_runEngineHydFwd:
                _4digit7seg_writeStr("FVVD");
            break;
            case machState_runEngineHydRev:
                _4digit7seg_writeStr("REV");
            break;
            case machState_killEngine:
                _4digit7seg_writeStr("kill");
            break;
        }
        m_lastMachState = stateNow;
    }
}
#endif // #if TEST_POLL_ITVL_MS

static void poll(void)
{
#if TEST_POLL_ITVL_MS
    testPoll();
#else
    machineStatePoll();
#endif // #if TEST_POLL_ITVL_MS

}

void _4digit7seg_init(void)
{
    i2c1_init(); // Make sure bus is enabled.

// turn on oscillator
    uint8_t byte = HT16K33_CMD_SYS_SETUP_OSC_ON;
    ret_code_t ret = i2c1_writeBytes(HT16K33_DEVADDR, &byte, 1);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("7seg not found at address 0x%x, bailing out", HT16K33_DEVADDR);
        m_initted = false;
        return;
    }

//    uint8_t rxByte = 0x12;
//    ret_code_t ret = i2c1_readByte(HT16K33_DEVADDR, HT16K33_ADDR_DDAP, &rxByte);
//    if (NRF_SUCCESS != ret)
//    {
//        NRF_LOG_ERROR("_4digit7seg_init failed to read I2C");
//        return;
//    }
//    NRF_LOG_DEBUG("Read DDAP at 0x%x", rxByte);
//    ret = i2c1_readByte(HT16K33_DEVADDR, HT16K33_ADDR_INTFLAGS, &rxByte);
//    if (NRF_SUCCESS != ret)
//    {
//        NRF_LOG_ERROR("_4digit7seg_init failed to read I2C");
//        return;
//    }
//    NRF_LOG_DEBUG("Read intflags at 0x%x", rxByte);
//    ret = i2c1_readByte(HT16K33_DEVADDR, HT16K33_ADDR_KADP, &rxByte);
//    if (NRF_SUCCESS != ret)
//    {
//        NRF_LOG_ERROR("_4digit7seg_init failed to read I2C");
//        return;
//    }
//    NRF_LOG_DEBUG("Read KADP at 0x%x", rxByte);

// internal RAM powers up with garbage/random values.
// ensure internal RAM is cleared before turning on display
// this ensures that no garbage pixels show up on the display
// when it is turned on.
    clearBuffer();
    writeDisplay();

    _4digit7seg_setDisplayState(dispState_onSolid);

    _4digit7seg_setBrightness(10); // 0 is still on

    m_initted = true;

#if POLL_ITVL_MS
    m_lastPoll_ms = uptimeCounter_getUptimeMs();
#endif // #if POLL_ITVL_MS

    pollers_registerPoller(poll);
}

