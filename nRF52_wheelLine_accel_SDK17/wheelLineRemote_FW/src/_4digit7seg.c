/*
 * ht16k33.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#include "_4digit7seg.h"

#include "i2c.h"

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

#define SEVENSEG_DIGITS 5 ///< # Digits in 7-seg displays, plus NUL end

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

#define POLL_ITVL_MS 1000 // non-zero to poll
#if POLL_ITVL_MS
#include "pollers.h"
#include "uptimeCounter.h"
#endif // #if POLL_ITVL_MS

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static uint8_t m_position;

#if POLL_ITVL_MS
static uint32_t m_lastPoll_ms;
#endif // #if POLL_ITVL_MS

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

void _4digit7seg_setDisplayState(dispState_t desired)
{
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
    i2c_writeBytes(HT16K33_DEVADDR, &buffer, 1);
}

void _4digit7seg_setBrightness(uint8_t zeroTo15)
{
    uint8_t buffer = HT16K33_CMD_BRIGHTNESS | (zeroTo15 & 0x0F);
    i2c_writeBytes(HT16K33_DEVADDR, &buffer, 1);
}

static uint16_t m_displaybuffer[8]; ///< Raw display data

void _4digit7seg_writeDisplay(void)
{
    uint8_t buffer[17]; // Send 1 address and 16 bytes of data.
    buffer[0] = HT16K33_ADDR_DDAP; // start at address 0, write them all
    for (uint8_t i = 0; i < 8; i++)
    { // Read in data from our m_displayBuffer
        buffer[1 + 2 * i] = m_displaybuffer[i] & 0xFF;
        buffer[2 + 2 * i] = m_displaybuffer[i] >> 8;
    }
    i2c_writeBytes(HT16K33_DEVADDR, buffer, 17);
}

/******************************* 7 SEGMENT OBJECT */

void _4digit7seg_printU32(uint32_t n, int base)
{
    if (base == 0)
        _4digit7seg_writeChar(n);
    else
        _4digit7seg_printNumber(n, base);
}

void _4digit7seg_println(void)
{
    m_position = 0;
}

void _4digit7seg_printlnStr(const char* c)
{
    _4digit7seg_writeStr(c, strlen(c));
    _4digit7seg_println();
}

void _4digit7seg_printlnChar(char c)
{
    _4digit7seg_writeChar(c);
    _4digit7seg_println();
}

void _4digit7seg_printlnU32(uint32_t b, int base)
{
    _4digit7seg_print(b, base);
    _4digit7seg_println();
}

uint32_t _4digit7seg_writeChar(char c)
{
    uint8_t r = 0;
    if (c == '\n')
    {
        m_position = 0;
        return;
    }
    if (c == '\r')
    {
        m_position = 0;
        return;
    }
    if ((c >= ' ') && (c <= 127))
    {
        _4digit7seg_writeDigitAscii(m_position, c, false);
        r = 1;
    }
    m_position++;
    if (m_position == 2)
    { // skip the colon slot
        m_position++;
    }
    return r;
}

uint32_t _4digit7seg_writeStr(const char* buffer, uint32_t size)
{
    uint32_t n = 0;
    while (n < size)
    {
        _4digit7seg_writeChar(buffer[n]);
        n++;
    }
    // Clear unwritten positions
    for (uint8_t i = m_position; i < 5; i++)
    {
        _4digit7seg_writeDigitRaw(i, 0x00);
    }
    return n;
}

void _4digit7seg_writeDigitRaw(uint8_t d, uint8_t bitmask)
{
    if (d > 4)
    {
        return;
    }
    m_displaybuffer[d] = bitmask;
}

void _4digit7seg_drawColon(bool state)
{
    if (state)
    {
        m_displaybuffer[2] = 0x2;
    }
    else
    {
        m_displaybuffer[2] = 0;
    }
}

void _4digit7seg_writeColon(void)
{
    uint8_t buffer[3];
    buffer[0] = HT16K33_ADDR_DDAP + 0x04; // Start at this address
    buffer[1] = (uint8_t)(m_displaybuffer[2] & 0xFF);
    buffer[2] = (uint8_t)(m_displaybuffer[2] >> 8);
    i2c_writeBytes(HT16K33_DEVADDR, buffer, 3);
}

void _4digit7seg_writeDigitNum(uint8_t d, uint8_t num, bool dot)
{
    if (d > 4 || num > 15)
        return;

    if (num >= 10)
    { // Hex characters
        switch (num)
        {
            case 10:
                _4digit7seg_writeDigitAscii(d, 'a', dot);
            break;
            case 11:
                _4digit7seg_writeDigitAscii(d, 'B', dot);
            break;
            case 12:
                _4digit7seg_writeDigitAscii(d, 'C', dot);
            break;
            case 13:
                _4digit7seg_writeDigitAscii(d, 'd', dot);
            break;
            case 14:
                _4digit7seg_writeDigitAscii(d, 'E', dot);
            break;
            case 15:
                _4digit7seg_writeDigitAscii(d, 'F', dot);
            break;
        }
    }

    else
        _4digit7seg_writeDigitAscii(d, num + 48, dot); // use ASCII offset
}

void _4digit7seg_writeDigitAscii(uint8_t d, uint8_t c, bool dot)
{
    if (d > 4)
    {
        return;
    }
//    uint8_t font = pgm_read_byte(sevensegfonttable + c - 32);
    _4digit7seg_writeDigitRaw(d, (sevensegfonttable[c - 32]) | (dot << 7));
}

void _4digit7seg_print(long n, int base)
{
    _4digit7seg_printNumber(n, base);
}

void _4digit7seg_printNumber(long n, uint8_t base)
{
    _4digit7seg_printFloat(n, 0, base);
}

void _4digit7seg_printFloat(double n, uint8_t fracDigits, uint8_t base)
{
    uint8_t numericDigits = 4; // available digits on display
    bool isNegative = false;   // true if the number is negative

    // is the number negative?
    if (n < 0)
    {
        isNegative = true; // need to draw sign later
        --numericDigits;   // the sign will take up one digit
        n *= -1;           // pretend the number is positive
    }

    // calculate the factor required to shift all fractional digits
    // into the integer part of the number
    double toIntFactor = 1.0;
    for (int i = 0; i < fracDigits; ++i)
        toIntFactor *= base;

    // create integer containing digits to display by applying
    // shifting factor and rounding adjustment
    uint32_t displayNumber = n * toIntFactor + 0.5;

    // calculate upper bound on displayNumber given
    // available digits on display
    uint32_t tooBig = 1;
    for (int i = 0; i < numericDigits; ++i)
        tooBig *= base;

    // if displayNumber is too large, try fewer fractional digits
    while (displayNumber >= tooBig)
    {
        --fracDigits;
        toIntFactor /= base;
        displayNumber = n * toIntFactor + 0.5;
    }

    // did toIntFactor shift the decimal off the display?
    if (toIntFactor < 1)
    {
        _4digit7seg_printError();
    }
    else
    {
        // otherwise, display the number
        int8_t displayPos = 4;

        for (uint8_t i = 0; displayNumber || i <= fracDigits; ++i)
        {
            bool displayDecimal = (fracDigits != 0 && i == fracDigits);
            _4digit7seg_writeDigitNum(displayPos--, displayNumber % base, displayDecimal);
            if (displayPos == 2)
            {
                _4digit7seg_writeDigitRaw(displayPos--, 0x00);
            }
            displayNumber /= base;
        }

        // display negative sign if negative
        if (isNegative)
        {
            _4digit7seg_writeDigitRaw(displayPos--, 0x40);
        }

        // clear remaining display positions
        while (displayPos >= 0)
        {
            _4digit7seg_writeDigitRaw(displayPos--, 0x00);
        }
    }
}

void _4digit7seg_printError(void)
{
    for (uint8_t i = 0; i < SEVENSEG_DIGITS; ++i)
    {
        _4digit7seg_writeDigitRaw(i, (i == 2 ? 0x00 : 0x40));
    }
}

#if POLL_ITVL_MS
static uint32_t m_printerState;
static void poll(void)
{
    if (uptimeCounter_elapsedSince(m_lastPoll_ms) > POLL_ITVL_MS)
    {
        _4digit7seg_setBrightness((m_lastPoll_ms / 1000) % 15);
        NRF_LOG_DEBUG("Set brightness to %d", (m_lastPoll_ms / 1000) % 15);

//        _4digit7seg_drawColon(true);
//        _4digit7seg_writeColon();
        switch (m_printerState)
        {
            case 1:
                _4digit7seg_writeStr("\nrt 1", 5);
            break;
            case 2:
                _4digit7seg_writeStr("\nRT 2", 5);
            break;
            case 3:
                _4digit7seg_writeStr("\nlt 1", 5);
            break;
            case 4:
                _4digit7seg_writeStr("\nLT 2", 5);
            break;
            default:
                m_printerState = 0;
            break;
        }
        _4digit7seg_writeDisplay();
        m_printerState++;

//        _4digit7seg_writeDigitAscii(1,'a',false);
//        _4digit7seg_writeDigitAscii(1,'a',false);
//        _4digit7seg_writeDigitAscii(1,'a',false);

        m_lastPoll_ms = uptimeCounter_getUptimeMs();
    }
}
#endif // #if POLL_ITVL_MS

void _4digit7seg_init(void)
{
    m_position = 0;
    i2c_init(); // Make sure bus is enabled.

    // turn on oscillator
    uint8_t byte = HT16K33_CMD_SYS_SETUP_OSC_ON;
    i2c_writeBytes(HT16K33_DEVADDR, &byte, 1);

    uint8_t rxByte = 0x12;
    ret_code_t ret = i2c_readByte(HT16K33_DEVADDR, HT16K33_ADDR_DDAP, &rxByte);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("_4digit7seg_init failed to read I2C");
        return;
    }
    NRF_LOG_DEBUG("Read DDAP at 0x%x", rxByte);
    ret = i2c_readByte(HT16K33_DEVADDR, HT16K33_ADDR_INTFLAGS, &rxByte);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("_4digit7seg_init failed to read I2C");
        return;
    }
    NRF_LOG_DEBUG("Read intflags at 0x%x", rxByte);
    ret = i2c_readByte(HT16K33_DEVADDR, HT16K33_ADDR_KADP, &rxByte);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("_4digit7seg_init failed to read I2C");
        return;
    }
    NRF_LOG_DEBUG("Read KADP at 0x%x", rxByte);

    // internal RAM powers up with garbage/random values.
    // ensure internal RAM is cleared before turning on display
    // this ensures that no garbage pixels show up on the display
    // when it is turned on.
    for (uint8_t i = 0; i < 8; i++)
    {
        m_displaybuffer[i] = 0;
    }
    _4digit7seg_writeDisplay();

    _4digit7seg_setDisplayState(dispState_onSolid);

    _4digit7seg_setBrightness(0); // 0 is still on

#if POLL_ITVL_MS
    m_lastPoll_ms = uptimeCounter_getUptimeMs();
    pollers_registerPoller(poll);
#endif // #if POLL_ITVL_MS

}

