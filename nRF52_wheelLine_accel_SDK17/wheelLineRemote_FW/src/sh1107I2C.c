/*
 * sh1107I2C.c
 *
 *  Created on: Feb 7, 2024
 *      Author: Collin Moore
 */

#include "sh1107I2C.h"

#include "i2c1.h"
#include "pollers.h"
#include "uptimeCounter.h"
#include "version.h"

#define NRF_LOG_MODULE_NAME sh1107
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define DEV_I2C_ADDR 0x3C

#define GRAYOLED_SETCONTRAST 0x81   ///< Generic contrast for almost all OLEDs
#define GRAYOLED_NORMALDISPLAY 0xA6 ///< Generic non-invert for almost all OLEDs
#define GRAYOLED_INVERTDISPLAY 0xA7 ///< Generic invert for almost all OLEDs

#define MONOOLED_BLACK 0   ///< Default black 'color' for monochrome OLEDS
#define MONOOLED_WHITE 1   ///< Default white 'color' for monochrome OLEDS
#define MONOOLED_INVERSE 2 ///< Default inversion command for monochrome OLEDS

/// fit into the SH110X_ naming scheme
#define SH110X_BLACK 0   ///< Draw 'off' pixels
#define SH110X_WHITE 1   ///< Draw 'on' pixels
#define SH110X_INVERSE 2 ///< Invert pixels

// Uncomment to disable Adafruit splash logo
//#define SH110X_NO_SPLASH

#define SH110X_MEMORYMODE 0x20          ///< See datasheet
#define SH110X_COLUMNADDR 0x21          ///< See datasheet
#define SH110X_PAGEADDR 0x22            ///< See datasheet
#define SH110X_SETCONTRAST 0x81         ///< See datasheet
#define SH110X_CHARGEPUMP 0x8D          ///< See datasheet
#define SH110X_SEGREMAP 0xA0            ///< See datasheet
#define SH110X_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SH110X_DISPLAYALLON 0xA5        ///< Not currently used
#define SH110X_NORMALDISPLAY 0xA6       ///< See datasheet
#define SH110X_INVERTDISPLAY 0xA7       ///< See datasheet
#define SH110X_SETMULTIPLEX 0xA8        ///< See datasheet
#define SH110X_DCDC 0xAD                ///< See datasheet
#define SH110X_DISPLAYOFF 0xAE          ///< See datasheet
#define SH110X_DISPLAYON 0xAF           ///< See datasheet
#define SH110X_SETPAGEADDR                                                     \
  0xB0 ///< Specify page address to load display RAM data to page address
///< register
#define SH110X_COMSCANINC 0xC0         ///< Not currently used
#define SH110X_COMSCANDEC 0xC8         ///< See datasheet
#define SH110X_SETDISPLAYOFFSET 0xD3   ///< See datasheet
#define SH110X_SETDISPLAYCLOCKDIV 0xD5 ///< See datasheet
#define SH110X_SETPRECHARGE 0xD9       ///< See datasheet
#define SH110X_SETCOMPINS 0xDA         ///< See datasheet
#define SH110X_SETVCOMDETECT 0xDB      ///< See datasheet
#define SH110X_SETDISPSTARTLINE                                                \
  0xDC ///< Specify Column address to determine the initial display line or
///< COM0.

#define SH110X_SETLOWCOLUMN 0x00  ///< Not currently used
#define SH110X_SETHIGHCOLUMN 0x10 ///< Not currently used
#define SH110X_SETSTARTLINE 0x40  ///< See datasheet

#define DISPLAY_INVERT_ITVL_MS 1800
#define DISPLAY_CONTRAST_ITVL_MS 100

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static uint32_t m_lastInvert_ms, m_lastContrastSet_ms;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static ret_code_t writeCommand(uint8_t byte)
{
    uint8_t toWrite[2];
    toWrite[0] = 0x00; // use two bytes, first being 0x00 to write a command
    toWrite[1] = byte;
    return i2c1_writeBytes(DEV_I2C_ADDR, &toWrite, 2);
}
static ret_code_t writeCommandList(uint8_t* pCommands, uint8_t num)
{
    ret_code_t ret = i2c1_tx(DEV_I2C_ADDR, pCommands, num, true);
    uint8_t dcByte = 0x00; // Co = 0 and D/C = 0
    ret |= i2c1_tx(DEV_I2C_ADDR, &dcByte, 1, false);
    return ret;
}

static ret_code_t writeReg(uint8_t regAddr, uint8_t byte)
{
    uint8_t toWrite[2];
    toWrite[0] = regAddr;
    toWrite[1] = byte;
    return i2c1_writeBytes(DEV_I2C_ADDR, toWrite, 2);
}

static ret_code_t readReg(uint8_t regAddr, uint8_t* pRxByte)
{
    return i2c1_readByte(DEV_I2C_ADDR, regAddr, pRxByte);
}

static ret_code_t readRegs(uint8_t startAddr, uint8_t* pRxByte, uint32_t len)
{
    return i2c1_readBytes(DEV_I2C_ADDR, startAddr, pRxByte, len);
}

static void invertDisplay(bool invert)
{
    NRF_LOG_DEBUG("Setting invert: %s", invert ? "Inverted" : "normal");
    writeCommand(invert ? GRAYOLED_INVERTDISPLAY : GRAYOLED_NORMALDISPLAY);
}

static void setContrast(uint8_t level)
{
    NRF_LOG_DEBUG("Setting contrast to %d", level);
//    uint8_t cmdList[2];
//    cmdList[0] = GRAYOLED_SETCONTRAST;
//    cmdList[1] = level;
    writeReg(GRAYOLED_SETCONTRAST, level);
}

static bool m_inverted;
static uint8_t m_contrastIterator;
static void sh1107I2CPoll(void)
{
    if (uptimeCounter_elapsedSince(m_lastInvert_ms) > DISPLAY_INVERT_ITVL_MS)
    {
        invertDisplay(!m_inverted);
        m_inverted = !m_inverted;
        m_lastInvert_ms = uptimeCounter_getUptimeMs();
    }
    if (uptimeCounter_elapsedSince(m_lastContrastSet_ms) >= DISPLAY_CONTRAST_ITVL_MS)
    {
//        setContrast(m_contrastIterator++);
        m_lastContrastSet_ms = uptimeCounter_getUptimeMs();
    }
}

// from datasheet
static void sh1107DatasheetInit(void)
{
    ret_code_t ret = NRF_SUCCESS;
    ret |= writeCommand(0xAE); /*display off*/
    ret |= writeCommand(0x00); /*set lower column address*/
    ret |= writeCommand(0x10); /*set higher column address*/
    ret |= writeCommand(0xB0); /*set page address*/
    ret |= writeCommand(0xdc); /*set display start line*/
    ret |= writeCommand(0x00);
    ret |= writeCommand(0x81); /*contract control*/

    ret |= writeCommand(0x6e); /*128*/
    ret |= writeCommand(0x20); /* Set Memory addressing mode (0x20/0x21) */
    ret |= writeCommand(0xA0); /*set segment remap*/
    ret |= writeCommand(0xC0); /*Com scan direction*/
    ret |= writeCommand(0xA4); /*Disable Entire Display On (0xA4/0xA5)*/
    ret |= writeCommand(0xA6); /*normal / reverse*/
    ret |= writeCommand(0xA8); /*multiplex ratio*/
    ret |= writeCommand(0x3F); /*duty = 1/64*/
    ret |= writeCommand(0xD3); /*set display offset*/
    ret |= writeCommand(0x60);
    ret |= writeCommand(0xD5); /*set osc division*/
    ret |= writeCommand(0x41);
    ret |= writeCommand(0xD9); /*set pre-charge period*/
    ret |= writeCommand(0x22);

    ret |= writeCommand(0xdb); /*set vcomh*/
    ret |= writeCommand(0x35);
    ret |= writeCommand(0xad); /*set charge pump enable*/
    ret |= writeCommand(0x8a); /*Set DC-DC enable (a=0:disable; a=1:enable) */
    ret |= writeCommand(0xAF); /*display ON*/
    if (NRF_SUCCESS == ret)
    {
        NRF_LOG_INFO("sh1107DatasheetInit done");

    }
    else
    {
        NRF_LOG_ERROR("Error initting SH1107, possibly 0x%x", ret);
    }
}

void sh1107I2C_init(void)
{
    i2c1_init();

// Init params, set it up the way we want, if we can.
    uint8_t rxByte;
    ret_code_t ret = readReg(0x00, &rxByte);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("Feather OLED not found at address 0x%x, bailing out", DEV_I2C_ADDR);
        return;
        // Don't poll if we don't exist
    }
    sh1107DatasheetInit();
    pollers_registerPoller(sh1107I2CPoll);

}
