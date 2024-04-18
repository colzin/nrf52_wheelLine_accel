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

// b7 is a don't care, it gets shifted up. This means b0 is used
#define DEV_I2C_ADDR 0x3C

/* First byte is a control byte, with special bits:
 * b7 is C0, b6 is Display/nCommand
 * Datasheet Note on Figure 9:
 * c0＝"0": The last control byte, only data bytes to follow,
 * c0＝"1": Next two bytes are a data byte and another control byte.
 * D/C ＝"0": The data byte is for command operation,
 * D/C ＝"1": The data byte is for RAM operation.
 */
#define SH110X_C0_BIT   0x80
#define SH110x_DnC_BIT  0x40

#define MONOOLED_BLACK 0   ///< Default black 'color' for monochrome OLEDS
#define MONOOLED_WHITE 1   ///< Default white 'color' for monochrome OLEDS
#define MONOOLED_INVERSE 2 ///< Default inversion command for monochrome OLEDS

/// fit into the SH110X_ naming scheme
#define SH110X_BLACK 0   ///< Draw 'off' pixels
#define SH110X_WHITE 1   ///< Draw 'on' pixels
#define SH110X_INVERSE 2 ///< Invert pixels

// COMMANDS, which must be preceeded by a 0x00 byte on I2C, then command, then maybe data byte, if applicable

/* Set Lower Column Address: (00H - 0FH)
 * Specify column address of display RAM. See datasheet for tables
 */
#define COMMAND_SETLOWCOLUMN(x)     ((uint8_t)((0x00|(x&0x0F)))) // b3:0 can be changed

/* Set Higher Column Address: (10H - 17H)
 * Specify column address of display RAM. See datasheet for tables
 */
#define COMMAND_SETHIGHCOLUMN(x)    ((uint8_t)((0x10|(x&0x0F)))) // b3:0 can be changed

/* Set Memory addressing mode (20H - 21H)
 * In page addressing mode, after the display RAM is read/ written, the column address is increased automatically by 1.
 * In vertical addressing mode, after the display RAM is read/ written, the page address is increased automatically by 1.
 * set b0 for vertical addressing mode.
 */
#define COMMAND_MEMORYMODE(x)       ((uint8_t)((0x20|(x&0x01)))) // b0 clear for page address mode, set for column address mode.

/* Set Contrast Control Register: (Double Bytes Command)
 * Next byte should be contrast, 0-255
 */
#define COMMAND_SETCONTRAST         0x81
// Next byte should be contrast, 0-255

/* Set Segment Re-map: (A0H - A1H)
 * b0 clear for normal diraction (POR default). b0 set for reverse direction up/down
 */
#define COMMAND_SETSEGMENTREMAP(x)  ((uint8_t)((0xA0|(x&0x01)))) // b0 set to remap up/down

/* Set Multiplex Ratio: (Double Bytes Command)
 * This command switches default 128 multiplex modes to any multiplex ratio from 1 to 128.
 * The output pads COM0-COM127 will be switched to corresponding common signal.
 * Next byte should be 0-127, b7 is don't care. 127 means 128, and is POR value
 */
#define COMMAND_SETMULTIPLEXRATIO   0xA8
// Next byte 0-127

/* Set Entire Display OFF/ON: (A4H - A5H)
 *  b0 low provides normal display status (POR), b1 set turns all pixels on
 */
#define COMMAND_DISPLAYALLON(x)     ((uint8_t)((0xA4|(x&0x01))))

/* Set Normal/Reverse Display: (A6H -A7H), NOTE: we call it invert display
 * b0 set to invert the normal value of each pixel (blacks turn white, etc)
 */
#define COMMAND_INVERTDISPLAY(x)    ((uint8_t)((0xA6|(x&0x01))))

/* Set Display Offset: (Double Bytes Command)
 * The next byte specifies the mapping of display start line to one of COM0-127
 */
#define COMMAND_SETDISPLAYOFFSET    0xD3

#define SH110X_PAGEADDR 0x22            ///< See datasheet

#define SH110X_CHARGEPUMP 0x8D          ///< See datasheet

#define SH110X_SETMULTIPLEX         0xA8 // See datasheet
#define SH110X_DCDC                 0xAD // See datasheet

/* Display OFF/ON: (AEH - AFH)
 * b0 set to turn display ON. b0 clear for off, default POR state
 */
#define COMMAND_DISPLAYON(x)        ((uint8_t)((0xAE|(x&0x01)))) // b1 set allows display to come on, cleared turns off

#define SH110X_SETPAGEADDR          0xB0 ///< Specify page address to load display RAM data to page address register

#define SH110X_COMSCANINC           0xC0         ///< Not currently used
#define SH110X_COMSCANDEC           0xC8         ///< See datasheet

#define SH110X_SETDISPLAYCLOCKDIV   0xD5 ///< See datasheet
#define SH110X_SETPRECHARGE         0xD9       ///< See datasheet
#define SH110X_SETCOMPINS           0xDA         ///< See datasheet
#define SH110X_SETVCOMDETECT        0xDB      ///< See datasheet
#define SH110X_SETDISPSTARTLINE     0xDC // Specify Column address to determine the initial display line or COM0.

#define SH110X_SETSTARTLINE         0x40  ///< See datasheet

#define SH1107_ID_VAL               0x07
#define SH110X_IDREG_IDVAL(x)      ((x&0x03F)) // b5:0 of ID reg are ID
#define SH110X_IDREG_BUSY           0x80 // b7 of ID reg is BUSY bit
#define SH110X_IDREG_DISPOFF        0x40 // b6 of ID reg set if display is off

// Turns pollers on or off
#define DISPLAY_INVERT_ITVL_MS      1000
#define DISPLAY_CONTRAST_ITVL_MS    100

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

static ret_code_t writeCmdByte(uint8_t byte)
{
    uint8_t toWrite[2];
    toWrite[0] = 0x00; // use two bytes, first being 0x00 to write a command (Co = 0 and D/C = 0)
    toWrite[1] = byte;
    return i2c1_writeBytes(DEV_I2C_ADDR, toWrite, sizeof(toWrite));
}

static ret_code_t writeCmd2Bytes(uint8_t command, uint8_t data)
{
    uint8_t toWrite[3];
    toWrite[0] = 0x00; // first being 0x00 to write a command (Co = 0 and D/C = 0), next command, next data
    toWrite[1] = command;
    toWrite[2] = data;
    return i2c1_writeBytes(DEV_I2C_ADDR, toWrite, sizeof(toWrite));
}

static ret_code_t readID(uint8_t* pRxBuf)
{  // set the data bit to read from RAM, clear to read ID register
// ID just repeats forever. It only makes sense to read one byte out, they are all the same
    return i2c1_readBytes(DEV_I2C_ADDR, 0x00, pRxBuf, 1);
}

static ret_code_t readRAM(uint8_t* pRxBuf, uint32_t len)
{ // set the data/nControl bit to read from RAM, clear to read ID register
    return i2c1_readBytes(DEV_I2C_ADDR, SH110x_DnC_BIT, pRxBuf, len);
}

static void setContrast(uint8_t level)
{
//    NRF_LOG_DEBUG("Setting contrast to %d", level);
    writeCmd2Bytes(COMMAND_SETCONTRAST, level);
}

#if DISPLAY_INVERT_ITVL_MS
static bool m_inverted;
#endif // #if DISPLAY_INVERT_ITVL_MS

static uint8_t m_contrastIterator;

static void sh1107I2CPoll(void)
{
#if DISPLAY_INVERT_ITVL_MS
    if (uptimeCounter_elapsedSince(m_lastInvert_ms) > DISPLAY_INVERT_ITVL_MS)
    {
//        setReMap(!m_inverted);
        writeCmdByte(COMMAND_DISPLAYON(m_inverted));
        m_inverted = !m_inverted;
        m_lastInvert_ms = uptimeCounter_getUptimeMs();
    }
#endif // #if DISPLAY_INVERT_ITVL_MS
#if DISPLAY_CONTRAST_ITVL_MS
    if (uptimeCounter_elapsedSince(m_lastContrastSet_ms) >= DISPLAY_CONTRAST_ITVL_MS)
    {
        setContrast(m_contrastIterator++);
        m_lastContrastSet_ms = uptimeCounter_getUptimeMs();
    }
#endif // #if DISPLAY_CONTRAST_ITVL_MS

}

// from datasheet
static void sh1107DatasheetInit(void)
{
    ret_code_t ret = NRF_SUCCESS;
    ret |= writeCmdByte(0xAE); /*display off*/
    ret |= writeCmdByte(0x00); /*set lower column address*/
    ret |= writeCmdByte(0x10); /*set higher column address*/
    ret |= writeCmdByte(0xB0); /*set page address*/
    ret |= writeCmdByte(0xdc); /*set display start line*/
    ret |= writeCmdByte(0x00);
    ret |= writeCmdByte(0x81); /*contract control*/

    ret |= writeCmdByte(0x6e); /*128*/
    ret |= writeCmdByte(0x20); /* Set Memory addressing mode (0x20/0x21) */
    ret |= writeCmdByte(0xA0); /*set segment remap*/
    ret |= writeCmdByte(0xC0); /*Com scan direction*/
    ret |= writeCmdByte(0xA4); /*Disable Entire Display On (0xA4/0xA5)*/
    ret |= writeCmdByte(0xA6); /*normal / reverse*/
    ret |= writeCmdByte(0xA8); /*multiplex ratio*/
    ret |= writeCmdByte(0x3F); /*duty = 1/64*/
    ret |= writeCmdByte(0xD3); /*set display offset*/
    ret |= writeCmdByte(0x60);
    ret |= writeCmdByte(0xD5); /*set osc division*/
    ret |= writeCmdByte(0x41);
    ret |= writeCmdByte(0xD9); /*set pre-charge period*/
    ret |= writeCmdByte(0x22);

    ret |= writeCmdByte(0xdb); /*set vcomh*/
    ret |= writeCmdByte(0x35);
    ret |= writeCmdByte(0xad); /*set charge pump enable*/
    ret |= writeCmdByte(0x8a); /*Set DC-DC enable (a=0:disable; a=1:enable) */
    ret |= writeCmdByte(0xAF); /*display ON*/
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
    // Read the ID, see if it's on the bus
    ret_code_t ret = readID(&rxByte);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("Feather OLED not found at address 0x%x, bailing out", DEV_I2C_ADDR);
        return;
        // Don't poll if we don't exist
    }
    if (SH1107_ID_VAL != SH110X_IDREG_IDVAL(rxByte))
    {
        NRF_LOG_WARNING("SH1107 should return 0x%x for ID, read 0x%x", SH1107_ID_VAL, SH110X_IDREG_IDVAL(rxByte));
    }
    // Read some DATA RAM
    uint8_t dataRamBytes[256];
    ret = readRAM(dataRamBytes, 256);

// TODO Zero out buffer, write to display before turning on

// Run init
    sh1107DatasheetInit();

    ret = readID(&rxByte);
    pollers_registerPoller(sh1107I2CPoll);

}
