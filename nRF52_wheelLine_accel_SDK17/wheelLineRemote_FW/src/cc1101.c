/*
 * CC1101.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Collin Moore
 */

#include "cc1101.h"

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "pollers.h"
#include "spi0.h"

#include <stdlib.h>

#include "uptimeCounter.h"
#include "version.h"

#define NRF_LOG_MODULE_NAME CC1101
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

// Configuration Registers
#define CC1101_IOCFG2       0x00        // GDO2 output pin configuration
#define CC1101_IOCFG1       0x01        // GDO1 output pin configuration
#define CC1101_IOCFG0       0x02        // GDO0 output pin configuration
#define CC1101_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1        0x04        // Sync word, high byte
#define CC1101_SYNC0        0x05        // Sync word, low byte
#define CC1101_PKTLEN       0x06        // Packet length
#define CC1101_PKTCTRL1     0x07        // Packet automation control
#define CC1101_PKTCTRL0     0x08        // Packet automation control
#define CC1101_ADDR         0x09        // Device address
#define CC1101_CHANNR       0x0A        // Channel number
#define CC1101_FSCTRL1      0x0B        // Frequency synthesizer control
#define CC1101_FSCTRL0      0x0C        // Frequency synthesizer control
#define CC1101_FREQ2        0x0D        // Frequency control word, high byte
#define CC1101_FREQ1        0x0E        // Frequency control word, middle byte
#define CC1101_FREQ0        0x0F        // Frequency control word, low byte
#define CC1101_MDMCFG4      0x10        // Modem configuration
#define CC1101_MDMCFG3      0x11        // Modem configuration
#define CC1101_MDMCFG2      0x12        // Modem configuration
#define CC1101_MDMCFG1      0x13        // Modem configuration
#define CC1101_MDMCFG0      0x14        // Modem configuration
#define CC1101_DEVIATN      0x15        // Modem deviation setting
#define CC1101_MCSM2        0x16        // Main Radio Control State Machine config
#define MCSM2_RX_TIME_RSSI                0x10
#define MCSM2_RX_TIME_QUAL                0x08
#define MCSM2_RX_TIME                     0x07
#define CC1101_MCSM1        0x17        // Main Radio Control State Machine config
#define CC1101_MCSM0        0x18        // Main Radio Control State Machine config
#define CC1101_FOCCFG       0x19        // Frequency Offset Compensation config
#define CC1101_BSCFG        0x1A        // Bit Synchronization configuration
#define CC1101_AGCCTRL2     0x1B        // AGC control
#define AGCCTRL2_MAX_DVGA_GAIN            0xC0
#define AGCCTRL2_MAX_LNA_GAIN             0x38
#define AGCCTRL2_MAGN_TARGET              0x07
#define CC1101_AGCCTRL1     0x1C        // AGC control
#define CC1101_AGCCTRL0     0x1D        // AGC control
#define CC1101_WOREVT1      0x1E        // High byte Event 0 timeout
#define CC1101_WOREVT0      0x1F        // Low byte Event 0 timeout
#define CC1101_WORCTRL      0x20        // Wake On Radio control
#define CC1101_FREND1       0x21        // Front end RX configuration
#define CC1101_FREND0       0x22        // Front end TX configuration
#define CC1101_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC1101_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC1101_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC1101_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC1101_RCCTRL1      0x27        // RC oscillator configuration
#define CC1101_RCCTRL0      0x28        // RC oscillator configuration
#define CC1101_FSTEST       0x29        // Frequency synthesizer cal control
#define CC1101_PTEST        0x2A        // Production test
#define CC1101_AGCTEST      0x2B        // AGC test
#define CC1101_TEST2        0x2C        // Various test settings
#define CC1101_TEST1        0x2D        // Various test settings
#define CC1101_TEST0        0x2E        // Various test settings

// Strobe commands
#define CC1101_STROBE_SRES      0x30        // Reset chip.
#define CC1101_STROBE_SFSTXON   0x31        // Enable/calibrate freq synthesizer
#define CC1101_STROBE_SXOFF     0x32        // Turn off crystal oscillator.
#define CC1101_STROBE_SCAL      0x33        // Calibrate freq synthesizer & disable
#define CC1101_STROBE_SRX       0x34        // Enable RX.
#define CC1101_STROBE_STX       0x35        // Enable TX.
#define CC1101_STROBE_SIDLE     0x36        // Exit RX / TX
#define CC1101_STROBE_SAFC      0x37        // AFC adjustment of freq synthesizer
#define CC1101_STROBE_SWOR      0x38        // Start automatic RX polling sequence
#define CC1101_STROBE_SPWD      0x39        // Enter pwr down mode when CSn goes hi
#define CC1101_STROBE_SFRX      0x3A        // Flush the RX FIFO buffer.
#define CC1101_STROBE_SFTX      0x3B        // Flush the TX FIFO buffer.
#define CC1101_STROBE_SWORRST   0x3C        // Reset real time clock.
#define CC1101_STROBE_SNOP      0x3D        // No operation.

// Status registers (need burst mode set)
#define CC1101_PARTNUM      0xF0        // Part number
#define CC1101_VERSION      0xF1        // Current version number
#define CC1101_FREQEST      0xF2        // Frequency offset estimate
#define CC1101_LQI          0xF3        // Demodulator estimate for link quality
#define CC1101_RSSI         0xF4        // Received signal strength indication
#define CC1101_MARCSTATE    0xF5        // Control state machine state
#define CC1101_WORTIME1     0xF6        // High byte of WOR timer
#define CC1101_WORTIME0     0xF7        // Low byte of WOR timer
#define CC1101_PKTSTATUS    0xF8        // Current GDOx status and packet status
#define CC1101_VCO_VC_DAC   0xF9        // Current setting from PLL cal module
#define CC1101_TXBYTES      0xFA        // Underflow and # of bytes in TXFIFO
#define CC1101_RXBYTES      0xFB        // Overflow and # of bytes in RXFIFO

// Other memory locations
#define CC1101_PATABLE      0x3E
#define CC1101_TXFIFO       0x3F
#define CC1101_RXFIFO       0x3F

// Masks for appended status bytes
#define CC1101_RSSI_RX        0x00        // Position of LQI byte
#define CC1101_LQI_RX         0x01        // Position of LQI byte
#define CC1101_CRC_OK         0x80        // Mask "CRC_OK" bit within LQI byte
#define CC1101_NUM_RXBYTES    0x7F        // Mask "# of bytes" field in _RXBYTES
#define CC1101_MARCSTATE_IDLE 0x01        // The status register indicates idle

// Definitions to support burst/single access:
#define CC1101_WRITE_BURST  0x40
#define CC1101_READ_SINGLE  0x80
#define CC1101_READ_BURST   0xC0

/*************************************************************************************
 *  Variables
 ************************************************************************************/

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static bool cc1101_readSingleByte(uint8_t address, uint8_t* pByte)
{
    // Just need the address byte for TX
    uint8_t buf = (address | CC1101_READ_SINGLE);
    // Send 1 byte, address
    ret_code_t ret = spi0_write(spi0_cc1101, &buf, 1, true);
    // Read 1 byte in, the byte after writing address
    ret |= spi0_read(spi0_cc1101, pByte, 1, false);
//    NRF_LOG_INFO("Wrote 0x%x, read 0x%x, data 0x%x", txBuf, rxBuf, *data);
    return ret == NRF_SUCCESS ? true : false;
}

// Command strobes are 1-byte addresses which trigger things
static bool cc1101_strobe(uint8_t address)
{
    // Send 1 byte, address to execute a task
    ret_code_t ret = spi0_write(spi0_cc1101, &address, 1, false);
    return ret == NRF_SUCCESS ? true : false;
}

static bool cc1101_writeSingleByte(uint8_t address, uint8_t data)
{
    uint8_t tx_data[2];
    tx_data[0] = address;
    tx_data[1] = data;
    ret_code_t ret = spi0_write(spi0_cc1101, tx_data, 2, false);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("spi_trans error 0x%x", ret);
    }
    return ret == NRF_SUCCESS ? true : false;
}

static bool cc1101_writeBurst(uint8_t startAddress, uint8_t* data, uint32_t len)
{
    uint8_t buf = startAddress | CC1101_WRITE_BURST;
    ret_code_t ret = spi0_write(spi0_cc1101, &buf, 1, true);
    ret |= spi0_write(spi0_cc1101, data, len, false);
    return ret == NRF_SUCCESS ? true : false;
}

static bool cc1101_readBurst(uint8_t startAddress, uint8_t* buffer, uint32_t len)
{
    uint8_t buf = startAddress | CC1101_READ_BURST;
    ret_code_t ret = spi0_write(spi0_cc1101, &buf, 1, true);
    ret |= spi0_read(spi0_cc1101, buffer, len, false);
    return ret == NRF_SUCCESS ? true : false;
}

bool cc1101_selfTest(void)
{
    uint8_t partNum;
    uint8_t version;
    bool ret;
    // Wait for it to boot. Datasheet says max of
    spi0_init(); // Make sure pins are muxed and working
    uint32_t retries = 100;
    uint32_t ms_timestamp = uptimeCounter_getUptimeMs();
    while (retries)
    { //  drive CS low, see if it responds by driving MISO low.
        NRF_P0->OUTCLR = (1UL << SPI0_CC1101_CS_GPIO);
        nrf_delay_us(10); // Wait for it to pull line low
        if (0 == ((NRF_P0->IN >> SPI0_MISO_PIN) & 1UL))
        { // If MISO is low, then CC1101 is ready.
            NRF_LOG_INFO("CC1101 pulled MISO low, ready");
            break;
        }
        NRF_P0->OUTSET = (1UL << SPI0_CC1101_CS_GPIO);
        // Else wait and try again
        nrf_delay_ms(1);
        retries--;
    }
    if (!retries)
    {
        NRF_LOG_ERROR("Did not detect CC1101, retries exceeded");
        return false;
    }
    else
    {
        NRF_LOG_INFO("Detected CC1101 ready after %d ms, %d retries left", uptimeCounter_elapsedSince(ms_timestamp),
                     retries);
    }
    NRF_LOG_INFO("Starting CC1101 self test:");
    // Issue software reset
    ret = cc1101_strobe(CC1101_STROBE_SRES);
    if (!ret)
    {
        NRF_LOG_WARNING("Failed to send strobe command");
    }
    // Wait for it to reboot. Datasheet says max of
    retries = 100;
    ms_timestamp = uptimeCounter_getUptimeMs();
    while (retries)
    { //  drive CS low, see if it responds by driving MISO low.
        NRF_P0->OUTCLR = (1UL << SPI0_CC1101_CS_GPIO);
        nrf_delay_us(10); // Wait for it to pull line low
        if (0 == ((NRF_P0->IN >> SPI0_MISO_PIN) & 1UL))
        { // If MISO is low, then CC1101 is ready.
            NRF_LOG_INFO("CC1101 pulled MISO low, ready");
            break;
        }
        NRF_P0->OUTSET = (1UL << SPI0_CC1101_CS_GPIO);
        // Else wait and try again
        nrf_delay_ms(1);
        retries--;
    }
    if (!retries)
    {
        NRF_LOG_ERROR("Did not detect CC1101, retries exceeded");
        return false;
    }
    else
    {
        NRF_LOG_INFO("Detected CC1101 ready after %d ms, %d retries left", uptimeCounter_elapsedSince(ms_timestamp),
                     retries);
    }
    // Read partNum and
    ret = cc1101_readSingleByte(CC1101_PARTNUM, &partNum);
    if (ret != true)
    {
        NRF_LOG_ERROR("Error reading PARTNUM");
        return ret;
    }
    ret = cc1101_readSingleByte(CC1101_VERSION, &version);
    if (ret != true)
    {
        NRF_LOG_ERROR("Error reading VERSION");
        return ret;
    }
    // CC1101 datasheet says partnum should be 0
    if (0 != partNum)
    {
        NRF_LOG_ERROR("Partnum is %d, expected 0", partNum);
        return false;
    }
    // Versions in the past have been 20 and 4. Datasheet says subject to change without notice.
    if (0 == version)
    {
        NRF_LOG_ERROR("Version was 0, is SPI set up right?");
        return false;
    }
    if (4 != version)
    {
        NRF_LOG_WARNING("Version is %d, expected %d", version, 4);
        // Continue
    }
    NRF_LOG_INFO("Self Test Passed");
    return true;

}

void cc1101_sidle()
{
    uint8_t marcState;

    cc1101_strobe(CC1101_STROBE_SIDLE);
    cc1101_readSingleByte(CC1101_MARCSTATE, &marcState);
    while (marcState != 0x01)
    {
        nrf_delay_us(100);
        cc1101_readSingleByte(CC1101_MARCSTATE, &marcState);
    }
    cc1101_strobe(CC1101_STROBE_SFTX);
    nrf_delay_us(100);

}

void cc1101_flushRxFifo(void)
{
    cc1101_strobe(CC1101_STROBE_SFRX);
    nrf_delay_us(2);
}

void cc1101_flushTxFifo(void)
{
    cc1101_strobe(CC1101_STROBE_SFTX);
    nrf_delay_us(2);
}

bool cc1101_setTxState(void)
{
    bool ret = cc1101_strobe(CC1101_STROBE_STX);
    if (ret)
    {
        nrf_delay_us(2);
    }
    return ret;
}

bool cc1101_setRxState(void)
{
    bool ret = cc1101_strobe(CC1101_STROBE_SRX);
    if (ret)
    {
        nrf_delay_us(2);
    }
    return ret;
}

// From TI studio Nov 13, 2023 for 3kBaud at 433MHz ASK
void cc1101_initASKTx_myStudio(void)
{
    cc1101_writeSingleByte(CC1101_IOCFG2, 0x29);
    cc1101_writeSingleByte(CC1101_IOCFG1, 0x2e);
//    cc1101_writeSingleByte(CC1101_IOCFG0, 0x06);
    cc1101_writeSingleByte(CC1101_IOCFG0, 0x3F);
    cc1101_writeSingleByte(CC1101_FIFOTHR, 0x07); // TODO is this what we want?
    cc1101_writeSingleByte(CC1101_SYNC1, 0xd3);
    cc1101_writeSingleByte(CC1101_SYNC0, 0x91);
//    cc1101_writeSingleByte(CC1101_PKTLEN, 0xff);
    cc1101_writeSingleByte(CC1101_PKTLEN, 0x10);
    cc1101_writeSingleByte(CC1101_PKTCTRL1, 0x04);
    cc1101_writeSingleByte(CC1101_PKTCTRL0, 0x05);
    cc1101_writeSingleByte(CC1101_PKTCTRL0, 0x04);
    cc1101_writeSingleByte(CC1101_ADDR, 0x00);
    cc1101_writeSingleByte(CC1101_CHANNR, 0x00);
//    cc1101_writeSingleByte(CC1101_FSCTRL1, 0x06);
    cc1101_writeSingleByte(CC1101_FSCTRL1, 0x0F);
    cc1101_writeSingleByte(CC1101_FSCTRL0, 0x00);
    cc1101_writeSingleByte(CC1101_FREQ2, 0x10);
//    cc1101_writeSingleByte(CC1101_FREQ1, 0xA7);
    cc1101_writeSingleByte(CC1101_FREQ1, 0xA8);
//    cc1101_writeSingleByte(CC1101_FREQ0, 0x62);
    cc1101_writeSingleByte(CC1101_FREQ0, 0x5E);
//    cc1101_writeSingleByte(CC1101_MDMCFG4, 0xF6);
    cc1101_writeSingleByte(CC1101_MDMCFG4, 0x86);
    cc1101_writeSingleByte(CC1101_MDMCFG3, 0xE4);
//    cc1101_writeSingleByte(CC1101_MDMCFG2, 0x37);
    cc1101_writeSingleByte(CC1101_MDMCFG2, 0x30);
//    cc1101_writeSingleByte(CC1101_MDMCFG1, 0x00);
    cc1101_writeSingleByte(CC1101_MDMCFG1, 0x02);
    cc1101_writeSingleByte(CC1101_MDMCFG0, 0xF8);
    cc1101_writeSingleByte(CC1101_DEVIATN, 0x47);
    cc1101_writeSingleByte(CC1101_MCSM2, 0x07);
    cc1101_writeSingleByte(CC1101_MCSM1, 0x30);
//    cc1101_writeSingleByte(CC1101_MCSM0, 0x18);
    cc1101_writeSingleByte(CC1101_MCSM0, 0x04);
//    cc1101_writeSingleByte(CC1101_FOCCFG, 0x16);
    cc1101_writeSingleByte(CC1101_FOCCFG, 0x36);
    cc1101_writeSingleByte(CC1101_BSCFG, 0x6C);
    cc1101_writeSingleByte(CC1101_AGCCTRL2, 0x03);
    cc1101_writeSingleByte(CC1101_AGCCTRL1, 0x40);
    cc1101_writeSingleByte(CC1101_AGCCTRL0, 0x91);
    cc1101_writeSingleByte(CC1101_WOREVT1, 0x87);
    cc1101_writeSingleByte(CC1101_WOREVT0, 0x6B);
    cc1101_writeSingleByte(CC1101_WORCTRL, 0xF8);
    cc1101_writeSingleByte(CC1101_FREND1, 0x56);
    cc1101_writeSingleByte(CC1101_FREND0, 0x11);
    cc1101_writeSingleByte(CC1101_FSCAL3, 0xE9);
    cc1101_writeSingleByte(CC1101_FSCAL2, 0x2A);
    cc1101_writeSingleByte(CC1101_FSCAL1, 0x00);
    cc1101_writeSingleByte(CC1101_FSCAL0, 0x1F);
    cc1101_writeSingleByte(CC1101_RCCTRL1, 0x41);
    cc1101_writeSingleByte(CC1101_RCCTRL0, 0x00);
    cc1101_writeSingleByte(CC1101_FSTEST, 0x59);
    cc1101_writeSingleByte(CC1101_PTEST, 0x7F);
    cc1101_writeSingleByte(CC1101_AGCTEST, 0x3F);
//    cc1101_writeSingleByte(CC1101_TEST2, 0x81);
    cc1101_writeSingleByte(CC1101_TEST2, 0x88);
//    cc1101_writeSingleByte(CC1101_TEST1, 0x35);
    cc1101_writeSingleByte(CC1101_TEST1, 0x31);
    cc1101_writeSingleByte(CC1101_TEST0, 0x09);
    uint8_t paTableBytes[8] = { 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    cc1101_writeBurst(CC1101_PATABLE, paTableBytes, 8);

    // TODO customize IO pins
//    cc1101_writeSingleByte(CC1101_IOCFG2, 0x0B);     //serial clock.synchronous to the data in synchronous serial mode
//        cc1101_writeSingleByte(CC1101_IOCFG0, 0x06); //asserts when sync word has been sent/received, and de-asserts at the end of the packet

    cc1101_flushTxFifo();
    cc1101_flushRxFifo();
    cc1101_sidle();

}

pkt_format_t cc1101_getPacketConfigurationMode()
{
    uint8_t configMode = 0;
    cc1101_readSingleByte(CC1101_PKTCTRL0, &configMode);
    configMode &= 0x03; // Mask all but the config mode
    return (pkt_format_t)configMode;
}

uint8_t* cc1101_tryReceiveData(uint32_t* pNumBytes)
{
    uint8_t rx_bytes_val;
    uint8_t* pData = NULL;
    *pNumBytes = 0;

    cc1101_readSingleByte(CC1101_RXBYTES, &rx_bytes_val);

    bool overflow = rx_bytes_val & 0x80 ? true : false;

    if (true == overflow)
    {
        NRF_LOG_WARNING("Rx OVERFLOW!");
        cc1101_flushRxFifo();
        cc1101_sidle();
        return pData;
    }

    rx_bytes_val &= 0x7F;

    if (rx_bytes_val != 0)
    {
        NRF_LOG_INFO("Received %d bytes!", rx_bytes_val);
        pkt_format_t sending_mode = cc1101_getPacketConfigurationMode();
        uint8_t data_len;
        cc1101_readSingleByte(CC1101_PKTLEN, &data_len);
        if (sending_mode == PKT_LEN_FIXED && rx_bytes_val >= data_len)
        {
            // Do nothing here
        }
        else if (sending_mode == PKT_LEN_VARIABLE)
        {
            uint8_t max_len;
            cc1101_readSingleByte(CC1101_PKTLEN, &max_len);
            cc1101_readSingleByte(CC1101_RXFIFO, &data_len);
            if (data_len > max_len)
            {
                NRF_LOG_WARNING("Len of data exceeds the configured maximum packet len");
                return pData;
            }
        }
        else if (sending_mode == PKT_LEN_INFINITE && rx_bytes_val != 0)
        {
            nrf_delay_us(1000); // Sleep for 1 msec
            cc1101_readSingleByte(CC1101_RXBYTES, &data_len);
        }
        else
        {
            NRF_LOG_WARNING("else case");
            return pData;
        }

        // Allocate memory to read the data into, and read it.
        pData = malloc(data_len);
        if (NULL == pData)
        {
            NRF_LOG_ERROR("Malloc failed!");
            return pData;
        }
        *pNumBytes = data_len;
        cc1101_readBurst(CC1101_RXFIFO, pData, data_len);

        uint8_t pktctrl_1;
        cc1101_readSingleByte(CC1101_PKTCTRL1, &pktctrl_1);
        if (pktctrl_1 & 0x04) // If bit3 is set
        {
            uint8_t rssi;
            uint8_t val;
            cc1101_readSingleByte(CC1101_RXFIFO, &rssi);
            cc1101_readSingleByte(CC1101_RXFIFO, &val);
        }
        cc1101_flushRxFifo();
        cc1101_sidle();
        return pData;
    }
    return pData;
}

static void cc1101Poll(void)
{
    uint32_t numBytesReceived;
    uint8_t* pData = cc1101_tryReceiveData(&numBytesReceived);
    if (NULL != pData)
    {
        NRF_LOG_INFO("CC1101 received %d bytes ", numBytesReceived);
        free(pData);
    }
}

void cc1101_init(void)
{
    bool testPass = cc1101_selfTest();
    if (!testPass)
    {
        NRF_LOG_ERROR("Couldn't init CC1101");
        return;
    }
    // If here, init it
    cc1101_initASKTx_myStudio();
    pollers_registerPoller(cc1101Poll);
    // Start receiving
    cc1101_setRxState();
}
