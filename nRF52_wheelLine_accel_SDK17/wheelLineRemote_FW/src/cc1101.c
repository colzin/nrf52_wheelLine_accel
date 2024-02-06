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

#define TX_TEST_ITVL_MS 3000 // non-zero to run TX test poll
#define TEST_PKT_LEN 60

#define RX_POLL_ITVL_MS 900 // Datasheet says not to spam SPI with traffic

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static uint8_t m_lastStatus;

#if TX_TEST_ITVL_MS
static uint32_t m_lastTx_ms;
#endif // #if TX_TEST_ITVL_MS

static uint32_t m_lastRxPoll_ms;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static void updateStatus(uint8_t newVal)
{

    if (m_lastStatus != newVal)
    {
        NRF_LOG_INFO("Status from 0x%x to 0x%x", m_lastStatus, newVal);
        m_lastStatus = newVal;
    }
}

static bool cc1101_readSingleByte(uint8_t address, uint8_t* pByte)
{
    // Just need the address byte for TX
    uint8_t txByte = (address | CC1101_READ_SINGLE);
    uint8_t rxBytes[2];
    // Send 1 byte, address
    nrfx_spi_xfer_desc_t xfer;
    xfer.p_rx_buffer = rxBytes;
    xfer.p_tx_buffer = &txByte;
    xfer.rx_length = 2;
    xfer.tx_length = 1;
    ret_code_t ret = spi0_xfer(spi0_cc1101, &xfer, false);
    updateStatus(rxBytes[0]);
//    NRF_LOG_INFO("Wrote 0x%x, read Status 0x%x, byte 0x%x", txByte, rxBytes[0], rxBytes[1]);
    *pByte = rxBytes[1];
    return ret == NRF_SUCCESS ? true : false;
}

// Command strobes are 1-byte addresses which trigger things
static bool cc1101_strobe(uint8_t address)
{
    // Send 1 byte, address to execute a task, read back the status reg
    uint8_t sres;
    nrfx_spi_xfer_desc_t xfer;
    xfer.p_rx_buffer = &sres;
    xfer.p_tx_buffer = &address;
    xfer.rx_length = 1;
    xfer.tx_length = 1;
    ret_code_t ret = spi0_xfer(spi0_cc1101, &xfer, false);
    updateStatus(sres);
    NRF_LOG_INFO("Strobed 0x%x, read Status 0x%x", address, sres);
    return ret == NRF_SUCCESS ? true : false;
}

static bool cc1101_writeSingleByte(uint8_t address, uint8_t data)
{
    uint8_t tx_data[2];
    tx_data[0] = address;
    tx_data[1] = data;
    uint8_t sres;
    nrfx_spi_xfer_desc_t xfer;
    xfer.p_rx_buffer = &sres;
    xfer.p_tx_buffer = tx_data;
    xfer.rx_length = 1;
    xfer.tx_length = 2;
    ret_code_t ret = spi0_xfer(spi0_cc1101, &xfer, false);
    updateStatus(sres);
//    NRF_LOG_INFO("Wrote address 0x%x to 0x%x, read Status 0x%x", address, data, sres);
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

static bool cc1101_reboot(void)
{
    // Wait for it to boot. It will drive MISO low when we drive CS low, if it's ready
    uint32_t retries = 100;
//    uint32_t ms_timestamp = uptimeCounter_getUptimeMs();
    while (retries)
    { //  drive CS low, see if it responds by driving MISO low. Can do this by reading status byte
        cc1101_strobe(CC1101_STROBE_SNOP);
        if (m_lastStatus & 0x80)
        {
            // Else wait and try again
            nrf_delay_ms(1);
            retries--;
        }
        else
        {
//            NRF_LOG_INFO("Detected CC1101 ready after %d ms, %d retries left", uptimeCounter_elapsedSince(ms_timestamp),
//                         retries);
            break;
        }
    }
    if (!retries)
    {
        NRF_LOG_ERROR("Did not detect CC1101, retries exceeded");
        return false;
    }
//    NRF_LOG_INFO("Checking for CC1101 to be rebooted:");
    // Issue software reset
    bool ret = cc1101_strobe(CC1101_STROBE_SRES);
    if (!ret)
    {
        NRF_LOG_WARNING("Failed to send strobe command");
    }
    // Wait for it to reboot. Datasheet says max of
    retries = 100;
//    ms_timestamp = uptimeCounter_getUptimeMs();
    while (retries)
    { //  drive CS low, see if it responds by driving MISO low. Can do this by reading status byte
        cc1101_strobe(CC1101_STROBE_SNOP);
        if (m_lastStatus & 0x80)
        {
            // Else wait and try again
            nrf_delay_ms(1);
            retries--;
        }
        else
        {
//            NRF_LOG_INFO("Detected CC1101 ready after %d ms, %d retries left", uptimeCounter_elapsedSince(ms_timestamp),
//                         retries);
            break;
        }
    }
    if (!retries)
    {
        NRF_LOG_ERROR("Did not detect CC1101, retries exceeded");
        return false;
    }
    return true;
}

bool cc1101_selfTest(void)
{
    uint8_t partNum;
    uint8_t version;
    bool ret;

    // Read partNum and version registers
    ret = cc1101_readSingleByte(CC1101_PARTNUM, &partNum);
    if (ret != true)
    {
        NRF_LOG_ERROR("Error reading PARTNUM");
        return false;
    }
    // CC1101 datasheet says partnum should be 0
    if (0 != partNum)
    {
        NRF_LOG_ERROR("Partnum is %d, expected 0", partNum);
        return false;
    }
    ret = cc1101_readSingleByte(CC1101_VERSION, &version);
    if (ret != true)
    {
        NRF_LOG_ERROR("Error reading VERSION");
        return ret;
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
    NRF_LOG_INFO("CC1101: Self Test Passed. Detected partnum %d, version %d, status 0x%x", partNum, version,
                 m_lastStatus);
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

static void setupFeb6(void)
{
    if (m_lastStatus & 0x70)
    {
        NRF_LOG_DEBUG("CC1101 in state %d, not idle, go to idle:", m_lastStatus >> 4);
        cc1101_sidle(); // Idle it before writing configs
    }

    /* Rf settings for CC1101, exported Feb 06, 2024
     *432.99MHz, XTAL 26MHz
     */
    cc1101_writeSingleByte(CC1101_IOCFG0, 0x06);  //GDO0 Output Pin Configuration
    cc1101_writeSingleByte(CC1101_FIFOTHR, 0x47); //RX FIFO and TX FIFO Thresholds
    cc1101_writeSingleByte(CC1101_PKTCTRL0, 0x05); //Packet Automation Control
    cc1101_writeSingleByte(CC1101_FSCTRL1, 0x06); //Frequency Synthesizer Control
    cc1101_writeSingleByte(CC1101_FREQ2, 0x10);   //Frequency Control Word, High Byte
    cc1101_writeSingleByte(CC1101_FREQ1, 0xA7);   //Frequency Control Word, Middle Byte
    cc1101_writeSingleByte(CC1101_FREQ0, 0x62);   //Frequency Control Word, Low Byte
    cc1101_writeSingleByte(CC1101_MDMCFG4, 0xF5); //Modem Configuration
    cc1101_writeSingleByte(CC1101_MDMCFG3, 0x83); //Modem Configuration
    cc1101_writeSingleByte(CC1101_MDMCFG2, 0x33); //Modem Configuration
    cc1101_writeSingleByte(CC1101_DEVIATN, 0x15); //Modem Deviation Setting
    cc1101_writeSingleByte(CC1101_MCSM0, 0x18);   //Main Radio Control State Machine Configuration
    cc1101_writeSingleByte(CC1101_FOCCFG, 0x14);  //Frequency Offset Compensation Configuration
    cc1101_writeSingleByte(CC1101_AGCCTRL0, 0x92);  //AGC Control
    cc1101_writeSingleByte(CC1101_WORCTRL, 0xFB); //Wake On Radio Control
    cc1101_writeSingleByte(CC1101_FREND0, 0x11);  //Front End TX Configuration
    cc1101_writeSingleByte(CC1101_FSCAL3, 0xE9);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(CC1101_FSCAL2, 0x2A);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(CC1101_FSCAL1, 0x00);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(CC1101_FSCAL0, 0x1F);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(CC1101_TEST2, 0x81);   //Various Test Settings
    cc1101_writeSingleByte(CC1101_TEST1, 0x35);   //Various Test Settings
    cc1101_writeSingleByte(CC1101_TEST0, 0x09);   //Various Test Settings
    uint8_t paTableBytes[8] = { 0x00, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    cc1101_writeBurst(CC1101_PATABLE, paTableBytes, 8);

    cc1101_writeSingleByte(CC1101_PKTLEN, TEST_PKT_LEN);
    NRF_LOG_WARNING("CC1101 feb 6 COMPLETE");
}

/* Packet sniffer settings Feb 06 2024
 # ---------------------------------------------------
 # Packet sniffer stttings for CC1101
 # ---------------------------------------------------
 IOCFG0   |0x0002|0x06|GDO0 Output Pin Configuration
 FIFOTHR  |0x0003|0x47|RX FIFO and TX FIFO Thresholds
 PKTCTRL0 |0x0008|0x05|Packet Automation Control
 FSCTRL1  |0x000B|0x06|Frequency Synthesizer Control
 FREQ2    |0x000D|0x10|Frequency Control Word, High Byte
 FREQ1    |0x000E|0xA7|Frequency Control Word, Middle Byte
 FREQ0    |0x000F|0x62|Frequency Control Word, Low Byte
 MDMCFG4  |0x0010|0xF5|Modem Configuration
 MDMCFG3  |0x0011|0x83|Modem Configuration
 MDMCFG2  |0x0012|0x33|Modem Configuration
 DEVIATN  |0x0015|0x15|Modem Deviation Setting
 MCSM0    |0x0018|0x18|Main Radio Control State Machine Configuration
 FOCCFG   |0x0019|0x14|Frequency Offset Compensation Configuration
 AGCCTRL0 |0x001D|0x92|AGC Control
 WORCTRL  |0x0020|0xFB|Wake On Radio Control
 FREND0   |0x0022|0x11|Front End TX Configuration
 FSCAL3   |0x0023|0xE9|Frequency Synthesizer Calibration
 FSCAL2   |0x0024|0x2A|Frequency Synthesizer Calibration
 FSCAL1   |0x0025|0x00|Frequency Synthesizer Calibration
 FSCAL0   |0x0026|0x1F|Frequency Synthesizer Calibration
 TEST2    |0x002C|0x81|Various Test Settings
 TEST1    |0x002D|0x35|Various Test Settings
 TEST0    |0x002E|0x09|Various Test Settings
 */

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

static void sendTestPacket(void)
{

    uint8_t bytes[TEST_PKT_LEN];
    for (uint32_t i = 0; i < TEST_PKT_LEN; i++)
    {
        bytes[i] = (uint8_t)i;
    }

    cc1101_writeBurst(CC1101_TXFIFO, bytes, TEST_PKT_LEN);
    cc1101_readSingleByte(CC1101_TXBYTES, bytes);
    NRF_LOG_DEBUG("Wrote %d bytes to txfifo, TXYBTES says %d, status 0x%x", TEST_PKT_LEN, bytes[0], m_lastStatus);
    cc1101_strobe(CC1101_STROBE_STX);
}

static void cc1101Poll(void)
{
    if (uptimeCounter_elapsedSince(m_lastRxPoll_ms) >= RX_POLL_ITVL_MS)
    {
        cc1101_strobe(CC1101_STROBE_SNOP);
        if (0x20 == (m_lastStatus & 0xF0))
        {
            if (uptimeCounter_elapsedSince(m_lastTx_ms) > 1000)
            {
                NRF_LOG_WARNING("Switching from TX to RX state");
                cc1101_setRxState();
            }
        }
        if (0x10 == (m_lastStatus & 0xF0))
        {
            NRF_LOG_DEBUG("Check for RX bytes");
            uint32_t numBytesReceived;
            uint8_t* pData = cc1101_tryReceiveData(&numBytesReceived);
            if (NULL != pData)
            {
                NRF_LOG_INFO("CC1101 received %d bytes ", numBytesReceived);
                free(pData);
            }
        }
        m_lastRxPoll_ms = uptimeCounter_getUptimeMs();
    }

#if TX_TEST_ITVL_MS
    if (uptimeCounter_elapsedSince(m_lastTx_ms) >= TX_TEST_ITVL_MS)
    {
        // TODO run a TX packet
        sendTestPacket();
        m_lastTx_ms = uptimeCounter_getUptimeMs();
    }
#endif // #if TX_TEST_ITVL_MS
}

void cc1101_init(void)
{
    spi0_init(); // Make sure pins are muxed and working
    m_lastStatus = 0xFF;
    bool testPass = cc1101_reboot();
    if (!testPass)
    {
        NRF_LOG_ERROR("Couldn't reboot CC1101");
        return;
    }
    testPass = cc1101_selfTest();
    if (!testPass)
    {
        NRF_LOG_ERROR("CC1101 self-test failed");
        return;
    }
    // If here, init it
//    cc1101_initASKTx_myStudio();
    setupFeb6(); // TODO figure out optimal parameters

    pollers_registerPoller(cc1101Poll);
// Start receiving
    cc1101_setRxState();
}
