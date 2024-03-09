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

// Chip status byte comes back on every SPI transaction's first byte
// b7 clear if ready
#define CHIPSTATUSBYTE_STATE(x)    ((x>>4)&0x07) // b6:4 state[2:0]
// In h file for users to poll.
#define CHIPSTATUSBYTE_FIFOBYTES_AVAIL(x) (x&0x0F) // b3:0 bytes available

// Configuration Registers
#define IOCFG2_REGADDR          0x00        // GDO2 output pin configuration
// b7 not used, reads 0
#define IOCFG2_GDO2_INV         0x40 // b6 inverts the GDO output, to active low, if set.
// b65:0 are GDO2_CFG, see macro below, which is the same for all 3 pins.
// Note that the default is CHP_RDYn on GDO2

#define IOCFG1_REGADDR          0x01        // GDO1 output pin configuration
#define IOCFG1_GDO_HIGHDRIVE    0x80 // b7 sets drive strength of all GDO pins, high if set. Default low, zero
#define IOCFG1_GDO1_INV         0x40  // b6 inverts the GDO output, to active low, if set.
// b65:0 are GDO1_CFG, see macro below, which is the same for all 3 pins.
// Note that the default is 3-state (hi-z) on GDO1

#define IOCFG0_REGADDR          0x02        // GDO0 output pin configuration
// b7 enables analog temp sensor. Must write all other bits in IOCFG to 0 to use temp sensor.
#define IOCFG0_TEMP_SENSOR_ENABLE   0x80
#define IOCFG0_GDO0_INV         0x40 // b6 inverts the GDO output, to active low, if set.
// b65:0 are GDO1_CFG, see macro below, which is the same for all 3 pins.
// Note that the default is clk_xosc/192 on GDO0. NOTE: disable this in RX

#define IOCFGx_GDOx_CFG(x)      (x&0x3F) // See table 41 on datasheet for functions
typedef enum
{
    gdox_rxfifoThreshold = 0,
    gdox_rxThresholdOrEndOfPkt,
    gdox_txFifoThreshold,
    gdox_txFifoFull,
    gdox_rxFifoOverflow,
    gdox_txFifoUnderflow,
    gdox_syncwordToEndOfPkt,
    gdox_rxPktCRCok,
    gdox_preambleQualityReached,
    gdox_channelIsClear,
    gdox_lockDetectorOutput,
    gdox_serialClock,
    gdox_syncSerialDataOutput,
    gdox_asyncSerialDataOutput,
    gdox_carrierSense,
    gdox_CRC_OK,
    gdox_rsvd1, // 16 to 21 reserved
    gdox_rsvd2,
    gdox_rsvd3,
    gdox_rsvd4,
    gdox_rsvd5,
    gdox_rsvd6,
    gdox_rx_hard_data_1,
    gdox_rx_hard_data_0,
    gdox_rsvd7,
    gdox_rsvd8,
    gdox_rsvd9, // 24 to 26 reserved
    gdox_PA_PD,
    gdox_LNA_PD,
    gdox_RX_SYMBOL_TICK,
    gdox_rsvd10, // 30 to 35 reserved
    gdox_rsvd11,
    gdox_rsvd12,
    gdox_rsvd13,
    gdox_rsvd14,
    gdox_rsvd15,
    gdox_WOR_EVT0,
    gdox_WOR_EVT1,
    gdox_CLK_256,
    gdox_CLK_32K,
    gdox_rsvd16,
    gdox_CHIP_RDYn,
    gdox_rsvd17,
    gdox_XOSC_STABLE,
    gdox_rsvd18,
    gdox_rsvd19,
    gdox_3state, // high-impedance
    gdox_HW_to_0, // to control an external LNA/PA or switch
    gdox_clk_xoscDiv1, // Note: Only one GDO pin can use any of these clk_xosc functions at a time.
    gdox_clk_xoscDiv1_5, // Also note: Please disable these oscillator functions when running RX or TX
    gdox_clk_xoscDiv2,
    gdox_clk_xoscDiv3,
    gdox_clk_xoscDiv4,
    gdox_clk_xoscDiv6,
    gdox_clk_xoscDiv8,
    gdox_clk_xoscDiv12,
    gdox_clk_xoscDiv16,
    gdox_clk_xoscDiv24,
    gdox_clk_xoscDiv32,
    gdox_clk_xoscDiv48,
    gdox_clk_xoscDiv64,
    gdox_clk_xoscDiv96,
    gdox_clk_xoscDiv128,
    gdox_clk_xoscDiv192
} iocfgx_gdox_cfg_t;

#define FIFOTHR_REGADDR         0x03        // RX FIFO and TX FIFO thresholds
// b7 unused, reads 0
#define FIFOTHR_ADC_RETENTION   0x40 // b6 ADC retention, see datasheet
#define FIFOTHR_CLOSE_IN_RX(x)  ((x<<4)&0x30)// b5:4 See DesignNote DN010, attenuation values
typedef enum
{
    closeInRx_0dB = 0,
    closeInRx_6dB,
    closeInRx_12dB,
    closeInRx_18dB
} fifothr_closeInRx_t;
#define FIFOTHR_FIFO_THR(x)     (x&0x07) // b3:0 are the threshold for RX and TX FIFO, exceeded when num bytes => threshold

#define SYNC1_REGADDR           0x04            // Sync word, high byte
// b7:0 are MSByte of 16-bit sync word
#define SYNC0_REGADDR           0x05            // Sync word, low byte
// b7:0 are LSByte of 16-bit sync word

#define PKTLEN_REGADDR          0x06            // Packet length, default 0xFF, don't write 0
// b7:0 indicate packet length in fixed packet length, max length in variable packet length mode

#define PKTCTRL1_REGADDR        0x07            // Packet automation control[1]
#define PKTCTRL1_PQT(x)         ((x<<5)&0xE0)   // b7:5 PQT[2:0]. 0 accepts all
// b4 unused, reads 0
#define PKTCTRL1_CRC_AUTOFLUSH  0x08 // b3 set: flushes RX FIFO if CRC not OK.
#define PKTCTRL1_APPEND_STATUS  0x04 // b2 set: Appends two status bytes onto the payload in RX
#define PKTCTRL1_ADDR_CHK(x)    (x&0x03) // b1:0 addr_chk mode
typedef enum
{
    addrChk_none = 0,
    addrChk_checkNoBroadcast,
    addrChk_checkAndZero,
    addrChk_checkAndZeroAndFF,
} pktCtrl1_addrChk_t;

#define PKTCTRL0_REGADDR        0x08            // Packet automation control[0]
// b7 unused, reads 0
#define PKTCTRL0_WHITE_DATA     0x40            // b6 set to turn data whitening on
// Commenting the below to get confused with fixed vs variable length packets
//#define PKTCTRL0_PKT_FORMAT_WRITE(x)  ((x<<4)&0x30)   // b5:4
//#define PKTCTRL0_PKT_FORMAT_READ(x)  ((pktCtrl0_pktFmt_t)((x>>4)&0x03))   // b5:4 shift down to 1:0
//typedef enum
//{
//    pktFormat_normal = 0,
//    pktFormat_syncSerial,
//    pktFormat_randomTX,
//    pktFormat_asyncSerial
//} pktCtrl0_pktFmt_t;
// b3 not used, reads 0
#define PKTCTRL0_CRC_EN         0x04            // b2 set turns on CRC calculation in RX and TX
#define PKTCTRL0_LENGTH_CFG(x)  (x&0x03)        // b1:0 packet length config
typedef enum
{
    pktLen_fixed = 0,
    pktLen_variable,
    pktLen_infinite,
    pktLen_reserved
} pktCtrl0_lenCfg_t;

#define ADDR_REGADDR            0x09        // Device address for Rx filtration.
// b7:0 are address to filter. Optional broadcast addresses are 0x00(default) and 0xFF

#define CHANNR_REGADDR          0x0A        // Channel number, default 0
// b7:0 are channel number, unsigned. Multiplied by channel spacing and added to base frequency

#define FSCTRL1_REGADDR         0x0B        // Frequency synthesizer control
// b7:6 not used, read 0
// b5 reserved, keep at 0
// b4:0 FREQ_IF, defaults to 0x0F
#define FSCTRL1_FREQ_IF(x)      (x&0x0F) // Desired IF frequency in RX

#define FSCTRL0_REGADDR         0x0C        // Frequency synthesizer control
// b7:0 are freqoffset, frequency offset added to base frequency. 2s-complement. Default 0

#define FREQ2_REGADDR           0x0D        // Frequency control word, high byte
// b7:6 are FREQ[23:22], default 0, always 0 with 26 or 27MHz crystal
// b5:0 are FREQ[21:16], base frequency for freq synth. Defaults to 0x1E
#define FREQ1_REGADDR           0x0E        // Frequency control word, middle byte
// b7:0 are FREQ[15:8], default 0xC4
#define FREQ0_REGADDR           0x0F        // Frequency control word, low byte
// b7:0 are FREQ[7:0], default 0xEC. So FREQ[23:0] default is 0x01EC4EC

#define MDMCFG4_REGADDR         0x10        // Modem configuration
// See datasheet
#define MDMCFG3_REGADDR         0x11        // Modem configuration
// See datasheet
#define MDMCFG2_REGADDR         0x12        // Modem configuration
#define MDMCFG2_DEM_DCFILT_OFF  0x08 // b7 disabled if 1
#define MDMCFG2_MOD_FORMAT(x)   ((x<<4)&0x7) // b6:4 modulation format
typedef enum
{
    modFormat_2FSK = 0,
    modFormat_GFSK,
    modFormat_rsvd1,
    modFormat_ASK_OOK,
    modFormat_4FSK,
    modFormat_rsvd2,
    modFormat_rsvd3,
    modFormat_MSK // MSK only supported for data rates above 26kBaud
} mdmcfg2_modFormat_t;
#define MDMCFG2_MANCHESTER_EN   0x08 // Manchester encoding enabled if set
#define MDMCFG2_SYNC_MODE(x)    (x&0x07) // 2:0 are sync mode, combo of sync word qualifier mode
typedef enum
{
    syncMode_noPreamble_sync = 0,
    syncMode_15of16BitsDetected,
    syncMode_16of16BitsDetected,
    syncMode_30of32BitsDetected,
    syncMode_noPreamble_sync_carrierSense, // Just sense above threshold
    syncMode_15of16Bits_carrierSense,
    syncMode_16of16Bits_carrierSense,
    syncMode_30of32Bits_carrierSense

} mdmcfg2_syncMode_t;

#define MDMCFG1_REGADDR         0x13        // Modem configuration
#define MDMCFG1_FEC_EN          0x80 // b7 set to enable forward Error Correction (FEC)
#define MDMCFG1_NUM_PREAMBLE(x) ((x<<4)&0x70) // b6:4 set min num preamble bytes to transmit
typedef enum
{
    preambleBytes_2 = 0,
    preambleBytes_3,
    preambleBytes_4,
    preambleBytes_6,
    preambleBytes_8,
    preambleBytes_12,
    preambleBytes_16,
    preambleBytes_24
} mdmcfg1_numPreamBytes_t;
// b3:2 not used, read 0
#define MDMCFG1_CHANSPC_E(x)    (x&0x03) // b1:0 exponent of channel spacing

#define MDMCFG0_REGADDR         0x14        // Modem configuration
// b7:0 CHANSPC_Mantissa[7:0], see datasheet

#define DEVIATN_REGADDR         0x15        // Modem deviation setting
// b7 not used, reads 0
#define DEVIATN_EXP(x)          ((x<<4)&0x70) // b6:4 exponent of deviation
// b3 not used, reads 0
#define DEVIATN_MANTISSA(x)     (x&0x07) // See datasheet for different uses depending on modulation format

#define MCSM2_REGADDR           0x16        // Main Radio Control State Machine config
// b7:5 not used
#define MCSM2_RX_TIME_RSSI      0x10 // Direct RX termination based on RSSI if set.
#define MCSM2_RX_TIME_QUAL      0x08 // check if sync word is found if set
#define MCSM2_RX_TIME(x)        (x&0x07) // See datasheet

#define MCSM1_REGADDR           0x17        // Main Radio Control State Machine config
// b7:6 not used, read 0
#define MCSM1_CCA_MODE(x)       ((x<<4)&0x30) // b5:4 CCA mode
typedef enum
{
    ccaMode_always = 0,
    ccaMode_RssiBelowThreshold,
    ccaMode_ulessReceiving,
    ccaMode_RssiBelowThresholdUnlessRx
} mcsm1_ccaMode_t;
#define MCSM1_RXOFF_MODE(x)     ((x<<2)&0xC0) // b3:2 rxoff_mode
typedef enum
{ // What to do once a packet has been received
    rxOff_idle = 0,
    rxOff_fstxon,
    rxOff_Tx,
    rxOff_stayInRx
} mcsm1_rxoffMode_t;
#define MCSM1_TXOFF_MODE(x)     (x&0x03)
typedef enum
{ // What to do once TX is done sending a packet
    txOff_idle = 0,
    txOff_fstxon,
    txOff_stayInTx, // Start sending another preamble
    txOff_Rx
} mcsm1_txoffMode_t;

#define MCSM0_REGADDR           0x18        // Main Radio Control State Machine config
// b7:6 not used, read 0
#define MCSM0_FS_AUTOCAL(x)     ((x<<4)&0x30) // b5:4 auto-cal when going to RX or TX
typedef enum
{
    fsAutocal_never = 0,
    faAutocal_idleToRxOrTx,
    fsAutocal_RxOrTxToIdle,
    fsAutocal_every4thRxOrTxToIdle
} mcsm0_fsAutocal_t;
#define MCSM0_PO_TIMEOUT(x)     ((x<<2)&0xC0) // b3:2 POWER-ON timeout
typedef enum
{
    poTimeout_1count = 0,
    poTimeout_16counts,
    poTimeout_64counts,
    poTimeout_256counts
} mcsm0_poTimeout_t;
#define MCSM0_PIN_CTRL_EN       0x02 // b1 set enables the pin radio control option
#define MCSM0_XOSC_FORCE_ON     0x01 // b0 set forces XOSC to stay on in SLEEP state

#define FOCCFG_REGADDR          0x19        // Frequency Offset Compensation config
// b7:6 not used, read 0
// b5 FOC_BS_CS_GATE
// b4:3 FOC_PRE_K[1:0]
// b2 FOC_POST_K
// b1:0 FOC_LIMIT[1:0]

#define BSCFG_REGADDR           0x1A        // Bit Synchronization configuration

#define AGCCTRL2_REGADDR        0x1B        // AGC control
#define AGCCTRL2_MAX_DVGA_GAIN(x)   ((x<<6)&0xC0) // b7:6 max_dvga_gain[1:0]
#define AGCCTRL2_MAX_LNA_GAIN(x)    ((x<<3)&0x38) // 5:3 max_lna_gain[2:0]
#define AGCCTRL2_MAGN_TARGET(x)     (x&0x07) // b2:0 MAGN_TARGET[2:0]

#define AGCCTRL1_REGADDR        0x1C        // AGC control
// b7 not used, reads 0
#define AGCCTRL1_AGC_LNA_PRIO       0x40 // b6 selects LNA/2 strategies.
#define AGCCTRL1_CSENSE_REL_THR(x)  ((x<<4)&0x30) // b5:4 set relative change threshold for asserting carrier sense
typedef enum
{
    csense_relThresholdDisabled = 0,
    csense_6dBincrease,
    csense_10dBincrease,
    csense_14dBincrease
} agcctrl1_csenseRelThr_t;
#define AGCCTRL1_CSENSE_ABS_THR(x)  (x&0x0F) // b3:0 set absolute threshold for asserting Csense, 2s-complement 4-bit
// See datasheet

#define AGCCTRL0_REGADDR        0x1D        // AGC control
// See datasheet

#define WOREVT1_REGADDR         0x1E        // High byte Event 0 timeout
#define WOREVT0_REGADDR         0x1F        // Low byte Event 0 timeout

#define WORCTRL_REGADDR         0x20        // Wake On Radio control

#define FREND1_REGADDR          0x21        // Front end RX configuration

#define FREND0_REGADDR          0x22        // Front end TX configuration

#define FSCAL3_REGADDR          0x23        // Frequency synthesizer calibration

#define FSCAL2_REGADDR          0x24        // Frequency synthesizer calibration

#define FSCAL1_REGADDR          0x25        // Frequency synthesizer calibration

#define FSCAL0_REGADDR          0x26        // Frequency synthesizer calibration

#define RCCTRL1_REGADDR         0x27        // RC oscillator configuration

#define RCCTRL0_REGADDR         0x28        // RC oscillator configuration

#define FSTEST_REGADDR          0x29        // Frequency synthesizer cal control

#define PTEST_REGADDR           0x2A        // Production test

#define AGCTEST_REGADDR         0x2B        // AGC test

#define TEST2_REGADDR           0x2C        // Various test settings
#define TEST1_REGADDR           0x2D        // Various test settings
#define TEST0_REGADDR           0x2E        // Various test settings

// Strobe commands
#define STROBE_SRES         0x30        // Reset chip.
#define STROBE_SFSTXON      0x31        // Enable/calibrate freq synthesizer
#define STROBE_SXOFF        0x32        // Turn off crystal oscillator.
#define STROBE_SCAL         0x33        // Calibrate freq synthesizer & disable
#define STROBE_SRX          0x34        // Enable RX.
#define STROBE_STX          0x35        // Enable TX.
#define STROBE_SIDLE        0x36        // Exit RX / TX
//#define STROBE_SAFC       0x37        // AFC adjustment of freq synthesizer
#define STROBE_STARTWOR     0x38        // Start automatic RX polling sequence (Wake on Radio)
#define STROBE_POWERDOWN    0x39        // Enter pwr down mode when CSn goes hi
#define STROBE_FLUSHRXFIFO  0x3A        // Flush the RX FIFO buffer. Can only do this in IDLE or RXOVF states
#define STROBE_FLUSHTXFIFO  0x3B        // Flush the TX FIFO buffer. Can only do this in IDLE or TXUVF states
#define STROBE_RESETRTC     0x3C        // Reset real time clock to event1 value
#define STROBE_NOP          0x3D        // No operation.

// Other memory locations
#define PATABLE_REGADDR      0x3E // Must read or write PATABLE bytes 0:7 in burst read or write
#define TXFIFO_REGADDR       0x3F // Access in burst to read multiple bytes
#define RXFIFO_REGADDR       0x3F // Access in burst to read multiple bytes

// Status registers (need burst mode set)
#define PARTNUM_REGADDR         0xF0        // b7:0 should be 0x00 for part number
#define VERSION_REGADDR         0xF1        // b7:0 are 0x14 maybe, Current version number
#define FREQEST_REGADDR         0xF2        // Frequency offset estimate

#define LQI_REGADDR             0xF3        // Demodulator estimate for link quality
#define LQI_CRC_OK          0x80 // b7 set if last CRC comparison matched.
#define LQI_LQI_EST(x)      (x&0x7F) // b6:0 link quality indicator calculated over first 64 packet bytes

#define RSSI_REGADDR            0xF4        // b7:0 are Received signal strength indication
#define MARCSTATE_REGADDR       0xF5        // Main Radio Control state machine state
// b7:5 not used, read 0
typedef enum
{
    marcState_sleep = 0,
    marcState_idle,
    marcState_xoff,
    marcState_vcoon_mc,
    marcState_regon_mc,
    marcState_mancal,
    marcState_VCOon,
    marcState_regon,
    marcState_startcal,
    marcState_bwboost,
    marcState_fs_lock,
    marcState_ifadcon,
    marcState_endcal,
    marcState_rx,
    marcState_rx_end,
    marcState_rx_rst,
    marcState_txrx_switch,
    marcState_rxfifo_overflow,
    marcState_fstxon,
    marcState_tx,
    marcState_tx_end,
    marcState_rxtx_switch,
    marcState_txfifo_underflow
} marcState_state_t;

#define WORTIME1_REGADDR        0xF6        // High byte of WOR timer
#define WORTIME0_REGADDR        0xF7        // Low byte of WOR timer

#define PKTSTATUS_REGADDR       0xF8        // Current GDOx status and packet status
#define PKTSTATUS_CRC_OK        0x80 // b7 set if last CRC matched
#define PKTSTATUS_CS            0x40 // b6 Carrier sense. Cleared when entering idle
#define PKTSTATUS_PQT_REACHED   0x20 // b5 set if Preamble Quality is reached.
#define PKTSTATUS_CCA           0x10 // b4 set if channel is clear
#define PKTSTATUS_SFD           0x08 // b3 set start of Frame Delimiter, sync word has been received
#define PKTSTATUS_GDO2          0x04 // b2 set if GDO2 is active (non-inverted)
// b1 not used, reads 0
#define PKTSTATUS_GDO0          0x01 // b0 set if GDO2 is active (non-inverted)

#define VCO_VC_DAC_REGADDR      0xF9        // Test only: Current setting from PLL cal module

#define TXBYTES_REGADDR         0xFA        // Underflow and # of bytes in TXFIFO
#define TXBYTES_UNDERFLOW   0x80 // b7 set if underflow in TX register
#define TXBYTES_NUM(x)      (x&0x7F) // b6:0 num bytes in TX FIFO

#define RXBYTES_REGADDR         0xFB        // Overflow and # of bytes in RXFIFO
#define RXBYTES_OVERFLOW    0x80 // b7 set if RXoverflowed FIFO
#define RXBYTES_NUM(x)      (x&0x7F) // b6:0 num bytes in RX FIFO

#define RCCTRL1_STATUS_REGADDR  0xFC    // RC oscillator cal routine result.
#define RCCTRL0_STATUS_REGADDR  0xFD    // RC oscillator cal routine result.

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

#define TX_TEST_ITVL_MS  0 // non-zero to run TX test poll

#define PKT_SIZE_FIXED 1 // TODO switch to variable later to save power.
#if PKT_SIZE_FIXED
#define PKT_LEN 1
#else
#error "Define packet size or type"
#endif // #if PKT_SIZE_FIXED

#define RX_POLL_ITVL_MS 500 // Datasheet says not to spam SPI with traffic

typedef enum
{
    packetRxState_awaitingPacketStart,
    packetRxState_awaitingPacketBytes,
    packetRxState_receivedPacketBytes,
    packetRxState_awaitingRSSILQI,
    packetRxState_finishedWithPacket
} packetRxState_t;

typedef struct
{
    uint8_t* pPacket;
    uint32_t packetBufIndex, packetBufLen;
    bool readFromChip;
    pktCtrl0_lenCfg_t packetLenCfg;
    uint8_t expectedPacketLen;
    bool RSSIandLQIAppended;
    int8_t RSSI;
    uint8_t CRCandLQI;
    packetRxState_t rxState;

} packetRxSettings_t;

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static uint8_t m_lastChipStatusByte;

#if TX_TEST_ITVL_MS
static uint32_t m_lastTx_ms;
#endif // #if TX_TEST_ITVL_MS

static uint32_t m_lastRxPoll_ms;

// For receiving packets
static uint8_t m_rxPacketBuffer[PKT_LEN];
static packetRxSettings_t m_rxSettings;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static void updateStatus(uint8_t newVal)
{

    if (CHIPSTATUSBYTE_STATE(m_lastChipStatusByte) != CHIPSTATUSBYTE_STATE(newVal))
    {
        NRF_LOG_DEBUG("Status from 0x%x to 0x%x", CHIPSTATUSBYTE_STATE(m_lastChipStatusByte),
                      CHIPSTATUSBYTE_STATE(newVal));
        m_lastChipStatusByte = newVal;
    }
}

chipStatusByteState_t cc1101_getLastState(void)
{
    return CHIPSTATUSBYTE_STATE(m_lastChipStatusByte);
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
//    NRF_LOG_INFO("Strobed 0x%x, read Status 0x%x", address, sres);
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
    startAddress |= CC1101_WRITE_BURST;
    uint8_t sres;
    nrfx_spi_xfer_desc_t xfer;
    xfer.p_rx_buffer = &sres;
    xfer.p_tx_buffer = &startAddress;
    xfer.rx_length = 1;
    xfer.tx_length = 1;
    ret_code_t ret = spi0_xfer(spi0_cc1101, &xfer, true);
    updateStatus(sres);
    ret |= spi0_write(spi0_cc1101, data, len, false);
    return ret == NRF_SUCCESS ? true : false;
}

static bool cc1101_readBurst(uint8_t startAddress, uint8_t* buffer, uint32_t len)
{
    startAddress |= CC1101_READ_BURST;
    uint8_t sres;
    nrfx_spi_xfer_desc_t xfer;
    xfer.p_rx_buffer = &sres;
    xfer.p_tx_buffer = &startAddress;
    xfer.rx_length = 1;
    xfer.tx_length = 1;
    ret_code_t ret = spi0_xfer(spi0_cc1101, &xfer, true);
    updateStatus(sres);
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
        cc1101_strobe(STROBE_NOP);
        if (m_lastChipStatusByte & 0x80)
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
    bool ret = cc1101_strobe(STROBE_SRES);
    if (!ret)
    {
        NRF_LOG_WARNING("Failed to send strobe command");
    }
    // Wait for it to reboot. Datasheet says max of
    retries = 100;
//    ms_timestamp = uptimeCounter_getUptimeMs();
    while (retries)
    { //  drive CS low, see if it responds by driving MISO low. Can do this by reading status byte
        cc1101_strobe(STROBE_NOP);
        if (m_lastChipStatusByte & 0x80)
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

static bool cc1101_selfTest(void)
{
    uint8_t partNum;
    uint8_t version;
    bool ret;

    // Read partNum and version registers
    ret = cc1101_readSingleByte(PARTNUM_REGADDR, &partNum);
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
    ret = cc1101_readSingleByte(VERSION_REGADDR, &version);
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
                 CHIPSTATUSBYTE_STATE(m_lastChipStatusByte));
    return true;

}

void cc1101_setIdle(bool flushFifos)
{
    cc1101_strobe(STROBE_SIDLE); // Tell it to idle
    uint8_t marcState;
    cc1101_readSingleByte(MARCSTATE_REGADDR, &marcState);
    while (marcState_idle != marcState)
    {
        nrf_delay_us(100);
        cc1101_readSingleByte(MARCSTATE_REGADDR, &marcState);
    }
    if (flushFifos)
    {
        cc1101_strobe(STROBE_FLUSHTXFIFO); // Can flush in idle
        cc1101_strobe(STROBE_FLUSHRXFIFO); // Can flush in idle
    }
//    nrf_delay_us(100);
}

// From TI studio Nov 13, 2023 for 3kBaud at 433MHz ASK
static void cc1101_initASKTx_myStudio(void)
{
    cc1101_writeSingleByte(IOCFG2_REGADDR, 0x29);
    cc1101_writeSingleByte(IOCFG1_REGADDR, 0x2e);
//    cc1101_writeSingleByte(IOCFG0, 0x06);
    cc1101_writeSingleByte(IOCFG0_REGADDR, 0x3F);
    cc1101_writeSingleByte(FIFOTHR_REGADDR, 0x07); // TODO is this what we want?
    cc1101_writeSingleByte(SYNC1_REGADDR, 0xd3);
    cc1101_writeSingleByte(SYNC0_REGADDR, 0x91);
//    cc1101_writeSingleByte(PKTLEN, 0xff);
    cc1101_writeSingleByte(PKTLEN_REGADDR, 0x10);
    cc1101_writeSingleByte(PKTCTRL1_REGADDR, 0x04);
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x05);
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x04);
    cc1101_writeSingleByte(ADDR_REGADDR, 0x00);
    cc1101_writeSingleByte(CHANNR_REGADDR, 0x00);
//    cc1101_writeSingleByte(FSCTRL1_REGADDR, 0x06);
    cc1101_writeSingleByte(FSCTRL1_REGADDR, 0x0F);
    cc1101_writeSingleByte(FSCTRL0_REGADDR, 0x00);
    cc1101_writeSingleByte(FREQ2_REGADDR, 0x10);
//    cc1101_writeSingleByte(FREQ1_REGADDR, 0xA7);
    cc1101_writeSingleByte(FREQ1_REGADDR, 0xA8);
//    cc1101_writeSingleByte(FREQ0_REGADDR, 0x62);
    cc1101_writeSingleByte(FREQ0_REGADDR, 0x5E);
//    cc1101_writeSingleByte(MDMCFG4_REGADDR, 0xF6);
    cc1101_writeSingleByte(MDMCFG4_REGADDR, 0x86);
    cc1101_writeSingleByte(MDMCFG3_REGADDR, 0xE4);
//    cc1101_writeSingleByte(MDMCFG2_REGADDR, 0x37);
    cc1101_writeSingleByte(MDMCFG2_REGADDR, 0x30);
//    cc1101_writeSingleByte(MDMCFG1_REGADDR, 0x00);
    cc1101_writeSingleByte(MDMCFG1_REGADDR, 0x02);
    cc1101_writeSingleByte(MDMCFG0_REGADDR, 0xF8);
    cc1101_writeSingleByte(DEVIATN_REGADDR, 0x47);
    cc1101_writeSingleByte(MCSM2_REGADDR, 0x07);
    cc1101_writeSingleByte(MCSM1_REGADDR, 0x30);
//    cc1101_writeSingleByte(MCSM0_REGADDR, 0x18);
    cc1101_writeSingleByte(MCSM0_REGADDR, 0x04);
//    cc1101_writeSingleByte(FOCCFG_REGADDR, 0x16);
    cc1101_writeSingleByte(FOCCFG_REGADDR, 0x36);
    cc1101_writeSingleByte(BSCFG_REGADDR, 0x6C);
    cc1101_writeSingleByte(AGCCTRL2_REGADDR, 0x03);
    cc1101_writeSingleByte(AGCCTRL1_REGADDR, 0x40);
    cc1101_writeSingleByte(AGCCTRL0_REGADDR, 0x91);
    cc1101_writeSingleByte(WOREVT1_REGADDR, 0x87);
    cc1101_writeSingleByte(WOREVT0_REGADDR, 0x6B);
    cc1101_writeSingleByte(WORCTRL_REGADDR, 0xF8);
    cc1101_writeSingleByte(FREND1_REGADDR, 0x56);
    cc1101_writeSingleByte(FREND0_REGADDR, 0x11);
    cc1101_writeSingleByte(FSCAL3_REGADDR, 0xE9);
    cc1101_writeSingleByte(FSCAL2_REGADDR, 0x2A);
    cc1101_writeSingleByte(FSCAL1_REGADDR, 0x00);
    cc1101_writeSingleByte(FSCAL0_REGADDR, 0x1F);
    cc1101_writeSingleByte(RCCTRL1_REGADDR, 0x41);
    cc1101_writeSingleByte(RCCTRL0_REGADDR, 0x00);
    cc1101_writeSingleByte(FSTEST_REGADDR, 0x59);
    cc1101_writeSingleByte(PTEST_REGADDR, 0x7F);
    cc1101_writeSingleByte(AGCTEST_REGADDR, 0x3F);
//    cc1101_writeSingleByte(TEST2_REGADDR, 0x81);
    cc1101_writeSingleByte(TEST2_REGADDR, 0x88);
//    cc1101_writeSingleByte(TEST1_REGADDR, 0x35);
    cc1101_writeSingleByte(TEST1_REGADDR, 0x31);
    cc1101_writeSingleByte(TEST0_REGADDR, 0x09);
    uint8_t paTableBytes[8] = { 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    cc1101_writeBurst(PATABLE_REGADDR, paTableBytes, 8);

    // TODO customize IO pins
//    cc1101_writeSingleByte(IOCFG2, 0x0B);     //serial clock.synchronous to the data in synchronous serial mode
//        cc1101_writeSingleByte(IOCFG0, 0x06); //asserts when sync word has been sent/received, and de-asserts at the end of the packet

    cc1101_setIdle(true);
//    cc1101_flushTxFifo();
//    cc1101_flushRxFifo();
}

static void setupFeb6(void)
{
    if (statusByteState_idle != CHIPSTATUSBYTE_STATE(m_lastChipStatusByte))
    {
        NRF_LOG_DEBUG("CC1101 in state %d, not idle, go to idle:", CHIPSTATUSBYTE_STATE(m_lastChipStatusByte));
        cc1101_setIdle(true); // Idle it before writing configs
    }

    /* Rf settings for CC1101, exported Feb 06, 2024
     *432.99MHz, XTAL 26MHz
     */
//    cc1101_writeSingleByte(IOCFG0_REGADDR, 0x06);  //GDO0 Output Pin Configuration
    cc1101_writeSingleByte(IOCFG2_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
    cc1101_writeSingleByte(IOCFG1_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
    cc1101_writeSingleByte(IOCFG0_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));

    cc1101_writeSingleByte(FIFOTHR_REGADDR, 0x47); //RX FIFO and TX FIFO Thresholds

    //    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x05); //Packet Automation Control
    cc1101_writeSingleByte(PKTCTRL1_REGADDR, PKTCTRL1_APPEND_STATUS | PKTCTRL1_ADDR_CHK(addrChk_none));
#if PKT_SIZE_FIXED
//    regByte | = TODO CRC, maybe whitening
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, PKTCTRL0_LENGTH_CFG(pktLen_fixed)); //Packet Automation Control
    cc1101_writeSingleByte(PKTLEN_REGADDR, PKT_LEN);
#else
#error "define"
#endif // #if PKT_SIZE_FIXED
    cc1101_writeSingleByte(FSCTRL1_REGADDR, 0x06); //Frequency Synthesizer Control
    cc1101_writeSingleByte(FREQ2_REGADDR, 0x10);   //Frequency Control Word_REGADDR, High Byte
    cc1101_writeSingleByte(FREQ1_REGADDR, 0xA7);   //Frequency Control Word_REGADDR, Middle Byte
    cc1101_writeSingleByte(FREQ0_REGADDR, 0x62);   //Frequency Control Word_REGADDR, Low Byte
    cc1101_writeSingleByte(MDMCFG4_REGADDR, 0xF5); //Modem Configuration
    cc1101_writeSingleByte(MDMCFG3_REGADDR, 0x83); //Modem Configuration
    cc1101_writeSingleByte(MDMCFG2_REGADDR, 0x33); //Modem Configuration
    cc1101_writeSingleByte(DEVIATN_REGADDR, 0x15); //Modem Deviation Setting
    cc1101_writeSingleByte(MCSM0_REGADDR, 0x18);   //Main Radio Control State Machine Configuration
    cc1101_writeSingleByte(FOCCFG_REGADDR, 0x14);  //Frequency Offset Compensation Configuration
    cc1101_writeSingleByte(AGCCTRL0_REGADDR, 0x92);  //AGC Control
    cc1101_writeSingleByte(WORCTRL_REGADDR, 0xFB); //Wake On Radio Control
    cc1101_writeSingleByte(FREND0_REGADDR, 0x11);  //Front End TX Configuration: PA_PWR[2:0] index in b2:0
    cc1101_writeSingleByte(FSCAL3_REGADDR, 0xE9);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL2_REGADDR, 0x2A);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL1_REGADDR, 0x00);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL0_REGADDR, 0x1F);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(TEST2_REGADDR, 0x81);   //Various Test Settings
    cc1101_writeSingleByte(TEST1_REGADDR, 0x35);   //Various Test Settings
    cc1101_writeSingleByte(TEST0_REGADDR, 0x09);   //Various Test Settings
//    uint8_t paTableBytes[8] = { 0x00, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // 0x1D for -15dBm
    uint8_t paTableBytes[8] = { 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // 0x03 for -30dBm TX power
    cc1101_writeBurst(PATABLE_REGADDR, paTableBytes, 8);

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

static void readInSettings(packetRxSettings_t* pSettings)
{
    uint8_t regByte;
    // Read the packet length config
    cc1101_readSingleByte(PKTCTRL0_REGADDR, &regByte);
    pSettings->packetLenCfg = PKTCTRL0_LENGTH_CFG(regByte);
    // Read whether the RSSI and LQI are appended
    cc1101_readSingleByte(PKTCTRL1_REGADDR, &regByte);
    if (regByte & PKTCTRL1_APPEND_STATUS)
    {
        pSettings->RSSIandLQIAppended = true;
        NRF_LOG_DEBUG("Read in settings: Expect appended RSSI and LQI");
    }
    if (pktLen_fixed == pSettings->packetLenCfg)
    { // Read expected packet length
        cc1101_readSingleByte(PKTLEN_REGADDR, &pSettings->expectedPacketLen);
        NRF_LOG_DEBUG("Read in settings: Fixed packet len of %d", pSettings->expectedPacketLen);
    }
    else if (pktLen_variable == pSettings->packetLenCfg)
    { // Read in max length, if desired
        NRF_LOG_DEBUG("Read in settings: Variable packet len");
    }
    else
    {
        NRF_LOG_ERROR("Read in settings: UNSUPPORTED packet len config %d", pSettings->packetLenCfg);
    }
    pSettings->readFromChip = false;
}

static void readLQIandRSSI(packetRxSettings_t* pSettings, uint8_t numFifoBytes)
{ // RSSI first, then LQI and CRC_OK
    cc1101_readSingleByte(RXFIFO_REGADDR, (uint8_t*)&pSettings->RSSI);
    cc1101_readSingleByte(RXFIFO_REGADDR, &pSettings->CRCandLQI);
    pSettings->rxState = packetRxState_finishedWithPacket;
    NRF_LOG_DEBUG("Read RSSI %d,LQI %d, CRC %s", pSettings->RSSI, LQI_LQI_EST(pSettings->CRCandLQI),
                  (pSettings->CRCandLQI & LQI_CRC_OK) ? "ok or disabled" : "not ok");
//    uint8_t regByte;
//    cc1101_readSingleByte(LQI_REGADDR, &regByte);
//    NRF_LOG_DEBUG(" Read LQI regaddr: LQI %d.", LQI_LQI_EST(pSettings->CRCandLQI));
}

static void readPacketBytes(packetRxSettings_t* pSettings, uint8_t numFifoBytes)
{
    uint32_t numExpected = pSettings->expectedPacketLen - pSettings->packetBufIndex;
    if (numFifoBytes < numExpected)
    { // If bytes are still coming in, don't empty the FIFO completely (section 20)
        cc1101_readBurst(RXFIFO_REGADDR, &pSettings->pPacket[pSettings->packetBufIndex], numFifoBytes - 1);
        pSettings->rxState = packetRxState_awaitingPacketBytes;
        pSettings->packetBufIndex += numFifoBytes - 1;
//        NRF_LOG_INFO("fixedLen read %d of expected %d bytes, try again later", pSettings->packetBufIndex,
//                     pSettings->expectedPacketLen);
    }
    else
    { // We have the bytes we need for a full packet, read them out, report done
        if (!numExpected)
        {
            NRF_LOG_ERROR("numExpected==0");
        }
        cc1101_readBurst(RXFIFO_REGADDR, &pSettings->pPacket[pSettings->packetBufIndex], numExpected);
        numFifoBytes -= numExpected;
        pSettings->packetBufIndex += numExpected;
        pSettings->rxState = packetRxState_receivedPacketBytes;
//        NRF_LOG_INFO("Read fixed length packet of %d bytes, done.", pSettings->packetBufIndex);
        if (pSettings->RSSIandLQIAppended)
        { // If we should read LQI and RSSI, try and read those
            if (2 <= numFifoBytes)
            {
                readLQIandRSSI(pSettings, numFifoBytes);
            }
            else
            { // Wait til next poll
                pSettings->rxState = packetRxState_awaitingRSSILQI;
            }
        }
        else
        {
            pSettings->rxState = packetRxState_finishedWithPacket;
        }
    }
}

static void readVariablePacket(packetRxSettings_t* pSettings, uint8_t numFifoBytes)
{
    if (packetRxState_awaitingPacketStart == pSettings->rxState)
    { // We need to read the first byte to know the length
        cc1101_readSingleByte(RXFIFO_REGADDR, &pSettings->expectedPacketLen); // First byte is length byte
        pSettings->rxState = packetRxState_awaitingPacketBytes;
        // Now see if we have enough bytes to receive the rest
        numFifoBytes--;
        if (numFifoBytes)
        {
            readPacketBytes(pSettings, numFifoBytes);
        }
    }
    else if (packetRxState_awaitingPacketBytes == pSettings->rxState)
    { // If awaiting packet bytes, read them
        readPacketBytes(pSettings, numFifoBytes);
    }
    else
    {
        NRF_LOG_ERROR("Stuck, ??");
    }
}

static void tryReceivePacket(packetRxSettings_t* pSettings)
{
    uint8_t regByte;
    if (NULL == pSettings)
    {
        return;
    }
    if (pSettings->readFromChip)
    {
        readInSettings(pSettings);
    }
    if (pSettings->packetBufIndex == pSettings->expectedPacketLen)
    {
        NRF_LOG_ERROR("Doesn't make sense, resetting receiver settings");
        pSettings->rxState = packetRxState_awaitingPacketStart;
        pSettings->packetBufIndex = 0;
    }
    // Read RXBYTES to see if there are any bytes
    cc1101_readSingleByte(RXBYTES_REGADDR, &regByte);
    if (regByte & RXBYTES_OVERFLOW)
    {
        NRF_LOG_WARNING("Rx OVERFLOW, flushing and going back to RX!");
        cc1101_strobe(STROBE_FLUSHRXFIFO);
        cc1101_strobe(STROBE_SRX);
        pSettings->rxState = packetRxState_awaitingPacketStart;
        pSettings->packetBufIndex = 0;
        return;
    }
    uint8_t numFifoBytes = RXBYTES_NUM(regByte);
    if (!numFifoBytes)
    { // No bytes to read
        return;
    }
    // If here, we have bytes to read in RX FIFO
//    NRF_LOG_INFO("have %d bytes in RXFIFO", numFifoBytes);
    switch (pSettings->packetLenCfg)
    {
        case pktLen_fixed:
            readPacketBytes(pSettings, numFifoBytes);
        break;
        case pktLen_variable:
            readVariablePacket(pSettings, numFifoBytes);
        break;
        default:
            NRF_LOG_ERROR("Packet type %d not handled yet", pSettings->packetLenCfg)
            ;
        break;
    }
}

static void parsePacket(uint8_t* pData, uint32_t len)
{
    NRF_LOG_DEBUG("TODO parse %d byte packet", len);
}

static void cc1101Poll(void)
{
    if (uptimeCounter_elapsedSince(m_lastRxPoll_ms) >= RX_POLL_ITVL_MS)
    {
        cc1101_strobe(STROBE_NOP);
        switch (CHIPSTATUSBYTE_STATE(m_lastChipStatusByte))
        {
            case statusByteState_tx:
                // If in TX state
#if TX_TEST_ITVL_MS
                if (uptimeCounter_elapsedSince(m_lastTx_ms) > 1000)
                #endif // #if TX_TEST_ITVL_MS
            {
                NRF_LOG_WARNING("timed out in TX, switch to RX");
                cc1101_strobe(STROBE_SRX);
            }
            break;
            case statusByteState_rx:
                // If in receiving state
//                NRF_LOG_DEBUG("Check for RX bytes")
//                ;
                tryReceivePacket(&m_rxSettings);
                if (packetRxState_finishedWithPacket == m_rxSettings.rxState)
                {
                    if (m_rxSettings.packetBufIndex != m_rxSettings.expectedPacketLen)
                    {
                        NRF_LOG_WARNING("Something wrong, didn't receive expected bytes");
                    }
                    else
                    { // Parse it, if it looks OK.
                        parsePacket(m_rxSettings.pPacket, m_rxSettings.packetBufLen);
                    }
                    m_rxSettings.rxState = packetRxState_awaitingPacketStart;
                    m_rxSettings.packetBufIndex = 0;
                }
                else
                { // If not in finished state, check if length is equal, that would be an error
                    if (m_rxSettings.packetBufIndex >= m_rxSettings.expectedPacketLen)
                    {
                        NRF_LOG_WARNING("Received %d of %d bytes, but state still %d", m_rxSettings.packetBufIndex,
                                        m_rxSettings.expectedPacketLen,
                                        m_rxSettings.rxState);
                        // reset parser state
                        m_rxSettings.rxState = packetRxState_awaitingPacketStart;
                        m_rxSettings.packetBufIndex = 0;
                    }
                    // Keep waiting for packet
                }
            break;
            case statusByteState_idle:
                NRF_LOG_DEBUG("move from idle to RX:")
                ;
                cc1101_strobe(STROBE_SRX);
            break;
            case statusByteState_rxOverflow:
                NRF_LOG_ERROR("Detected RX overflow, flushing and idling")
                ;
                cc1101_setIdle(true);
            break;
            case statusByteState_txUnderflow:
                NRF_LOG_ERROR("Detected TX underflow, flushing and idling")
                ;
                cc1101_setIdle(true);
            break;
            default:
                NRF_LOG_WARNING("State 0x%x", CHIPSTATUSBYTE_STATE(m_lastChipStatusByte))
                ;
            break;
        }
        m_lastRxPoll_ms = uptimeCounter_getUptimeMs();
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
        cc1101_sendBytes(txPacketBytes, PKT_LEN);
        m_lastTx_ms = uptimeCounter_getUptimeMs();
    }
#endif // #if TX_TEST_ITVL_MS
}

bool cc1101_sendPacket(uint8_t* pBytes, uint8_t len)
{
    if (len > PKT_LEN)
    {
        NRF_LOG_ERROR("Can only send %d bytes per packet, not %d, ignoring", PKT_LEN, len);
        return false;
    }
    uint8_t pktBuf[PKT_LEN];
    for (uint8_t i = 0; i < PKT_LEN; i++)
    {
        if (i < len)
        {
            pktBuf[i] = pBytes[i];
        }
        else
        {
            pktBuf[i] = 0xFF;
        }
    }
    bool ret = true;
    ret &= cc1101_writeBurst(TXFIFO_REGADDR, pBytes, PKT_LEN);
    uint8_t regData;
    ret &= cc1101_readSingleByte(TXBYTES_REGADDR, &regData);
    NRF_LOG_DEBUG("Wrote %d bytes to txfifo, TXYBTES says %d", PKT_LEN, regData);
    ret &= cc1101_strobe(STROBE_STX);
    return ret;
}

void cc1101_init(void)
{
    spi0_init(); // Make sure pins are muxed and working
    m_lastChipStatusByte = 0xFF;
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

    m_rxSettings.pPacket = m_rxPacketBuffer;
    m_rxSettings.readFromChip = true;
    m_rxSettings.packetBufIndex = 0;
    m_rxSettings.packetBufLen = sizeof(m_rxPacketBuffer);
    m_rxSettings.rxState = packetRxState_awaitingPacketStart;

    pollers_registerPoller(cc1101Poll);
// Start by going to idle
    cc1101_setIdle(true);
}
