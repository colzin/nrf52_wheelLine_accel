/*
 * CC1101.c
 *
 *  Created on: Feb 5, 2024
 *      Author: Collin Moore
 */

#include "cc1101.h"

#if COMPILE_RADIO_CC1101
#include "_4digit7seg.h"

#include "ev1527SPI.h" // For Async TX

#include "globalInts.h" // To set machine state

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

#define UART_PRINTS 1

#if UART_PRINTS
#include "uartTerminal.h"
#include <stdio.h> // for snprintf
#endif // #if UART_PRINTS

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
// See .h for def

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
#define PKTCTRL0_PKT_FORMAT_WRITE(x)  ((x<<4)&0x30)   // b5:4
#define PKTCTRL0_PKT_FORMAT_READ(x)  ((pktCtrl0_pktFmt_t)((x>>4)&0x03))   // b5:4 shift down to 1:0
typedef enum
{
    pktCtrl0Format_normal = 0,
    pktCtrl0Format_syncSerial,
    pktCtrl0Format_randomTX,
    pktCtrl0Format_asyncSerial
} pktCtrl0_pktFmt_t;
// b3 not used, reads 0
#define PKTCTRL0_CRC_EN         0x04            // b2 set turns on CRC calculation in RX and TX
#define PKTCTRL0_LENGTH_CFG(x)  (x&0x03)        // b1:0 packet length config
typedef enum
{
    pktCtrl0Len_fixed = 0,
    pktCtrl0Len_variable,
    pktCtrl0Len_infinite,
    pktCtrl0Len_reserved
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
    ccaMode_RssiBelowThresholdUnlessRx // This is default
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
    bool CRCok;
    uint8_t LQI;
    packetRxState_t rxState;

} packetRxSettings_t;

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static uint8_t m_lastChipStatusByte;

#if TX_TEST_ITVL_MS
static uint32_t m_lastTx_ms;
#endif // #if TX_TEST_ITVL_MS

static chipStatusByteState_t m_lastReadStatusByteState;
static uint32_t m_inState_ms;
static uint32_t m_lastPoll_ms;

// For receiving packets
static uint8_t m_rxPacketBuffer[PKT_LEN];
static packetRxSettings_t m_rxSettings;

static cc1101Mode_t m_opMode;

static uint8_t m_sendState;

static bool m_newTxPower;
static int8_t m_desiredTxdBm;

const char* m_statuses[statusByteState_txUnderflow + 1]
=
        {
          "idle",
          "rx",
          "tx",
          "fastTxReady",
          "cal",
          "settling",
          "rxOverflow",
          "txUnderflow",
        };

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
        NRF_LOG_DEBUG("Status from %s to %s", m_statuses[CHIPSTATUSBYTE_STATE(m_lastChipStatusByte)],
                      m_statuses[CHIPSTATUSBYTE_STATE(newVal)]);
    }
    m_lastChipStatusByte = newVal;
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
    if (NRF_SUCCESS == ret)
    {
//        NRF_LOG_INFO("Strobed 0x%x, read Status 0x%x", address, sres);
        updateStatus(sres);
        return true;
    }
    else
    {
        NRF_LOG_ERROR("Error %d strobing 0x%x", ret, address);
        return false;
    }
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
    if (NRF_SUCCESS == ret)
    {
        //    NRF_LOG_INFO("Wrote address 0x%x to 0x%x, read Status 0x%x", address, data, sres);
        updateStatus(sres);
        return true;
    }
    else
    {
        NRF_LOG_ERROR("Error %d writing reg 0x%x with byte 0x%x", ret, address, data);
        return false;
    }
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
    if (NRF_SUCCESS == ret)
    {
        updateStatus(sres);
    }
    else
    {
        NRF_LOG_ERROR("Error %d writing startAddress 0x%x", ret, startAddress);
        return false;
    }
    // Else continue
    ret = spi0_write(spi0_cc1101, data, len, false);
    if (NRF_SUCCESS == ret)
    {
        return true;
    }
    else
    {
        NRF_LOG_ERROR("Error %d writing %d regs ", ret, len);
        return false;
    }
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

#if READ_ALL_REGS
static void readAllRegs(void)
{
    nrf_delay_ms(1);
    uint8_t buf[0x3D];
    cc1101_readBurst(0x0, buf, sizeof(buf));
    for (uint32_t i = 0; i < sizeof(buf); i += 16)
    {
        NRF_LOG_HEXDUMP_DEBUG(&buf[i], 16);
        nrf_delay_ms(50);
    }
}
#endif // #if READ_ALL_REGS

static bool cc1101_reset(void)
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
    if (statusByteState_idle != CHIPSTATUSBYTE_STATE(m_lastChipStatusByte))
    {
        NRF_LOG_DEBUG("Go from state %s to idle", m_statuses[CHIPSTATUSBYTE_STATE(m_lastChipStatusByte)]);
        cc1101_strobe(STROBE_SIDLE); // Tell it to idle
        uint8_t marcState;
        cc1101_readSingleByte(MARCSTATE_REGADDR, &marcState);
        while (marcState_idle != marcState)
        {
            nrf_delay_us(100);
            cc1101_readSingleByte(MARCSTATE_REGADDR, &marcState);
        } // Make sure it goes into idle
    }
    // Now, if flushFifos, send those strobes
    if (flushFifos)
    {
        NRF_LOG_DEBUG("Flushing RX and TX fifos");
        cc1101_strobe(STROBE_FLUSHTXFIFO); // Can flush in idle
        cc1101_strobe(STROBE_FLUSHRXFIFO); // Can flush in idle
    }
//    nrf_delay_us(100);
}

static void setTxPower(int8_t tx_dBm)
{ // DN013 says don't use 0x61 to 0x6F
    uint8_t desiredPaVal;
    if (tx_dBm <= -62)
    {
        m_desiredTxdBm = -62;
        desiredPaVal = 0x00;
    }
    else if (tx_dBm <= -38)
    {
        m_desiredTxdBm = -38;
        desiredPaVal = 0x30;
    }
    else if (tx_dBm <= -36)
    {
        m_desiredTxdBm = -36;
        desiredPaVal = 0x01;
    }
    else if (tx_dBm <= -34)
    {
        m_desiredTxdBm = -34;
        desiredPaVal = 0x02;
    }
    else if (tx_dBm <= -31)
    {
        m_desiredTxdBm = -31;
        desiredPaVal = 0x03;
    }
    else if (tx_dBm <= -30)
    {
        m_desiredTxdBm = -30;
        desiredPaVal = 0x12;
    }
    else if (tx_dBm <= -28)
    {
        m_desiredTxdBm = -28;
        desiredPaVal = 0x05;
    }
    else if (tx_dBm <= -25)
    {
        m_desiredTxdBm = -25;
        desiredPaVal = 0x07;
    }
    else if (tx_dBm <= -23)
    {
        m_desiredTxdBm = -23;
        desiredPaVal = 0x09;
    }
    else if (tx_dBm <= -20)
    {
        m_desiredTxdBm = -20;
        desiredPaVal = 0x0E;
    }
    else if (tx_dBm <= -15)
    {
        m_desiredTxdBm = -15;
        desiredPaVal = 0x1D;
    }
    else if (tx_dBm <= -10)
    {
        m_desiredTxdBm = -10;
        desiredPaVal = 0x34;
    }
    else if (tx_dBm <= -6)
    {
        m_desiredTxdBm = -6;
        desiredPaVal = 0x2A;
    }
    else if (tx_dBm <= 0)
    {
        m_desiredTxdBm = 0;
        desiredPaVal = 0x60;
    }
    else if (tx_dBm <= 5)
    {
        m_desiredTxdBm = 5;
        desiredPaVal = 0x84;
    }
    else if (tx_dBm <= 7)
    {
        m_desiredTxdBm = 7;
        desiredPaVal = 0xC8;
    }
    else if (tx_dBm <= 9)
    {
        m_desiredTxdBm = 9;
        desiredPaVal = 0xC1;
    }
    else
    {
        m_desiredTxdBm = 10;
        desiredPaVal = 0xC0;
    }

    NRF_LOG_INFO("Switching to %d dBm", m_desiredTxdBm);
#ifdef UART_TX_PIN
    char strBuf[96];
    int strLen = snprintf(strBuf, sizeof(strBuf), "Switching to TX %d dBm\n", m_desiredTxdBm);
    if (0 < strLen)
    {
        uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
    }
#endif // #ifdef UART_TX_PIN

#if ASK_PATABLE
    // First value in PATABLE is off value, second is on value
    uint8_t paTableBytes[8] = { 0x00, desiredPaVal, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
#else
    // Set the first entry to what we want
    uint8_t paTableBytes[8] = { desiredPaVal, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
#endif // #if ASK_PATABLE
    if (cc1101_writeBurst(PATABLE_REGADDR, paTableBytes, 8))
    {
        NRF_LOG_INFO("Set PATABLE");
    }
    else
    {
        NRF_LOG_ERROR("ERROR setting PATABLE");
    }
    m_newTxPower = false;
}

#if BASIC_ASK_433
static void setupPacketRadio(int8_t desiredTx_dBm)
{
    if (statusByteState_idle != CHIPSTATUSBYTE_STATE(m_lastChipStatusByte))
    {
        cc1101_setIdle(true); // Idle it before writing configs
    }

    /* Rf settings for CC1101, exported Feb 06, 2024
     * 432.99MHz, XTAL 26MHz
     */
//    cc1101_writeSingleByte(IOCFG0_REGADDR, 0x06);  //GDO0 Output Pin Configuration
#if GDO2_STATUS
    cc1101_writeSingleByte(IOCFG2_REGADDR, IOCFGx_GDOx_CFG(gdox_));
#else
    cc1101_writeSingleByte(IOCFG2_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
#endif // #if GDO2_STATUS

    cc1101_writeSingleByte(IOCFG1_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
    cc1101_writeSingleByte(IOCFG0_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));

    cc1101_writeSingleByte(FIFOTHR_REGADDR, 0x47); //RX FIFO and TX FIFO Thresholds

    //    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x05); //Packet Automation Control
    cc1101_writeSingleByte(PKTCTRL1_REGADDR, PKTCTRL1_APPEND_STATUS | PKTCTRL1_ADDR_CHK(addrChk_none));
#if PKT_SIZE_FIXED
//    regByte | = TODO CRC, maybe whitening
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, PKTCTRL0_LENGTH_CFG(pktCtrl0Len_fixed)); //Packet Automation Control
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

    // Added MCSM1 to go automatically from TX to RX when sending.
    uint8_t u8 = MCSM1_CCA_MODE(ccaMode_RssiBelowThresholdUnlessRx);
    u8 |= MCSM1_RXOFF_MODE(rxOff_idle); // Go to Idle when done with RX
    u8 |= MCSM1_TXOFF_MODE(txOff_Rx); // Go to TX when done with RX
    cc1101_writeSingleByte(MCSM1_REGADDR, u8);
    // generated: cc1101_writeSingleByte(MCSM0_REGADDR, 0x18);   //Main Radio Control State Machine Configuration
    // Generated had autocal when idle to RX or TX 0x10, and PO_TIMEOUT of 0b10 (0x08)
    u8 = MCSM0_FS_AUTOCAL(faAutocal_idleToRxOrTx);
    u8 |= MCSM0_PO_TIMEOUT(poTimeout_16counts);
    cc1101_writeSingleByte(MCSM0_REGADDR, u8);   //Main Radio Control State Machine Configuration
    cc1101_writeSingleByte(FOCCFG_REGADDR, 0x14);  //Frequency Offset Compensation Configuration
    cc1101_writeSingleByte(AGCCTRL0_REGADDR, 0x92);  //AGC Control
    cc1101_writeSingleByte(WORCTRL_REGADDR, 0xFB); //Wake On Radio Control
    // Studio wanted FREND0 to 0x11, and in OOK, PATABLE[0] is logic 0, PATABLE[1] is logic 1
    cc1101_writeSingleByte(FREND0_REGADDR, 0x11);  //Front End TX Configuration: PA_PWR[2:0] index in b2:0
    cc1101_writeSingleByte(FSCAL3_REGADDR, 0xE9);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL2_REGADDR, 0x2A);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL1_REGADDR, 0x00);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL0_REGADDR, 0x1F);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(TEST2_REGADDR, 0x81);   //Various Test Settings
    cc1101_writeSingleByte(TEST1_REGADDR, 0x35);   //Various Test Settings
    cc1101_writeSingleByte(TEST0_REGADDR, 0x09);   //Various Test Settings
//    uint8_t paTableBytes[8] = { 0x00, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // 0x1D for -15dBm
//    uint8_t paTableBytes[8] = { 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // 0x03 for -30dBm TX power
//    cc1101_writeBurst(PATABLE_REGADDR, paTableBytes, 8);
    setTxPower(desiredTx_dBm);

    NRF_LOG_WARNING("CC1101 setup433PacketTx done, TX dBm %d", m_desiredTxdBm);
}
#endif // #if BASIC_ASK_433

#if SENSITIVE_433
static void setupPacketRadio(int8_t desiredTx_dBm)
{
    if (statusByteState_idle != CHIPSTATUSBYTE_STATE(m_lastChipStatusByte))
    {
        cc1101_setIdle(true); // Idle it before writing configs
    }

    /* Rf settings for CC1101, exported April 17, 2024
     * 432.99MHz, XTAL 26MHz
     */
    //    cc1101_writeSingleByte(IOCFG0_REGADDR, 0x06);  //GDO0 Output Pin Configuration
#if GDO2_STATUS
    // Set up GDO2 to assert at beginning and de-assert at end of packet.
    cc1101_writeSingleByte(IOCFG2_REGADDR, IOCFGx_GDOx_CFG(gdox_syncwordToEndOfPkt));
#else
    cc1101_writeSingleByte(IOCFG2_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
#endif // #if GDO2_STATUS
    // Set all pins to tri-state
    cc1101_writeSingleByte(IOCFG1_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
    cc1101_writeSingleByte(IOCFG0_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));

    cc1101_writeSingleByte(FIFOTHR_REGADDR, 0x47);   //RX FIFO and TX FIFO Thresholds
    cc1101_writeSingleByte(SYNC1_REGADDR, 0xD3);
    cc1101_writeSingleByte(SYNC0_REGADDR, 0x91);
#if PKT_SIZE_FIXED
    cc1101_writeSingleByte(PKTLEN_REGADDR, PKT_LEN);
#endif // #if PKT_SIZE_FIXED

    uint8_t u8 = PKTCTRL1_APPEND_STATUS;
    u8 |= PKTCTRL1_ADDR_CHK(addrChk_none); // No addresses yet
    cc1101_writeSingleByte(PKTCTRL1_REGADDR, u8);

    // normal mode in b5:4
    //    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x05);   //Packet Automation Control
    u8 = 0; // TODO test PKTCTRL0_WHITE_DATA;
    u8 |= PKTCTRL0_CRC_EN;
#if PKT_SIZE_FIXED
    u8 |= PKTCTRL0_LENGTH_CFG(pktCtrl0Len_fixed);
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, u8);   //Packet Automation Control
#else
#error "define"
#endif // #if PKT_SIZE_FIXED

    cc1101_writeSingleByte(ADDR_REGADDR, 0x00);
    cc1101_writeSingleByte(CHANNR_REGADDR, 0x00);

    cc1101_writeSingleByte(FSCTRL1_REGADDR, 0x06);   //Frequency Synthesizer Control
    cc1101_writeSingleByte(FSCTRL0_REGADDR, 0x00);
    // Want 433.0MHz, because boards are that freq
    cc1101_writeSingleByte(FREQ2_REGADDR, 0x10);   //Frequency Control Word, High Byte
    cc1101_writeSingleByte(FREQ1_REGADDR, 0xA7);   //Frequency Control Word, Middle Byte
    cc1101_writeSingleByte(FREQ0_REGADDR, 0x62);   //Frequency Control Word, Low Byte

    cc1101_writeSingleByte(MDMCFG4_REGADDR, 0xF5);   //Modem Configuration
    cc1101_writeSingleByte(MDMCFG3_REGADDR, 0x83);   //Modem Configuration
//    Set mode to GFSK
    cc1101_writeSingleByte(MDMCFG2_REGADDR, 0x13);   //Modem Configuration
    cc1101_writeSingleByte(MDMCFG1_REGADDR, 0x22);
    cc1101_writeSingleByte(MDMCFG0_REGADDR, 0xF8);
    cc1101_writeSingleByte(DEVIATN_REGADDR, 0x31);   //Modem Deviation Setting
    cc1101_writeSingleByte(MCSM2_REGADDR, 0x07);

    // Added MCSM1 to go automatically from TX to RX when sending.
    u8 = MCSM1_CCA_MODE(ccaMode_RssiBelowThresholdUnlessRx);
    u8 |= MCSM1_RXOFF_MODE(rxOff_idle); // Go to Idle when done with RX
    u8 |= MCSM1_TXOFF_MODE(txOff_idle); // Go to TX when done with RX
    cc1101_writeSingleByte(MCSM1_REGADDR, u8);

    // generated: cc1101_writeSingleByte(MCSM0_REGADDR, 0x18);   //Main Radio Control State Machine Configuration
    // Generated had autocal when idle to RX or TX 0x10, and PO_TIMEOUT of 0b10 (0x08)
//    u8 = MCSM0_FS_AUTOCAL(faAutocal_idleToRxOrTx);
//    u8 |= MCSM0_PO_TIMEOUT(poTimeout_64counts);
//    cc1101_writeSingleByte(MCSM0_REGADDR, u8);   //Main Radio Control State Machine Configuration
    cc1101_writeSingleByte(MCSM0_REGADDR, 0x18);

    cc1101_writeSingleByte(FOCCFG_REGADDR, 0x16);   //Frequency Offset Compensation Configuration
    cc1101_writeSingleByte(BSCFG_REGADDR, 0x6C);
    cc1101_writeSingleByte(AGCCTRL2_REGADDR, 0x03);
    cc1101_writeSingleByte(AGCCTRL1_REGADDR, 0x40);
    cc1101_writeSingleByte(AGCCTRL0_REGADDR, 0x91);
    cc1101_writeSingleByte(WOREVT1_REGADDR, 0x87);
    cc1101_writeSingleByte(WOREVT0_REGADDR, 0x6B);
    cc1101_writeSingleByte(WORCTRL_REGADDR, 0xFB);   //Wake On Radio Control
    cc1101_writeSingleByte(FREND1_REGADDR, 0x56);
    cc1101_writeSingleByte(FREND0_REGADDR, 0x10);
    cc1101_writeSingleByte(FSCAL3_REGADDR, 0xE9);   //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL2_REGADDR, 0x2A);   //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL1_REGADDR, 0x00);   //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL0_REGADDR, 0x1F);   //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(RCCTRL1_REGADDR, 0x41);
    cc1101_writeSingleByte(RCCTRL0_REGADDR, 0x00);
    cc1101_writeSingleByte(FSTEST_REGADDR, 0x59);
    cc1101_writeSingleByte(PTEST_REGADDR, 0x7F);
    cc1101_writeSingleByte(AGCTEST_REGADDR, 0x3F);
    cc1101_writeSingleByte(TEST2_REGADDR, 0x81);   //Various Test Settings
    cc1101_writeSingleByte(TEST1_REGADDR, 0x35);   //Various Test Settings
    cc1101_writeSingleByte(TEST0_REGADDR, 0x09);   //Various Test Settings

    setTxPower(desiredTx_dBm);
    NRF_LOG_WARNING("CC1101 setupSensitive433 done, TX dBm %d", m_desiredTxdBm);
}
#endif // #if SENSITIVE_433

#if FORUM_433
static void setupPacketRadio(int8_t desiredTx_dBm)
{
// Rf settings for CC1101
//    0x06,  // IOCFG0        GDO0 Output Pin Configuration
    cc1101_writeSingleByte(IOCFG2_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
    cc1101_writeSingleByte(IOCFG1_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
    cc1101_writeSingleByte(IOCFG0_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));

//    0x47,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
    cc1101_writeSingleByte(FIFOTHR_REGADDR, 0x47);

//    0x05,  // PKTCTRL0      Packet Automation Control

    // normal mode in b5:4
    //    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x05);   //Packet Automation Control
    uint8_t u8 = 0; // TODO | PKTCTRL0_WHITE_DATA;
    u8 |= PKTCTRL0_CRC_EN;
#if PKT_SIZE_FIXED
    u8 |= PKTCTRL0_LENGTH_CFG(pktCtrl0Len_fixed);
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, u8);   //Packet Automation Control
    cc1101_writeSingleByte(PKTLEN_REGADDR, PKT_LEN);
#else
#error "define"
#endif // #if PKT_SIZE_FIXED

//    0x06,        // FSCTRL1       Frequency Synthesizer Control
    cc1101_writeSingleByte(FSCTRL1_REGADDR, 0x06);
    /*0x22,        // FREQ2         Frequency Control Word, High Byte
     0xB6,        // FREQ1         Frequency Control Word, Middle Byte
     0x27,        // FREQ0         Frequency Control Word, Low Byte
     The above is for 915MHz, we need 433
     */
    cc1101_writeSingleByte(FREQ2_REGADDR, 0x10);   //Frequency Control Word, High Byte
    cc1101_writeSingleByte(FREQ1_REGADDR, 0xA7);   //Frequency Control Word, Middle Byte
    cc1101_writeSingleByte(FREQ0_REGADDR, 0x62);   //Frequency Control Word, Low Byte

//    0xCA,        // MDMCFG4       Modem Configuration
    cc1101_writeSingleByte(MDMCFG4_REGADDR, 0xCA);
//    0x83,        // MDMCFG3       Modem Configuration
    cc1101_writeSingleByte(MDMCFG3_REGADDR, 0x83);
//    0x13,        // MDMCFG2       Modem Configuration
    cc1101_writeSingleByte(MDMCFG2_REGADDR, 0x13);
//    0x35,        // DEVIATN       Modem Deviation Setting
    cc1101_writeSingleByte(DEVIATN_REGADDR, 0x35);

    // Added MCSM1 to go automatically from TX to RX when sending.
    u8 = MCSM1_CCA_MODE(ccaMode_RssiBelowThresholdUnlessRx);
    u8 |= MCSM1_RXOFF_MODE(rxOff_idle); // Go to Idle when done with RX
    u8 |= MCSM1_TXOFF_MODE(txOff_Rx); // Go to TX when done with RX
    cc1101_writeSingleByte(MCSM1_REGADDR, u8);

//    0x18,        // MCSM0         Main Radio Control State Machine Configuration
    cc1101_writeSingleByte(MCSM0_REGADDR, 0x18); // 0x18 is cal on idle to RX/TX, and PO timeout 64
//    0x16,        // FOCCFG        Frequency Offset Compensation Configuration
    cc1101_writeSingleByte(FOCCFG_REGADDR, 0x16);
//    0x43,        // AGCCTRL2      AGC Control
    cc1101_writeSingleByte(AGCCTRL2_REGADDR, 0x43);
//    0xFB,        // WORCTRL       Wake On Radio Control
    cc1101_writeSingleByte(WORCTRL_REGADDR, 0xFB);
//    0xE9,        // FSCAL3        Frequency Synthesizer Calibration
//    0x2A,        // FSCAL2        Frequency Synthesizer Calibration
//    0x00,        // FSCAL1        Frequency Synthesizer Calibration
//    0x1F,        // FSCAL0        Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL3_REGADDR, 0xE9);
    cc1101_writeSingleByte(FSCAL2_REGADDR, 0x2A);
    cc1101_writeSingleByte(FSCAL1_REGADDR, 0x00);
    cc1101_writeSingleByte(FSCAL0_REGADDR, 0x1F);
//    0x81,        // TEST2         Various Test Settings
//    0x35,        // TEST1         Various Test Settings
//    0x09,        // TEST0         Various Test Settings
    cc1101_writeSingleByte(TEST2_REGADDR, 0x81);
    cc1101_writeSingleByte(TEST1_REGADDR, 0x35);
    cc1101_writeSingleByte(TEST0_REGADDR, 0x09);

    setTxPower(desiredTx_dBm);
    NRF_LOG_WARNING("CC1101 setupForum433 done, TX dBm %d", m_desiredTxdBm);

}
#endif // #if FORUM_433

#if SENSITIVE_915
static void setupPacketRadio(int8_t desiredTx_dBm)
{
    if (statusByteState_idle != CHIPSTATUSBYTE_STATE(m_lastChipStatusByte))
    {
        cc1101_setIdle(true); // Idle it before writing configs
    }

    /* Rf settings for CC1101, exported April 18, 2024
     * 915.00MHz, XTAL 26MHz
     */
    //    cc1101_writeSingleByte(IOCFG0_REGADDR, 0x06);  //GDO0 Output Pin Configuration
    // Set all pins to tri-state
    cc1101_writeSingleByte(IOCFG2_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
    cc1101_writeSingleByte(IOCFG1_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
    cc1101_writeSingleByte(IOCFG0_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));

    cc1101_writeSingleByte(FIFOTHR_REGADDR, 0x47);   //RX FIFO and TX FIFO Thresholds
    uint8_t u8 = PKTCTRL1_APPEND_STATUS;
    u8 |= PKTCTRL1_ADDR_CHK(addrChk_none); // No addresses yet
    cc1101_writeSingleByte(PKTCTRL1_REGADDR, u8);
    // normal mode in b5:4
    //    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x05);   //Packet Automation Control
    u8 = 0; // TODO test PKTCTRL0_WHITE_DATA;
    u8 |= PKTCTRL0_CRC_EN;
#if PKT_SIZE_FIXED
    u8 |= PKTCTRL0_LENGTH_CFG(pktCtrl0Len_fixed);
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, u8);   //Packet Automation Control
    cc1101_writeSingleByte(PKTLEN_REGADDR, PKT_LEN);
#else
#error "define"
#endif // #if PKT_SIZE_FIXED

    cc1101_writeSingleByte(FSCTRL1_REGADDR, 0x06);   //Frequency Synthesizer Control
    // Want 433.0MHz, because boards are that freq
    cc1101_writeSingleByte(FREQ2_REGADDR, 0x23);   //Frequency Control Word, High Byte
    cc1101_writeSingleByte(FREQ1_REGADDR, 0x31);   //Frequency Control Word, Middle Byte
    cc1101_writeSingleByte(FREQ0_REGADDR, 0x3B);   //Frequency Control Word, Low Byte

    cc1101_writeSingleByte(MDMCFG4_REGADDR, 0xC7);   //Modem Configuration
    cc1101_writeSingleByte(MDMCFG3_REGADDR, 0x83);   //Modem Configuration
//    Set mode to GFSK
    cc1101_writeSingleByte(MDMCFG2_REGADDR, 0x13);   //Modem Configuration

    cc1101_writeSingleByte(DEVIATN_REGADDR, 0x40);   //Modem Deviation Setting

    // Added MCSM1 to go automatically from TX to RX when sending.
    u8 = MCSM1_CCA_MODE(ccaMode_RssiBelowThresholdUnlessRx);
    u8 |= MCSM1_RXOFF_MODE(rxOff_idle); // Go to Idle when done with RX
    u8 |= MCSM1_TXOFF_MODE(txOff_Rx); // Go to TX when done with RX
    cc1101_writeSingleByte(MCSM1_REGADDR, u8);

    // Generated had autocal when idle to RX or TX 0x10, and PO_TIMEOUT of 0b10 (0x08)
    cc1101_writeSingleByte(MCSM0_REGADDR, 0x18);    //Main Radio Control State Machine Configuration

    cc1101_writeSingleByte(FOCCFG_REGADDR, 0x16);   //Frequency Offset Compensation Configuration

    cc1101_writeSingleByte(AGCCTRL2_REGADDR, 0x43);  //AGC Control

    cc1101_writeSingleByte(WORCTRL_REGADDR, 0xFB);   //Wake On Radio Control
    cc1101_writeSingleByte(FSCAL3_REGADDR, 0xE9);   //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL2_REGADDR, 0x2A);   //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL1_REGADDR, 0x00);   //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL0_REGADDR, 0x1F);   //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(TEST2_REGADDR, 0x81);   //Various Test Settings
    cc1101_writeSingleByte(TEST1_REGADDR, 0x35);   //Various Test Settings
    cc1101_writeSingleByte(TEST0_REGADDR, 0x09);   //Various Test Settings

    setTxPower(desiredTx_dBm);
    NRF_LOG_WARNING("CC1101 setupSensitive915 done, TX dBm %d", m_desiredTxdBm);
}

#endif // #if SENSITIVE_915

#if FORUM_915
//https://e2e.ti.com/support/wireless-connectivity/sub-1-ghz-group/sub-1-ghz/f/sub-1-ghz-forum/1275571/cc1101-unable-to-recevie-packets-with-rssi-value-less-than--70-dbm-at-904mhz
static void setupPacketRadio(int8_t desiredTx_dBm)
{
// Rf settings for CC1101
//    0x06,  // IOCFG0        GDO0 Output Pin Configuration
    cc1101_writeSingleByte(IOCFG2_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
    cc1101_writeSingleByte(IOCFG1_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));
    cc1101_writeSingleByte(IOCFG0_REGADDR, IOCFGx_GDOx_CFG(gdox_3state));

//    0x47,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
    cc1101_writeSingleByte(FIFOTHR_REGADDR, 0x47);

//    0x05,  // PKTCTRL0      Packet Automation Control

    // normal mode in b5:4
    //    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x05);   //Packet Automation Control
    uint8_t u8 = 0; // TODO | PKTCTRL0_WHITE_DATA;
    u8 |= PKTCTRL0_CRC_EN;
#if PKT_SIZE_FIXED
    u8 |= PKTCTRL0_LENGTH_CFG(pktCtrl0Len_fixed);
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, u8);   //Packet Automation Control
    cc1101_writeSingleByte(PKTLEN_REGADDR, PKT_LEN);
#else
#error "define"
#endif // #if PKT_SIZE_FIXED

//    0x06,        // FSCTRL1       Frequency Synthesizer Control
    cc1101_writeSingleByte(FSCTRL1_REGADDR, 0x06);
    /*0x22,        // FREQ2         Frequency Control Word, High Byte
     0xB6,        // FREQ1         Frequency Control Word, Middle Byte
     0x27,        // FREQ0         Frequency Control Word, Low Byte
     The above is for 915MHz
     */
    cc1101_writeSingleByte(FREQ2_REGADDR, 0x22);   //Frequency Control Word, High Byte
    cc1101_writeSingleByte(FREQ1_REGADDR, 0xB6);   //Frequency Control Word, Middle Byte
    cc1101_writeSingleByte(FREQ0_REGADDR, 0x27);   //Frequency Control Word, Low Byte

//    0xCA,        // MDMCFG4       Modem Configuration
    cc1101_writeSingleByte(MDMCFG4_REGADDR, 0xCA);
//    0x83,        // MDMCFG3       Modem Configuration
    cc1101_writeSingleByte(MDMCFG3_REGADDR, 0x83);
//    0x13,        // MDMCFG2       Modem Configuration
    cc1101_writeSingleByte(MDMCFG2_REGADDR, 0x13);
//    0x35,        // DEVIATN       Modem Deviation Setting
    cc1101_writeSingleByte(DEVIATN_REGADDR, 0x35);

    // Added MCSM1 to go automatically from TX to RX when sending.
    u8 = MCSM1_CCA_MODE(ccaMode_RssiBelowThresholdUnlessRx);
    u8 |= MCSM1_RXOFF_MODE(rxOff_idle); // Go to Idle when done with RX
    u8 |= MCSM1_TXOFF_MODE(txOff_Rx); // Go to TX when done with RX
    cc1101_writeSingleByte(MCSM1_REGADDR, u8);

//    0x18,        // MCSM0         Main Radio Control State Machine Configuration
    cc1101_writeSingleByte(MCSM0_REGADDR, 0x18); // 0x18 is cal on idle to RX/TX, and PO timeout 64
//    0x16,        // FOCCFG        Frequency Offset Compensation Configuration
    cc1101_writeSingleByte(FOCCFG_REGADDR, 0x16);
//    0x43,        // AGCCTRL2      AGC Control
    cc1101_writeSingleByte(AGCCTRL2_REGADDR, 0x43);
//    0xFB,        // WORCTRL       Wake On Radio Control
    cc1101_writeSingleByte(WORCTRL_REGADDR, 0xFB);
//    0xE9,        // FSCAL3        Frequency Synthesizer Calibration
//    0x2A,        // FSCAL2        Frequency Synthesizer Calibration
//    0x00,        // FSCAL1        Frequency Synthesizer Calibration
//    0x1F,        // FSCAL0        Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL3_REGADDR, 0xE9);
    cc1101_writeSingleByte(FSCAL2_REGADDR, 0x2A);
    cc1101_writeSingleByte(FSCAL1_REGADDR, 0x00);
    cc1101_writeSingleByte(FSCAL0_REGADDR, 0x1F);
//    0x81,        // TEST2         Various Test Settings
//    0x35,        // TEST1         Various Test Settings
//    0x09,        // TEST0         Various Test Settings
    cc1101_writeSingleByte(TEST2_REGADDR, 0x81);
    cc1101_writeSingleByte(TEST1_REGADDR, 0x35);
    cc1101_writeSingleByte(TEST0_REGADDR, 0x09);

    setTxPower(desiredTx_dBm);
    NRF_LOG_WARNING("CC1101 setupForum433 done, TX dBm %d", m_desiredTxdBm);

}
#endif // #if FORUM_915

#if locustcox_433
static void setupPacketRadio(int8_t desiredTx_dBm)
{
    char PA[] = { 0x60 };
    const unsigned char PA_LEN = 1;
    /* Sync word qualifier mode = 30/32 sync word bits detected */
    /* CRC autoflush = false */
    /* Channel spacing = 199.951172 */
    /* Data format = Normal mode */
    /* Data rate = 512.573 */
    /* RX filter BW = 58.035714 */
    /* PA ramping = false */
    /* Preamble count = 4 */
    /* Whitening = false */
    /* Address config = No address check */
    /* Carrier frequency = 433.999969 */
    /* Device address = 0 */
    /* TX power = 0 */
    /* Manchester enable = true */
    /* CRC enable = true */
    /* Deviation = 5.157471 */
    /* Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word */
    /* Packet length = 255 */
    /* Modulation format = GFSK */
    /* Base frequency = 433.999969 */
    /* Modulated = true */
    /* Channel number = 0 */
//-------------------------------------------------------
//Register Settings for CC1101
//-Generated by the Awesomely Awesome SmartRF(c) studio
//-------------------------------------------------------
    cc1101_writeSingleByte(IOCFG0_REGADDR, 0x06);
    cc1101_writeSingleByte(FIFOTHR_REGADDR, 0x47);
//    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x05);
    uint8_t u8 = 0; // TODO | PKTCTRL0_WHITE_DATA;
    u8 |= PKTCTRL0_CRC_EN;
#if PKT_SIZE_FIXED
    u8 |= PKTCTRL0_LENGTH_CFG(pktCtrl0Len_fixed);
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, u8);   //Packet Automation Control
    cc1101_writeSingleByte(PKTLEN_REGADDR, PKT_LEN);
#else
#error "define"
#endif // #if PKT_SIZE_FIXED
    cc1101_writeSingleByte(FSCTRL1_REGADDR, 0x06);
    cc1101_writeSingleByte(FREQ2_REGADDR, 0x10);
    cc1101_writeSingleByte(FREQ1_REGADDR, 0xB1);
    cc1101_writeSingleByte(FREQ0_REGADDR, 0x3B);

    // Added MCSM1 to go automatically from TX to RX when sending.
    u8 = MCSM1_CCA_MODE(ccaMode_RssiBelowThresholdUnlessRx);
    u8 |= MCSM1_RXOFF_MODE(rxOff_idle); // Go to Idle when done with RX
    u8 |= MCSM1_TXOFF_MODE(txOff_Rx); // Go to TX when done with RX
    cc1101_writeSingleByte(MCSM1_REGADDR, u8);

    cc1101_writeSingleByte(MDMCFG4_REGADDR, 0xFE);
    cc1101_writeSingleByte(MDMCFG3_REGADDR, 0x43);
    cc1101_writeSingleByte(MDMCFG2_REGADDR, 0x1B);
    cc1101_writeSingleByte(DEVIATN_REGADDR, 0x15);
    cc1101_writeSingleByte(MCSM0_REGADDR, 0x18);
    cc1101_writeSingleByte(FOCCFG_REGADDR, 0x16);
    cc1101_writeSingleByte(WORCTRL_REGADDR, 0xFB);
    cc1101_writeSingleByte(FSCAL3_REGADDR, 0xEA);
    cc1101_writeSingleByte(FSCAL2_REGADDR, 0x2A);
    cc1101_writeSingleByte(FSCAL1_REGADDR, 0x00);
    cc1101_writeSingleByte(FSCAL0_REGADDR, 0x1F);
    cc1101_writeSingleByte(TEST2_REGADDR, 0x81);
    cc1101_writeSingleByte(TEST1_REGADDR, 0x35);
    cc1101_writeSingleByte(TEST0_REGADDR, 0x09);

    cc1101_writeBurst(PATABLE_REGADDR, PA, PA_LEN);
}
#endif // #if locustcox_433

#if SENSITIVE_ASK433
static void setupPacketRadio(int8_t desiredTx_dBm)
{
    cc1101_writeSingleByte(IOCFG0_REGADDR, 0x06);  //GDO0 Output Pin Configuration
    // RX filter BW = 58kHz, <=325kHz so set FIFOTHR 0 0x47
    cc1101_writeSingleByte(FIFOTHR_REGADDR, 0x47); //RX FIFO and TX FIFO Thresholds
    cc1101_writeSingleByte(PKTLEN_REGADDR, PKT_LEN);  //Packet Length
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x04);  //Packet Automation Control
    cc1101_writeSingleByte(FSCTRL1_REGADDR, 0x06); //Frequency Synthesizer Control
    cc1101_writeSingleByte(FREQ2_REGADDR, 0x10);   //Frequency Control Word, High Byte
    cc1101_writeSingleByte(FREQ1_REGADDR, 0xA7);   //Frequency Control Word, Middle Byte
    cc1101_writeSingleByte(FREQ0_REGADDR, 0x62);   //Frequency Control Word, Low Byte
    cc1101_writeSingleByte(MDMCFG4_REGADDR, 0xF7); //Modem Configuration
    cc1101_writeSingleByte(MDMCFG3_REGADDR, 0x83); //Modem Configuration
    cc1101_writeSingleByte(MDMCFG2_REGADDR, 0x33); //Modem Configuration
    cc1101_writeSingleByte(DEVIATN_REGADDR, 0x15); //Modem Deviation Setting

    // Added MCSM1 to go automatically from TX to RX when sending.
    uint8_t u8 = MCSM1_CCA_MODE(ccaMode_RssiBelowThresholdUnlessRx);
    u8 |= MCSM1_RXOFF_MODE(rxOff_idle); // Go to Idle when done with RX
    u8 |= MCSM1_TXOFF_MODE(txOff_Rx); // Go to RX when done with TX
    cc1101_writeSingleByte(MCSM1_REGADDR, u8);

    cc1101_writeSingleByte(MCSM0_REGADDR, 0x18);   //Main Radio Control State Machine Configuration
    cc1101_writeSingleByte(FOCCFG_REGADDR, 0x16);  //Frequency Offset Compensation Configuration
    /* From DN022, OOK sensitivity document:
     * The optimum AGC settings change with RX filter bandwidth and data rate, but for OOK/ASK
     * the following has been found to give good results:
     *  AGCCTRL2 = 0x03 to 0x07 (default 0x03)
     *  AGCCTRL1 = 0x00         (default 0x40)
     *  AGCCTRL0 = 0x91 or 0x92 (default 0x91)
     */
    // Default from smartRF was only to set AGCCTRL0: cc1101_writeSingleByte(AGCCTRL0_REGADDR, 0x92);
    // Try the following:
    cc1101_writeSingleByte(AGCCTRL2_REGADDR, 0x02);
    cc1101_writeSingleByte(AGCCTRL1_REGADDR, 0x00);
    cc1101_writeSingleByte(AGCCTRL0_REGADDR, 0x92);

    cc1101_writeSingleByte(WORCTRL_REGADDR, 0xFB); //Wake On Radio Control
    // RX filter BW = 203kHz, >101kHz so set FREND1 = 0xB6
    // RX filter BW = 58kHz, <=101kHz so set FREND1 = 0x56 (default 0x56)
    cc1101_writeSingleByte(FREND1_REGADDR, 0x56);
    cc1101_writeSingleByte(FREND0_REGADDR, 0x11);  //Front End TX Configuration
    cc1101_writeSingleByte(FSCAL3_REGADDR, 0xE9);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL2_REGADDR, 0x2A);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL1_REGADDR, 0x00);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL0_REGADDR, 0x1F);  //Frequency Synthesizer Calibration
    // RX filter BW = 203kHz, <=325kHz so set TEST2 = 0x81
    cc1101_writeSingleByte(TEST2_REGADDR, 0x81);   //Various Test Settings
    // RX filter BW = 203kHz, <=325kHz so set TEST1 = 0x35
    cc1101_writeSingleByte(TEST1_REGADDR, 0x35);   //Various Test Settings
    cc1101_writeSingleByte(TEST0_REGADDR, 0x09);   //Various Test Settings

    setTxPower(desiredTx_dBm);

#if READ_ALL_REGS
            NRF_LOG_WARNING("Regs after setup")
            ;
            readAllRegs();
            m_lastPrint_ms = uptimeCounter_getUptimeMs();
#endif // #if READ_ALL_REGS
}
#endif // #if SENSITIVE_ASK433

/* Packet sniffer settings Feb 06 2024
 # ---------------------------------------------------
 # Packet sniffer settings for CC1101
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

//Async TX should work with EV1527SPI to send EV1527 data
/* SmartRF studio set these values:
 *
 {CC1101_IOCFG2,      0x0B},
 {CC1101_IOCFG0,      0x0C},
 {CC1101_FIFOTHR,     0x47},
 {CC1101_PKTCTRL0,    0x12},
 {CC1101_FREQ2,       0x10},
 {CC1101_FREQ1,       0xA7},
 {CC1101_FREQ0,       0x62},
 {CC1101_MDMCFG4,     0x87},
 {CC1101_MDMCFG3,     0x0C},
 {CC1101_MDMCFG2,     0x30},
 {CC1101_MDMCFG1,     0x00},
 {CC1101_FREND0,      0x11},
 {CC1101_FSCAL3,      0xE9},
 {CC1101_FSCAL2,      0x2A},
 {CC1101_FSCAL1,      0x00},
 {CC1101_FSCAL0,      0x1F},
 {CC1101_TEST2,       0x81},
 {CC1101_TEST1,       0x35},
 {CC1101_TEST0,       0x09},

 cc1101_writeSingleByte(FIFOTHR,0x47);//RX FIFO and TX FIFO Thresholds
 cc1101_writeSingleByte(MDMCFG2,0x30);//Modem Configuration
 cc1101_writeSingleByte(FREND0,0x11); //Front End TX Configuration
 cc1101_writeSingleByte(FSCAL3,0xEA); //Frequency Synthesizer Calibration
 cc1101_writeSingleByte(FSCAL2,0x2A); //Frequency Synthesizer Calibration
 cc1101_writeSingleByte(FSCAL1,0x00); //Frequency Synthesizer Calibration
 cc1101_writeSingleByte(FSCAL0,0x1F); //Frequency Synthesizer Calibration
 cc1101_writeSingleByte(TEST2,0x81);  //Various Test Settings
 cc1101_writeSingleByte(TEST1,0x35);  //Various Test Settings

 *
 */
static void setupAsyncTx(void)
{
    uint8_t u8 = 0;
    // Set up IO2 for 3-state
    /* Studio sent 0x0B, which means:
     * b7 unused, zero
     * b6 GDOx_INV, zero
     * b5:0 usage: 0x0B = gdox_serialClock
     * We don't need that, we are doing async
     */
    u8 = IOCFGx_GDOx_CFG(gdox_3state);
    cc1101_writeSingleByte(IOCFG2_REGADDR, u8);
    // Set GDO1 to 3-state as well, make all GDO pins low strength
    u8 = IOCFGx_GDOx_CFG(gdox_3state);
    cc1101_writeSingleByte(IOCFG1_REGADDR, u8);
    // Set up IO0 for data to TX in async mode
    /* Studio sent 0x0C, which is gdox_syncSerialDataOutput
     * I don't think it matters here, but we should set
     */
    u8 = IOCFGx_GDOx_CFG(gdox_3state);
    cc1101_writeSingleByte(IOCFG0_REGADDR, 0x0C);
    /* Studio sent 0x47, which means
     * b6: ADC retention set, for <100kbps baud, good.
     * b3:0 0b0111, fine.
     */
    cc1101_writeSingleByte(FIFOTHR_REGADDR, 0x47); //RX FIFO and TX FIFO Thresholds
    /* Studio sent 0x35 which means:
     * b7 clear, good
     * b6 white_data clear
     * b5:4 0b11, for async serial
     * b3 not used, zero, good.
     * b2 CRC enable, enabled, TURN OFF, change to 0b0
     * b1:0 LEN_CFG, 0b01=variable
     * So we want 0x31
     */
    u8 = PKTCTRL0_PKT_FORMAT_WRITE(pktCtrl0Format_asyncSerial);
//    u8 |= PKTCTRL0_CRC_EN;
    u8 |= PKTCTRL0_LENGTH_CFG(pktCtrl0Len_infinite);
    cc1101_writeSingleByte(PKTCTRL0_REGADDR, 0x32); //Packet Automation Control

    cc1101_writeSingleByte(FREQ2_REGADDR, 0x10);   //Frequency Control Word, High Byte
    cc1101_writeSingleByte(FREQ1_REGADDR, 0xB1);   //Frequency Control Word, Middle Byte
    cc1101_writeSingleByte(FREQ0_REGADDR, 0x3A);   //Frequency Control Word, Low Byte

    cc1101_writeSingleByte(MDMCFG4_REGADDR, 0x87); //Modem Configuration
    cc1101_writeSingleByte(MDMCFG3_REGADDR, 0x0C); //Modem Configuration
    cc1101_writeSingleByte(MDMCFG2_REGADDR, 0x30); //Modem Configuration
    /* Studio sent 0x87, which means
     * b5:4, datasheet says to get from studio
     * b1:0 patable index for TXing a '1', so index 1
     */
    cc1101_writeSingleByte(FREND0_REGADDR, 0x11);  //Front End TX Configuration
    cc1101_writeSingleByte(FSCAL3_REGADDR, 0xE9);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL2_REGADDR, 0x2A);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL1_REGADDR, 0x00);  //Frequency Synthesizer Calibration
    cc1101_writeSingleByte(FSCAL0_REGADDR, 0x1F);  //Frequency Synthesizer Calibration

    cc1101_writeSingleByte(TEST2_REGADDR, 0x81);   //Various Test Settings
    cc1101_writeSingleByte(TEST1_REGADDR, 0x35);   //Various Test Settings
    cc1101_writeSingleByte(TEST0_REGADDR, 0x09);   //Various Test Settings

    // PATABLE for TX
    uint8_t paTableBytes[8] = { 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    cc1101_writeBurst(PATABLE_REGADDR, paTableBytes, 8);
}

static void initPacketReceiver(void)
{
    m_rxSettings.pPacket = m_rxPacketBuffer;
    m_rxSettings.readFromChip = true;
    m_rxSettings.packetBufIndex = 0;
    m_rxSettings.packetBufLen = sizeof(m_rxPacketBuffer);
    m_rxSettings.rxState = packetRxState_awaitingPacketStart;
}

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
    if (pktCtrl0Len_fixed == pSettings->packetLenCfg)
    { // Read expected packet length
        cc1101_readSingleByte(PKTLEN_REGADDR, &pSettings->expectedPacketLen);
        NRF_LOG_DEBUG("Read in settings: Fixed packet len of %d", pSettings->expectedPacketLen);
    }
    else if (pktCtrl0Len_variable == pSettings->packetLenCfg)
    { // Read in max length, if desired
        NRF_LOG_DEBUG("Read in settings: Variable packet len");
    }
    else
    {
        NRF_LOG_ERROR("Read in settings: UNSUPPORTED packet len config %d", pSettings->packetLenCfg);
    }
    pSettings->readFromChip = false;
}

static void readRSSIandLQI(packetRxSettings_t* pSettings, uint8_t numFifoBytes)
{ // RSSI first, then LQI and CRC_OK
    cc1101_readSingleByte(RXFIFO_REGADDR, (uint8_t*)&pSettings->RSSI);
    uint8_t u8;
    cc1101_readSingleByte(RXFIFO_REGADDR, &u8);
    pSettings->LQI = LQI_LQI_EST(u8);
    pSettings->CRCok = (u8 & LQI_CRC_OK) ? true : false;
    pSettings->rxState = packetRxState_finishedWithPacket;
    NRF_LOG_DEBUG("Read RSSI %d ,LQI %d, CRC %s", pSettings->RSSI, pSettings->LQI,
                  pSettings->CRCok ? "ok or na" : "not ok");

    uint8_t u82;
    cc1101_readSingleByte(LQI_REGADDR, &u82);
    if (u8 != u82)
    {
        NRF_LOG_WARNING(" Read LQI regaddr: LQI %d, CRC %s", LQI_LQI_EST(u8), (u8&LQI_CRC_OK)?"ok":"not ok");
    }
}

static void readPacketBytes(packetRxSettings_t* pSettings, uint8_t numFifoBytes)
{
    uint32_t numExpected = pSettings->expectedPacketLen - pSettings->packetBufIndex;
    if (numFifoBytes < numExpected)
    { // If bytes are still coming in, don't empty the FIFO completely (section 20)
        cc1101_readBurst(RXFIFO_REGADDR, &pSettings->pPacket[pSettings->packetBufIndex], numFifoBytes - 1);
        pSettings->rxState = packetRxState_awaitingPacketBytes;
        pSettings->packetBufIndex += numFifoBytes - 1;
        NRF_LOG_INFO("fixedLen read %d of expected %d bytes, try again later", pSettings->packetBufIndex,
                     pSettings->expectedPacketLen);
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
                readRSSIandLQI(pSettings, numFifoBytes);
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

static void parsePacketAndLQI(int8_t RSSI, uint8_t LQI, bool crcOK, uint8_t* pData, uint32_t len)
{
//    NRF_LOG_INFO("  RX %d-byte packet, RSSI %d, LQI %d, CRC %d.", len, RSSI, LQI, crcOK);
#ifdef UART_TX_PIN
    char strBuf[256];
    int strLen;

//    strLen = snprintf(strBuf, sizeof(strBuf), "%d", RSSI);
//    _4digit7seg_writeStr(strBuf);

//    strLen = snprintf(strBuf, sizeof(strBuf), "RX %d-byte packet, RSSI %d, LQI %d, CRC %d:\n", len, RSSI,
//                      LQI,
//                      crcOK);
//    if (0 < strLen)
//    {
//        uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
//    }
#endif // #ifdef UART_TX_PIN

    if (1 == len)
    {
        NRF_LOG_INFO(" RX 1-byte packet, TX power %d, RSSI %d, LQI %d, CRC %d.", (int8_t )pData[0], len, RSSI, LQI,
                     crcOK);
#ifdef UART_TX_PIN
        strLen = snprintf(strBuf, sizeof(strBuf), " RX 1-byte packet, TX power %d, RSSI %d, LQI %d, CRC %d.\n",
                          (int8_t)pData[0],
                          RSSI, LQI, crcOK);
        if (0 < strLen)
        {
            uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
        }
#endif // #ifdef UART_TX_PIN

    }
    if (2 <= len)
    { // 2nd byte holds the command on command, or ACKer's RSSI of our first command
        int8_t theirTxdBm = (int8_t)pData[0];
        cc1101_setOutputPower(theirTxdBm);
        switch (m_opMode)
        {
            case cc1101_packetRX:
                // If we are the motor driver, we receive the state to set
                globalInts_setMachineState((machineState_t)pData[1]);
                NRF_LOG_INFO(" RX command to mode %d. Sending ack.", (machineState_t )pData[1])
                ;
                NRF_LOG_INFO(" They sent at %d, RSSI %d, LQI %d, CRC %d.", theirTxdBm, RSSI, LQI, crcOK)
                ;
#ifdef UART_TX_PIN
                strLen = snprintf(strBuf, sizeof(strBuf), " RX CMD %d, TX %d dBm, RSSI %d, LQI %d, CRC %d.\n",
                                  pData[1],
                                  (int8_t)pData[0],
                                  RSSI,
                                  LQI, crcOK);
                if (0 < strLen)
                {
                    uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
                }
#endif // #ifdef UART_TX_PIN

                // Send an ACK when we RX a packet
                cc1101_sendPacket((uint8_t)RSSI);
            break;
            case cc1101_packetTX:
                // If we are the remote controller, we receive the RSSI that the driver heard our packet at.

                NRF_LOG_INFO(" RX ACK packet. Receiver heard us at %d.", (int8_t )pData[1])
                ;
                NRF_LOG_INFO(" They sent at %d, RSSI %d, LQI %d, CRC %d.", theirTxdBm, RSSI, LQI, crcOK)
                ;
#ifdef UART_TX_PIN
                strLen = snprintf(strBuf, sizeof(strBuf),
                                  " RX ACK of CMD, received at %d. ACK TX power %d, RSSI %d, LQI %d, CRC %d.\n",
                                  (int8_t)pData[1],
                                  (int8_t)pData[0],
                                  RSSI,
                                  LQI,
                                  crcOK);
                if (0 < strLen)
                {
                    uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
                }
#endif // #ifdef UART_TX_PIN
            break;
            default:
                NRF_LOG_INFO("  RX %d-byte packet, RSSI %d, LQI %d, CRC %d.", len, RSSI, LQI, crcOK)
                ;
            break;
        }

    }
}
static void tryReceivePacket(packetRxSettings_t* pSettings)
{
    uint8_t regByte;
    if (NULL == pSettings)
    {
        NRF_LOG_ERROR("NULL pSettings");
        return;
    }
    if (pSettings->readFromChip)
    {
        readInSettings(pSettings);
    }
    if (pSettings->packetBufIndex == pSettings->expectedPacketLen)
    {
        NRF_LOG_ERROR("Doesn't make sense, resetting receive state");
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
//        NRF_LOG_DEBUG("0 RXbytes");
        return;
    }
    // If here, we have bytes to read in RX FIFO
//    NRF_LOG_INFO("have %d bytes in RXFIFO", numFifoBytes);
    switch (pSettings->packetLenCfg)
    {
        case pktCtrl0Len_fixed:
            readPacketBytes(pSettings, numFifoBytes);
        break;
        case pktCtrl0Len_variable:
            NRF_LOG_WARNING("WHY are we in readVariablePacket?")
            ;
            readVariablePacket(pSettings, numFifoBytes);
        break;
        default:
            NRF_LOG_ERROR("Packet type %d not handled yet", pSettings->packetLenCfg)
            ;
        break;
    }
    // Try and parse it
    if (packetRxState_finishedWithPacket == m_rxSettings.rxState)
    {
        if (m_rxSettings.packetBufIndex != m_rxSettings.expectedPacketLen)
        {
            NRF_LOG_WARNING("Something wrong, didn't receive expected bytes, flush and idle");
        }
        else
        { // Parse it, if it looks OK.
            if (m_rxSettings.RSSIandLQIAppended)
            {
                parsePacketAndLQI(m_rxSettings.RSSI, m_rxSettings.LQI, m_rxSettings.CRCok, m_rxSettings.pPacket,
                                  m_rxSettings.packetBufLen);
            }
            else
            {
                NRF_LOG_WARNING("Packet without LQI, TODO parse");
            }

        }
//        cc1101_setIdle(true);
        m_rxSettings.rxState = packetRxState_awaitingPacketStart;
        m_rxSettings.packetBufIndex = 0;
    }
    else
    { // If not in finished state, check if length is equal, that would be an error
        if (m_rxSettings.packetBufIndex >= m_rxSettings.expectedPacketLen)
        {
            NRF_LOG_WARNING("Received %d of %d bytes, but state still %d, reset parser",
                            m_rxSettings.packetBufIndex,
                            m_rxSettings.expectedPacketLen,
                            m_rxSettings.rxState);
            // reset parser state
            m_rxSettings.rxState = packetRxState_awaitingPacketStart;
            m_rxSettings.packetBufIndex = 0;
        }
        else
        {
            // Keep waiting for packet, TODO timeout?
        }
    }
}

static void asyncTxPoll(chipStatusByteState_t currentState)
{
    switch (currentState)
    {
        case statusByteState_tx:
            // Gets into this state from someone telling it to send a packet.
            if (statusByteState_tx != m_lastReadStatusByteState)
            {
                NRF_LOG_INFO("Went into TX state, good");
            }
        break;
        case statusByteState_rx:
            NRF_LOG_ERROR("Should not be in RX, idling")
            ;
            cc1101_setIdle(true);
        break;
        case statusByteState_idle:
            // should not be in idle, go to TX state
            NRF_LOG_ERROR("Should NOT be in idle state, goto TX")
            ;
            cc1101_strobe(STROBE_STX);
        break;
        case statusByteState_rxOverflow:
            NRF_LOG_ERROR("Detected RX overflow, idling")
            ;
            cc1101_setIdle(true);
        break;
        case statusByteState_txUnderflow:
            NRF_LOG_ERROR("Detected TX underflow, idling")
            ;
            cc1101_setIdle(true);
        break;
        default:
            //ignore for now
        break;
    }
}

static void packetRxPoll(chipStatusByteState_t currentState)
{
    switch (currentState)
    {
        case statusByteState_tx:
            if (1 == m_sendState)
            {
                // Gets into this state from someone telling it to send a packet.
                if (m_inState_ms >= TX_TIMEOUT_MS)
                { // Shouldn't take a long time to send
                    m_sendState = 0;
                    NRF_LOG_ERROR("Timed out in TX state, idling")
                    ;
                    cc1101_setIdle(true);
                }
            }
            else
            {
                NRF_LOG_ERROR("In TX state, should NOT be here, idling.")
                ;
                cc1101_setIdle(true);
            }
        break;
        case statusByteState_rx:
            // If in receiving state
            if (0 == m_inState_ms)
            { // Reset RX parser on entry, then try and receive packet
                m_rxSettings.rxState = packetRxState_awaitingPacketStart;
                m_rxSettings.packetBufIndex = 0;
            }
            if (m_inState_ms >= 5000)
            { // It should transition to IDLE when done receiving a packet
                NRF_LOG_WARNING("Timed out in RX state, idling")
                ;
#ifdef UART_TX_PIN
                uartTerminal_enqueueToUSB((uint8_t*)"Timed out in RX, idling\n",
                                          strlen("Timed out in RX, idling\n"));
#endif // #ifdef UART_TX_PIN
                cc1101_setIdle(true);
            }
        break;
        case statusByteState_idle:
            // Usually here because we received a packet, or because we errored and came here
            //            NRF_LOG_DEBUG("Idle Check for Received")
            //            ;
            // In RX mode, switch to RX state ASAP
            if (m_newTxPower)
            {
                setTxPower(m_desiredTxdBm);
                m_newTxPower = false;
            }
            if (0 == m_inState_ms)
            { // Reset RX parser on entry, then try and receive packet
                m_rxSettings.rxState = packetRxState_awaitingPacketStart;
                m_rxSettings.packetBufIndex = 0;
            }
            m_sendState = 0;
            // Check if we have received a packet
            tryReceivePacket(&m_rxSettings);

            if (1 == m_sendState)
            {
//                NRF_LOG_INFO("sendState 1, it should go to TX");
                cc1101_strobe(STROBE_NOP);
                NRF_LOG_INFO("is it in TX?");
            }
            else
            {
                NRF_LOG_DEBUG("poller start RX:")
                ;
                cc1101_strobe(STROBE_FLUSHRXFIFO); // Can flush in idle
                cc1101_strobe(STROBE_FLUSHTXFIFO); // Can flush in idle
                cc1101_strobe(STROBE_SRX);
            }
        break;
        case statusByteState_rxOverflow:
            NRF_LOG_ERROR("Detected RX overflow,idling")
            ;
            cc1101_setIdle(true);
        break;
        case statusByteState_txUnderflow:
            NRF_LOG_ERROR("Detected TX underflow, idling")
            ;
            cc1101_setIdle(true);
        break;
        default:
            if (m_inState_ms > 1000)
            {
                NRF_LOG_WARNING("State 0x%x, idling", CHIPSTATUSBYTE_STATE(m_lastChipStatusByte))
                ;
                cc1101_setIdle(true);
            }
        break;
    }
}

static void packetTxPoll(chipStatusByteState_t currentState)
{
    switch (currentState)
    {
        case statusByteState_tx:
            // Gets into this state from someone telling it to send a packet.
            if (m_inState_ms >= TX_TIMEOUT_MS)
            { // Shouldn't take a long time to send, TODO find time
                NRF_LOG_ERROR("Timed out in TX state, idling")
                ;
                cc1101_setIdle(true);
            }
        break;
        case statusByteState_rx:
            if ((m_sendState && (RX_AFTER_TX_TIMEOUT_MS < m_inState_ms))
                    || (0 == m_sendState))
            {
                NRF_LOG_WARNING("packetTx RX state timeout after %d ms, sendState %d, idling", m_inState_ms,
                                m_sendState)
                ;
#ifdef UART_TX_PIN
                char strBuf[96];
                int strLen = snprintf(strBuf, sizeof(strBuf),
                                      "packetTx RX state timeout after %d ms, sendState %d, idling\n",
                                      m_inState_ms,
                                      m_sendState);
                if (0 < strLen)
                {
                    uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
                }
#endif // #ifdef UART_TX_PIN
                cc1101_setIdle(true);
                m_sendState = 0;
            }
        break;
        case statusByteState_idle:
            // Idle until someone calls sendPacket
            if (m_newTxPower)
            {
                setTxPower(m_desiredTxdBm);
                m_newTxPower = false;
            }
            if (0 == m_inState_ms)
            { // Reset RX parser on entry, then try and receive packet
                m_rxSettings.rxState = packetRxState_awaitingPacketStart;
                m_rxSettings.packetBufIndex = 0;
                // Usually here because we received a packet
                tryReceivePacket(&m_rxSettings);
            }
            // Stay in idle until next TX command triggers TX, then TX to RX, then idle again.
        break;
        case statusByteState_rxOverflow:
            NRF_LOG_ERROR("Detected RX overflow, idling")
            ;
            cc1101_setIdle(true);
        break;
        case statusByteState_txUnderflow:
            NRF_LOG_ERROR("Detected TX underflow, idling")
            ;
            cc1101_setIdle(true);
        break;
        default:
            if (m_inState_ms > 1000)
            {
                NRF_LOG_WARNING("State 0x%x, idling", CHIPSTATUSBYTE_STATE(m_lastChipStatusByte))
                ;
                cc1101_setIdle(true);
            }
        break;
    }
}

#if READ_ALL_REGS
static uint32_t m_lastPrint_ms;
#endif // #if READ_ALL_REGS

static void cc1101Poll(void)
{
    if (uptimeCounter_elapsedSince(m_lastPoll_ms) < STATUS_POLL_ITVL_MS)
    { // Don't spam SPI bus, especially when RXing
        return;
    }
// If here, we have expired the timer and can ask it what's up.
    cc1101_strobe(STROBE_NOP); // Strobe NOP to get status
// See if state has changed
    chipStatusByteState_t currentState = CHIPSTATUSBYTE_STATE(m_lastChipStatusByte);
    if (currentState != m_lastReadStatusByteState)
    {
        m_inState_ms = 0;
    }
    else
    { // If in state, increment timer
        m_inState_ms += uptimeCounter_elapsedSince(m_lastPoll_ms);
    }
    switch (m_opMode)
    {
        case cc1101_asyncTX:
            asyncTxPoll(currentState);
        break;
        case cc1101_packetRX:
            packetRxPoll(currentState);
        break;
        case cc1101_packetTX:
            packetTxPoll(currentState);
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
    cc1101_sendPacket(txPacketBytes, PKT_LEN);
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
    m_lastReadStatusByteState = currentState;
    m_lastPoll_ms = uptimeCounter_getUptimeMs();

}

bool cc1101_sendPacket(uint8_t byte)
{
    uint8_t pktBuf[PKT_LEN];
    pktBuf[0] = (uint8_t)m_desiredTxdBm;
    pktBuf[1] = byte;
    bool ret = true;
    ret &= cc1101_writeBurst(TXFIFO_REGADDR, pktBuf, PKT_LEN);
    ret &= cc1101_readSingleByte(TXBYTES_REGADDR, pktBuf);
    if (PKT_LEN == pktBuf[0])
    {
//        NRF_LOG_DEBUG("Sending packet of %d bytes at %d dBm.", PKT_LEN, m_desiredTxdBm);
#if verbose_tx
        NRF_LOG_DEBUG("Sending packet of %d bytes at %d dBm.", PKT_LEN, m_desiredTxdBm);
#ifdef UART_TX_PIN
        uint8_t strBytes[256];
        int printlen = snprintf((char*)strBytes, sizeof(strBytes), "CC1101 sending %d bytes at %d dBm\n", PKT_LEN,
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
    ret &= cc1101_strobe(STROBE_STX);
    m_sendState++;
    return ret;

}

cc1101Mode_t cc1101_readMode(void)
{
    return m_opMode;
}

void cc1101_setOutputPower(int8_t tx_dBm)
{
    if (m_desiredTxdBm != tx_dBm)
    {
        NRF_LOG_INFO("Desire change of power to %d dBm", tx_dBm);
#ifdef UART_TX_PIN
        char strBuf[96];
        int strLen = snprintf(strBuf, sizeof(strBuf), "Desire TX %d dBm\n", tx_dBm);
        if (0 < strLen)
        {
            uartTerminal_enqueueToUSB((const uint8_t*)strBuf, (uint32_t)strLen);
        }
#endif // #ifdef UART_TX_PIN
        m_desiredTxdBm = tx_dBm;
        m_newTxPower = true;
    }
}

bool cc1101_setCloseInRx(fifothr_closeInRx_t atten)
{
    uint8_t oldRegVal;
    if (!cc1101_readSingleByte(FIFOTHR_REGADDR, &oldRegVal))
    {
        return false;
    }
    // Now mask the bits to zero, then OR in the bits we want set
    uint8_t newVal = (uint8_t)((oldRegVal & (~FIFOTHR_CLOSE_IN_RX(0b11))) | FIFOTHR_CLOSE_IN_RX(atten));
    if (newVal == oldRegVal)
    { // Don't need to write
        return true;
    }
    else if (!cc1101_writeSingleByte(FIFOTHR_REGADDR, newVal))
    {
        return false;
    }
    return true;
}

static void setupGDO2Input(uint8_t pin)
{ // GDO2 will go high at beginning of packet RX, then low when done receiving. Sense the first high-going edge.
    nrf_gpio_cfg(pin,
                 NRF_GPIO_PIN_DIR_INPUT,
                 NRF_GPIO_PIN_INPUT_CONNECT,
                 GPIO_PIN_CNF_PULL_Pulldown,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_SENSE_HIGH);
}

void cc1101_init(cc1101Mode_t desired)
{
    spi0_init(); // Make sure pins are muxed and working
    m_lastChipStatusByte = 0x80; // Default to idle, no FIFO bytes
#if READ_ALL_REGS
    NRF_LOG_WARNING("Regs before reset:");
    readAllRegs();
#endif // #if READ_ALL_REGS
    bool testPass = cc1101_reset();
#if READ_ALL_REGS
    NRF_LOG_WARNING("Regs after reset:");
    readAllRegs();
#endif // #if READ_ALL_REGS
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
    m_opMode = cc1101_unknownMode;

    ev1527SPI_turnOff(SPI2_MOSI_PIN); // Default to off.

    switch (desired)
    {
        case cc1101_packetRX:
            setupGDO2Input(CC1101_GDO2_PIN);
            initPacketReceiver();
            setupPacketRadio(-20); // Same settings for Rx side
            m_opMode = cc1101_packetRX;
            m_sendState = 0;
        break;
        case cc1101_packetTX:
            setupGDO2Input(CC1101_GDO2_PIN);
            initPacketReceiver();
            setupPacketRadio(-20); // Same settings for Rx side
            m_opMode = cc1101_packetTX;
            m_sendState = 0;
        break;
        case cc1101_asyncTX:
            // do NOT set up the MOSI pin to drive yet, until we de-init the default CC1101 drive as output.
            setupAsyncTx();
            // Now set up our output pin, which should idle low to not send.
            ev1527SPI_init(SPI2_MOSI_PIN); // init SPI, which idles low.
            cc1101_strobe(STROBE_STX); // Start TX of CC1101, to send what comes out MOSI
            m_opMode = cc1101_asyncTX;
            NRF_LOG_WARNING("Be sure to wire SPI2_MOSI pin %d to CC1101 GDO0", SPI2_MOSI_PIN)
            ;
        break;
        default:
            NRF_LOG_ERROR("Don't know how to set  up mode %d\n", desired)
            ;
        break;
    }

    pollers_registerPoller(cc1101Poll);
}

#endif // #if COMPILE_RADIO_CC1101
