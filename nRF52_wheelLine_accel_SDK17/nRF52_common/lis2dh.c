/*
 * lis2dh.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#include "lis2dh.h"

#include "globalInts.h"
#include "i2c1.h"
#include <math.h> // for atanf and pi
#include "nrf_delay.h" // for delay calls
#include "pollers.h"

#include "uptimeCounter.h"
#include "utils.h" // for diff since

#define NRF_LOG_MODULE_NAME lis2dh
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/* Register definitions taken from
 * https://github.com/STMicroelectronics/lis2dh12-pid/blob/master/lis2dh12_reg.h
 * AND
 * lis2dh12.pdf datasheet, rev 6
 */

/** I2C Device Address 8 bit format if SA0=0 -> 31 if SA0=1 -> 33 **/
#define LIS2DH12_I2C_ADD_L   0x31U
#define LIS2DH12_I2C_ADD_H   0x33U

// See datasheet 6.1.1 - I2C Operation
#define AUTO_INCREMENT_MASK 0x80 // If this mask is applied, the I2C slave auto-increments after a write or read.

#define STATUS_REG_AUX_REGADDR  0x07U
// b7 not used
#define STATUS_REG_AUX_TEMP_OVERRUN 0x04
// b5:3 not used
#define STATUS_REG_AUX_TEMP_AVAIL   0x04;
// b1:0 not used

#define OUT_TEMP_L_REGADDR  0x0CU

#define OUT_TEMP_H_REGADDR  0x0DU

#define WHO_AM_I_REGADDR    0x0FU
#define WHO_AM_I_VALUE      0x33U

#define CTRL_REG0_REGADDR  0x1EU
#define CTRL_REG0_DISABLE_SDO_PU    0x80
#define CTRL_REG0_B6_0_VALUE    0x10 // MUST keep these bits at these values

#define TEMP_CFG_REGADDR        0x1FU
#define TEMP_CFG_TEMP_ENABLED   0xC0 // 0b11 for temp enabled, 0b00 for disabled
// b5:0 unused

#define CTRL_REG1_REGADDR 0x20U
#define CTRL_REG1_ODR_MASK 0xF0            // b7:4 are ODR
#define CTRL_REG1_ODR(x) ((x << 4) & 0xF0) // b7:4 are ODR 3:0
typedef enum
{
    odr_powerDown = 0,
    odr_1Hz,
    odr_10Hz,
    odr_25Hz,
    odr_50Hz,
    odr_100Hz,
    odr_200Hz,
    odr_400Hz,
    odr_lowpower_1620Hz,
    odr_norm_hr_1344Hz_lowpwr_5376Hz,
    odr_invalid // Used to init
// states 10-15 not used
} ctrl1_odr_t;
#define CTRL_REG1_LPEN      0x08 // low-power enabled if set, else normal mode
#define CTRL_REG1_Z_EN      0x04
#define CTRL_REG1_Y_EN      0x02
#define CTRL_REG1_X_EN      0x01

#define CTRL_REG2_REGADDR       0x21U
#define CTRL_REG2_HPM(x)        ((x<<6)&0xC0) // b7:6 are HPM
typedef enum
{
    highPass_normalMode = 0,
    highPass_referenceForFiltering,
    highPass_normalModeAlso,
    highPass_autoresetOnInt,
} ctrl2_hpm_t;
#define CTRL_REG2_HPCF(x)       ((x<<4)&0x30) // b5:4 are HPCF
typedef enum
{
    highpassCutoff_most = 0,
    highpassCutoff_more,
    highpassCutoff_some,
    highpassCutoff_less,
} ctrl2_hpcf_t;

#define CTRL_REG2_FILTERED_DATA 0x08
#define CTRL_REG2_HP_CLICK      0x04
#define CTRL_REG2_HP_IA2        0x02
#define CTRL_REG2_HP_IA1        0x01

#define CTRL_REG3_REGADDR   0x22U
#define CTRL_REG3_I1_CLICK  0x80
#define CTRL_REG3_I1_IA1    0x40
#define CTRL_REG3_I1_IA2    0x20
#define CTRL_REG3_I1_ZYXDA  0x10
// b3 must be zero
#define CTRL_REG3_I1_WTM    0x04
#define CTRL_REG3_I1_OVER   0x02
// b0 unused

#define CTRL_REG4_REGADDR           0x23U
#define CTRL_REG4_BLOCKDATAUPDATE   0x80
#define CTRL_REG4_DATALITTLEENDIAN  0x40 // little endian if set, big if clear
#define CTRL_REG4_FULLSCALE(x)      (((x)<<4)&0x30) // b5:4 is fs 1:0
typedef enum
{
    fullScale_2g = 0,
    fullScale_4g,
    fullScale_8g,
    fullScale_16g
} ctrl_reg4_fullscale_t;
#define CTRL_REG4_HIGHRES 0x10
#define CTRL_REG4_SELFTEST_MASK 0x06           // b2:1 are self-test mode
#define CTRL_REG4_SELFTST(x) ((x << 1) & 0x06) // st 1:0
typedef enum
{
    selfTest_disabled = 0,
    selfTest_0,
    selfTest_1,
    selfTest_NA,
} ctrl_reg4_st_t; // See lis2dh12.pdf table 39
#define CTRL_REG4_SPI3WIRE       0x01 // 4-wire if 0

#define CTRL_REG5_REGADDR      0x24U
#define CTRL_REG5_MEM_REBOOT   0x80
#define CTRL_REG5_FIFO_EN      0x40
// b5, b44 unused
#define CTRL_REG5_LIR_INT1     0x08
#define CTRL_REG5_D4D_INT1     0x04
#define CTRL_REG5_LIR_INT2     0x02
#define CTRL_REG5_D4D_INT2     0x01

#define CTRL_REG6_REGADDR       0x25U
#define CTRL_REG6_I2_CLICK      0x80
#define CTRL_REG6_I2_IA1        0x40
#define CTRL_REG6_I2_IA2        0x20
#define CTRL_REG6_I2_BOOT       0x10
#define CTRL_REG6_I2_ACT        0x08
// b2 unused
#define CTRL_REG6_INT_POLARITY  0x02
// b0 unused

#define REFERENCE_REGADDR   0x26U
// Reference value for interrupt

#define STATUS_REG_REGADDR 0x27U
#define STATUS_REG_XYZ_OVERRUN 0x80
#define STATUS_REG_Z_OVERRUN 0x40
#define STATUS_REG_Y_OVERRUN 0x20
#define STATUS_REG_X_OVERRUN 0x10
#define STATUS_REG_XYZ_NEWDATA 0x08
#define STATUS_REG_Z_NEWDATA 0x40
#define STATUS_REG_Y_NEWDATA 0x20
#define STATUS_REG_X_NEWDATA 0x10

#define OUT_X_L_REGADDR 0x28U
#define OUT_X_H_REGADDR 0x29U
#define OUT_Y_L_REGADDR 0x2AU
#define OUT_Y_H_REGADDR 0x2BU
#define OUT_Z_L_REGADDR 0x2CU
#define OUT_Z_H_REGADDR 0x2DU

#define FIFO_CTRL_REGADDR   0x2EU
#define FIFO_CTRL_MODE(x)   (((x)<<6) & 0xC0)
typedef enum
{
    fifoMode_bypass = 0,
    fifoMode_fifo,
    fifoMode_stream,
    fifoMode_streamToFifo,
} fifoCtrlMode_t;
#define FIFO_CTRL_TRIG_INT2  0x20 // 0 for trigger signal on INT1
#define FIFO_CTRL_THR(x)     ((x) & 0x1F)

#define FIFO_SRC_REGADDR        0x2FU
#define FIFO_SRC_WTM            0x80
#define FIFO_SRC_OVRN           0x40
#define FIFO_SRC_EMPTY          0x20
#define FIFO_SRC_NUMUNREAD(x)   ((x) & 0x1F)

#define INT1_CFG_REGADDR    0x30U
#define INT1_CFG_AOI        0x80
#define INT1_CFG_6D         0x40
#define INT1_CFG_ZHIE       0x20
#define INT1_CFG_ZLIE       0x10
#define INT1_CFG_YHIE       0x08
#define INT1_CFG_YLIE       0x04
#define INT1_CFG_XHIE       0x02
#define INT1_CFG_XLIE       0x01

#define INT1_SRC_REGADDR    0x31U
// b7 not used
#define INT1_SRC_IA         0x40
#define INT1_SRC_ZH         0x20
#define INT1_SRC_ZL         0x10
#define INT1_SRC_YH         0x08
#define INT1_SRC_YL         0x04
#define INT1_SRC_XH         0x02
#define INT1_SRC_XL         0x01

#define INT1_THS_REGADDR    0x32U
// b7 not used
#define INT1_THS_THS(x)  ((x) & 0x7F)

#define INT1_DURATION_REGADDR  0x33U
// b7 not used
#define INT1_DURATION(x)  ((x) & 0x7F)

#define INT2_CFG_REGADDR    0x34U
#define INT2_CFG_AOI        0x80
#define INT2_CFG_6D         0x40
#define INT2_CFG_ZHIE       0x20
#define INT2_CFG_ZLIE       0x10
#define INT2_CFG_YHIE       0x08
#define INT2_CFG_YLIE       0x04
#define INT2_CFG_XHIE       0x02
#define INT2_CFG_XLIE       0x01

#define INT2_SRC_REGADDR    0x35U
// b7 not used
#define INT2_SRC_IA         0x40
#define INT2_SRC_ZH         0x20
#define INT2_SRC_ZL         0x10
#define INT2_SRC_YH         0x08
#define INT2_SRC_YL         0x04
#define INT2_SRC_XH         0x02
#define INT2_SRC_XL         0x01

#define INT2_THS_REGADDR    0x36U
// b7 not used
#define INT2_THS_THS(x)  ((x) & 0x7F)

#define INT2_DURATION_REGADDR   0x37U
// b7 not used
#define INT2_DURATION(x)  ((x) & 0x7F)

#define CLICK_CFG_REGADDR   0x38U
// b7,6 not used
#define CLICK_CFG_ZDOUBLE   0x20
#define CLICK_CFG_ZSINGLE   0x10
#define CLICK_CFG_YDOUBLE   0x08
#define CLICK_CFG_YSINGLE   0x04
#define CLICK_CFG_XDOUBLE   0x02
#define CLICK_CFG_XSINGLE   0x01

#define CLICK_SRC_REGADDR   0x39U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t x :1;
    uint8_t y :1;
    uint8_t z :1;
    uint8_t sign :1;
    uint8_t sclick :1;
    uint8_t dclick :1;
    uint8_t ia :1;
    uint8_t not_used_01 :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ia                : 1;
  uint8_t dclick            : 1;
  uint8_t sclick            : 1;
  uint8_t sign              : 1;
  uint8_t z                 : 1;
  uint8_t y                 : 1;
  uint8_t x                 : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_click_src_t;

#define CLICK_THS_REGADDR   0x3AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t ths :7;
    uint8_t lir_click :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t lir_click         : 1;
  uint8_t ths               : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_click_ths_t;

#define TIME_LIMIT_REGADDR 0x3BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t tli :7;
    uint8_t not_used_01 :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t tli               : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_time_limit_t;

#define TIME_LATENCY_REGADDR    0x3CU
typedef struct
{
    uint8_t tla :8;
} lis2dh12_time_latency_t;

#define TIME_WINDOW_REGADDR    0x3DU
typedef struct
{
    uint8_t tw :8;
} lis2dh12_time_window_t;

#define ACT_THS_REGADDR 0x3EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t acth :7;
    uint8_t not_used_01 :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t acth              : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_act_ths_t;

#define ACT_DUR_REGADDR 0x3FU
typedef struct
{
    uint8_t actd :8;
} lis2dh12_act_dur_t;

typedef enum
{
    opMode_highRes_12bit = 0,
    opMode_normal_10bit = 1,
    opMode_lowPower_8bit = 2,
    opMode_invalid,
} opMode_t;

// For holding data later
typedef struct
{
    int32_t avgX_128x, avgY_128x, avgZ_128x; // 128 is RSR 7
    uint8_t numSamples;
} samplesStruct_t;

#define NUM_ROLLING_BUFFERS 0

#define NUM_PREVIOUS_ANGLES 3

#define NUM_PREVIOUS_Z_COUNTS 0

/*************************************************************************************
 *  Variables
 ************************************************************************************/

static uint8_t m_i2cAddr;
static bool m_initted;
static opMode_t m_opMode;

static uint32_t m_lastPoll_ms;
static uint32_t m_desiredPollItvl_ms;

#if NUM_ROLLING_BUFFERS
static samplesStruct_t m_rollingSamples[NUM_ROLLING_BUFFERS];
static uint32_t m_rollingSampleWriteIndex;
#endif // #if NUM_ROLLING_BUFFERS

#if NUM_PREVIOUS_ANGLES

// #define MOVING_MIN_DEG_PER_MIN 1 // Minimum speed to be considered moving
static uint32_t m_lastRollover_ms;
#define PRE_STOP_DEGREES 15 // How many degrees short of top to stop, to account for actuator delay in turning off.
#define POST_STOP_GRACE_DEGREES 35
typedef struct
{
    int32_t angle_degree;
    uint32_t time_ms;
    int32_t speedFromPrevious;
} angleTimestamp_t;
static angleTimestamp_t m_previousAngles[NUM_PREVIOUS_ANGLES];

#define ROLLOVER_LOCKOUT_MS 15000 // How many ms minimum before we can roll over again. 30 sec per revolution, 15 sec should be good.

#endif // #if NUM_PREVIOUS_ANGLES

#if NUM_PREVIOUS_Z_COUNTS
static int32_t m_previousZ128ths[NUM_PREVIOUS_Z_COUNTS];
#endif // #if NUM_PREVIOUS_Z_COUNTS

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static ret_code_t writeAccelReg(uint8_t regAddr, uint8_t byte)
{
    uint8_t toWrite[2];
    toWrite[0] = regAddr;
    toWrite[1] = byte;
    return i2c1_writeBytes(m_i2cAddr, toWrite, 2);
}

static ret_code_t readAccelReg(uint8_t regAddr, uint8_t* pRxByte)
{
    return i2c1_readByte(m_i2cAddr, regAddr, pRxByte);
}

static ret_code_t setOpMode(opMode_t desiredMode)
{
    uint8_t ctrl_reg1;
    uint8_t ctrl_reg4;
    ret_code_t ret;

    ret = readAccelReg(CTRL_REG1_REGADDR, &ctrl_reg1);
    if (NRF_SUCCESS == ret)
    {
        ret = readAccelReg(CTRL_REG4_REGADDR, &ctrl_reg4);
    }
    if (NRF_SUCCESS == ret)
    {
        if (desiredMode == opMode_highRes_12bit)
        {
            ctrl_reg1 &= (uint8_t)(~CTRL_REG1_LPEN); // Clear Low power EN
            ctrl_reg4 |= CTRL_REG4_HIGHRES; // Set High res
        }
        if (desiredMode == opMode_normal_10bit)
        {
            ctrl_reg1 &= (uint8_t)(~CTRL_REG1_LPEN); // Clear Low power EN
            ctrl_reg4 &= (uint8_t)(~CTRL_REG4_HIGHRES); // Clear High Resolution
        }
        if (desiredMode == opMode_lowPower_8bit)
        {
            ctrl_reg1 |= CTRL_REG1_LPEN; // Set low-power
            ctrl_reg4 &= (uint8_t)(~CTRL_REG4_HIGHRES); // Clear High Resolution
        }
        ret |= writeAccelReg(CTRL_REG1_REGADDR, ctrl_reg1);
        ret |= writeAccelReg(CTRL_REG4_REGADDR, ctrl_reg4);
    }
    if (NRF_SUCCESS == ret)
    {
        m_opMode = desiredMode;
    }
    return ret;
}

static void setODR(ctrl1_odr_t odr)
{
    uint8_t u8;
    readAccelReg(CTRL_REG1_REGADDR, &u8);
    if (CTRL_REG1_ODR(odr) != u8 & CTRL_REG1_ODR_MASK)
    {
        u8 &= ~CTRL_REG1_ODR_MASK;
        u8 |= CTRL_REG1_ODR(odr);
        writeAccelReg(CTRL_REG1_REGADDR, u8);
        NRF_LOG_DEBUG("CTRL_REG1 to 0x%02x to change ODR to %d\n", u8, odr);
    }
}

static void setSelfTestMode(ctrl_reg4_st_t mode)
{
    uint8_t u8;
    readAccelReg(CTRL_REG4_REGADDR, &u8);
    u8 &= ~CTRL_REG4_SELFTEST_MASK;
    u8 |= CTRL_REG4_SELFTST(mode);
    writeAccelReg(CTRL_REG4_REGADDR, u8);
    NRF_LOG_DEBUG("Wrote ctrlreg4 to 0x%02x to set test mode %d\n", u8, mode);
}

static void convertSamples(uint8_t* pRawData, int16_t* pConvertedSamples, samplesStruct_t* p)
{
    for (uint8_t i = 0; i < p->numSamples; i++)
    {
        /* Each 6 bytes is 1 sample of X, Y, and Z.
         * All data is left-justified, and LSByte is first, eg OUT_X_L=0x28, OUT_X_H=0x29
         * Left shift generates zeroes to the right, right-shifts should generate sign bits to the left (usually).
         * So, put uint8[1] into upper 8 bits, uint8[0] into lower 8 bits.
         * Then shift right by however many bits, as per resolution mode bits: 12bit, 10bit, or 8bit.
         */
        pConvertedSamples[i * 3] = (int16_t)(((int16_t)pRawData[i * 6 + 1]) << 8);
        // OR in the LSByte
        pConvertedSamples[i * 3] |= pRawData[i * 3];
        pConvertedSamples[i * 3 + 1] = (int16_t)(((int16_t)pRawData[i * 6 + 3]) << 8);
        pConvertedSamples[i * 3 + 1] |= pRawData[i * 6 + 2];
        pConvertedSamples[i * 3 + 2] = (int16_t)(((int16_t)pRawData[i * 6 + 5]) << 8);
        pConvertedSamples[i * 3 + 2] |= pRawData[i * 6 + 4];
        switch (m_opMode)
        {
            case opMode_highRes_12bit:
                // Shift down by 16-12=4 bits
                pConvertedSamples[i * 3] >>= 4;
                pConvertedSamples[i * 3 + 1] >>= 4;
                pConvertedSamples[i * 3 + 2] >>= 4;
            break;
            case opMode_normal_10bit:
                // Shift down by 16-10=6 bits
                pConvertedSamples[i * 3] >>= 6;
                pConvertedSamples[i * 3 + 1] >>= 6;
                pConvertedSamples[i * 3 + 2] >>= 6;
            break;
            case opMode_lowPower_8bit:
                // Shift down by 16-8=8 bits
                pConvertedSamples[i * 3] >>= 8;
                pConvertedSamples[i * 3 + 1] >>= 8;
                pConvertedSamples[i * 3 + 2] >>= 8;
            break;
            default:
                NRF_LOG_ERROR("opMode %d not supported\n", m_opMode)
                ;
            break;
        }
        // NRF_LOG_DEBUG("Sample %d: X: %d, Y: %d, Z: %d\n", i, pConvertedSamples[i * 3 + 0], pConvertedSamples[i * 3 + 1],
        //               pConvertedSamples[i * 3 + 2]);
    }
}

static bool isDataReady(void)
{
    uint8_t u8;
    readAccelReg(STATUS_REG_REGADDR, &u8);
    // NRF_LOG_DEBUG("Read status reg 0x%02x\n", u8);
    if (u8 & STATUS_REG_XYZ_NEWDATA)
    {
        return true;
    }
    return false;
}

static void readSingleSample(int16_t* pX, int16_t* pY, int16_t* pZ)
{
    samplesStruct_t samps;
    samps.numSamples = 1;
    // Read 6 bytes for samples
    uint8_t rawDataBuf[samps.numSamples * 6];
    memset(rawDataBuf, 0x00, 6);
    // Read all
    // NRF_LOG_DEBUG("Checking DataReady before read:%s\n", isDataReady() ? "Ready" : "Not ready");
    ret_code_t ret = i2c1_readBytes(m_i2cAddr, OUT_X_L_REGADDR | AUTO_INCREMENT_MASK, rawDataBuf, 6 * samps.numSamples);
    // NRF_LOG_DEBUG("Checking DataReady AFTER read:%s\n", isDataReady() ? "Ready" : "Not ready");
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("readSamples I2C error 0x%x\n", ret);
        return;
    }
    // Now convert samples to int16 readings. X, Y, Z so 3 axes per sample.
    int16_t convertedSamples[samps.numSamples * 3];
    convertSamples(rawDataBuf, convertedSamples, &samps);
    *pX = convertedSamples[0];
    *pY = convertedSamples[1];
    *pZ = convertedSamples[2];
    NRF_LOG_DEBUG("Read single sample %d, %d, %d\n", convertedSamples[0], convertedSamples[1], convertedSamples[2]);
}

static ret_code_t runSelfTest(void)
{
    // Assume all defaults
    setODR(odr_400Hz);
    nrf_delay_ms(2); // Normal mode enable takes 1.6ms.

    // In normal mode, it takes 2 samples to start test
    // Turn on self-test mode
    setSelfTestMode(selfTest_0);
    int16_t x0 = 0, y0 = 0, z0 = 0;
    uint8_t delayCycles = 4;
    while (delayCycles)
    {
        readSingleSample(&x0, &y0, &z0);
        uint8_t timeout = 5;
        while (!isDataReady() && timeout)
        {
            nrf_delay_ms(2); // At 400Hz, sample should be ready in 2.5ms
            timeout--;
        }
        delayCycles--;
    }
    readSingleSample(&x0, &y0, &z0);
    NRF_LOG_DEBUG("Test0 readings: %d, %d, %d\n", x0, y0, z0);
    setSelfTestMode(selfTest_1);
    int16_t x1 = 0, y1 = 0, z1 = 0;
    delayCycles = 4;
    while (delayCycles)
    {
        readSingleSample(&x1, &y1, &z1);
        uint8_t timeout = 5;
        while (!isDataReady() && timeout)
        {
            nrf_delay_ms(2); // At 400Hz, sample should be ready in 2.5ms
            timeout--;
        }
        delayCycles--;
    }
    readSingleSample(&x1, &y1, &z1);
    NRF_LOG_DEBUG("Test1 readings: %d, %d, %d\n", x1, y1, z1);
    // Set to normal to compare
    setSelfTestMode(selfTest_disabled);
    int16_t xNorm = 0, yNorm = 0, zNorm = 0;
    delayCycles = 4;
    while (delayCycles)
    {
        readSingleSample(&xNorm, &yNorm, &zNorm);
        uint8_t timeout = 5;
        while (!isDataReady() && timeout)
        {
            nrf_delay_ms(3); // At 400Hz, sample should be ready in 2.5ms
            timeout--;
        }
        delayCycles--;
    }
    readSingleSample(&xNorm, &yNorm, &zNorm);
    NRF_LOG_DEBUG("Normal readings: %d, %d, %d\n", xNorm, yNorm, zNorm);
    if ((x1 - xNorm) < 17 || (x1 - xNorm) > 360)
    {
        NRF_LOG_ERROR("X deviation was %d, should be 17 to 360\n", x1 - xNorm);
        return NRF_ERROR_INVALID_STATE;
    }

    if ((y1 - yNorm) < 17 || (y1 - yNorm) > 360)
    {
        NRF_LOG_ERROR("Y deviation was %d, should be 17 to 360\n", (y1 - yNorm));
        return NRF_ERROR_INVALID_STATE;
    }
    if ((z1 - zNorm) < 17 || (z1 - zNorm) > 360)
    {
        NRF_LOG_ERROR("Z deviation was %d, should be 17 to 360\n", (z1 - zNorm));
        return NRF_ERROR_INVALID_STATE;
    }
    NRF_LOG_DEBUG("Deviations were %d, %d, %d\n", (x1 - xNorm), (y1 - yNorm), (z1 - zNorm));
    return NRF_SUCCESS;
}

static ret_code_t accelInit(void)
{
//    NRF_LOG_DEBUG("Starting accel init:");
    uint8_t byte = 0x00;
    m_i2cAddr = LIS2DH12_I2C_ADD_H >> 1;
    ret_code_t ret = readAccelReg(WHO_AM_I_REGADDR, &byte);
    if (NRF_SUCCESS != ret || WHO_AM_I_VALUE != byte)
    {
        NRF_LOG_WARNING("LIS2DH12 not found at 0x%x, returned 0x%x, need 0x%x", m_i2cAddr, byte, WHO_AM_I_VALUE);
        m_i2cAddr = LIS2DH12_I2C_ADD_L >> 1;
        ret = readAccelReg(WHO_AM_I_REGADDR, &byte);
        if (NRF_SUCCESS != ret || WHO_AM_I_VALUE != byte)
        {
            NRF_LOG_ERROR("LIS2DH12 not found at 0x%x, returned 0x%x, need 0x%x", m_i2cAddr, byte,
                          WHO_AM_I_VALUE);
            return ret;
        }
    }
    ret = 0;
    // reset the chip to defaults
    ret = writeAccelReg(CTRL_REG5_REGADDR, CTRL_REG5_MEM_REBOOT);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("Error rebooting LIS2DH12");
    }
    // LIS2DH12 specifies 5ms max boot time
    nrf_delay_ms(5);
    ret = readAccelReg(WHO_AM_I_REGADDR, &byte);
    if (NRF_SUCCESS != ret || WHO_AM_I_VALUE != byte)
    {
        NRF_LOG_ERROR("Error after reboot of LIS2DH12, WHOAMI 0x%x, need 0x%x", byte, WHO_AM_I_VALUE);
    }
    // Run self-test after reboot, when it has default values
    ret = runSelfTest();
    if (NRF_SUCCESS != ret)
    {
        // NRF_LOG_ERROR("SelfTest error %d, bail out\n", ret);
        // return ret;
        NRF_LOG_ERROR("SelfTest error %d, ignore, continue\n", ret);
    }

    // Set up all registers with values we need.
//    ret |= writeAccelReg(CTRL_REG0_REGADDR, CTRL_REG0_B6_0_VALUE);
//    ret |= writeAccelReg(TEMP_CFG_REGADDR, 0x00);
    byte = CTRL_REG1_ODR(odr_50Hz);
    // TODO can we ignore one of the axes when mounted properly?
    byte |= (CTRL_REG1_X_EN | CTRL_REG1_Y_EN | CTRL_REG1_Z_EN);
    // TODO should we enable LP mode? Probably don't need to.
    ret = writeAccelReg(CTRL_REG1_REGADDR, byte);

    // We do NOT want high-pass filtering, we need absolute positioning
//    byte = CTRL_REG2_HPM(highPass_normalMode);
//    ret |= writeAccelReg(CTRL_REG2_REGADDR, byte);
    // Don't enable any interrupts. TODO, wire INT pin to MCU, and interrupt on fifo watermark?
//    ret |= writeAccelReg(CTRL_REG3_REGADDR, 0x00);
    byte = CTRL_REG4_BLOCKDATAUPDATE;
    byte |= CTRL_REG4_FULLSCALE(fullScale_2g);
    // TODO do we need to set high-res mode?
//    byte |= CTRL_REG4_HIGHRES;
    m_opMode = opMode_normal_10bit;
    ret |= writeAccelReg(CTRL_REG4_REGADDR, byte);
    // Do ctrl_reg5 AFTER setting up the FIFO parameters
//    byte = CTRL_REG6_INT_POLARITY;
//    ret |= writeAccelReg(CTRL_REG6_REGADDR, byte);
    // Read Reference register, to reset filtering block, as recommended.
    ret |= readAccelReg(REFERENCE_REGADDR, &byte);
    // Enable Stream mode, bits 4:0 FIFO size, set as large as possible
    byte = FIFO_CTRL_MODE(fifoMode_stream); // Stream so it always has the latest data.
    byte |= FIFO_CTRL_THR(0xFF); // Threshold as large as possible
    ret |= writeAccelReg(FIFO_CTRL_REGADDR, byte);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("Error %d after init steps, bail out\n", ret);
        return ret;
    }

    /* Set poll interval based on ODR and FIFO size
     * At 100Hz ODR, FIFO 31 samples, we will get a sample every 10ms, can poll every 310ms.
     * At 50Hz ODR, FIFO 31 samples, we will get 31 samples every 600ms
     * TODO change if ODR changes, to read all samples.
     */
    setODR(odr_50Hz);
    m_desiredPollItvl_ms = 580;

    // CTRL_REG5 to only enable FIFO, no interrupts
    byte = CTRL_REG5_FIFO_EN;
    ret |= writeAccelReg(CTRL_REG5_REGADDR, byte);

    // Disable INT1 interrupts
//    ret |= writeAccelReg(INT1_CFG_REGADDR, 0x00);
    // Disable INT2 interrupts
//    ret |= writeAccelReg(INT2_CFG_REGADDR, 0x00);
    // Disable Click detection
//    ret |= writeAccelReg(CLICK_CFG_REGADDR, 0x00);
    // Don't set up activity
    if (NRF_SUCCESS == ret)
    {
        NRF_LOG_INFO("LIS2DH12 init success, I2C address 0x%x", m_i2cAddr);
        m_initted = true;
    }
    else
    { // Log error(s)
        NRF_LOG_ERROR("LIS2DH init error somewhere, ORed error %d", ret);
    }
    return ret;
}

static void findMinMaxAverage(int16_t* pConvertedSamples, samplesStruct_t* p)
{
    int16_t minX = 1000, maxX = -1000, minY = 1000, maxY = -1000, minZ = 1000, maxZ = -1000;
    // use int32 for accumulating, before storing into int16
    int32_t avgX = 0, avgY = 0, avgZ = 0;
    for (uint8_t i = 0; i < p->numSamples; i++)
    {
        if (pConvertedSamples[i * 3] < minX)
        {
            minX = pConvertedSamples[i * 3];
        }
        if (pConvertedSamples[i * 3] > maxX)
        {
            maxX = pConvertedSamples[i * 3];
        }
        if (pConvertedSamples[i * 3 + 1] < minY)
        {
            minY = pConvertedSamples[i * 3 + 1];
        }
        if (pConvertedSamples[i * 3 + 1] > maxY)
        {
            maxY = pConvertedSamples[i * 3 + 1];
        }
        if (pConvertedSamples[i * 3 + 2] < minZ)
        {
            minZ = pConvertedSamples[i * 3 + 2];
        }
        if (pConvertedSamples[i * 3 + 2] > maxZ)
        {
            maxZ = pConvertedSamples[i * 3 + 2];
        }
        avgX += pConvertedSamples[i * 3];
        avgY += pConvertedSamples[i * 3 + 1];
        avgZ += pConvertedSamples[i * 3 + 2];
    }
    // NRF_LOG_DEBUG("Accum:             %d, %d, %d\n", avgX, avgY, avgZ);
    // Store 128x the sample
    avgX *= 128;
    avgY *= 128;
    avgZ *= 128;
    // NRF_LOG_DEBUG("Accumx128:         %d, %d, %d\n", avgX, avgY, avgZ);
    avgX /= (int32_t)p->numSamples;
    avgY /= (int32_t)p->numSamples;
    avgZ /= (int32_t)p->numSamples;

    p->avgX_128x = (int32_t)avgX;
    p->avgY_128x = (int32_t)avgY;
    p->avgZ_128x = (int32_t)avgZ;

    // NRF_LOG_DEBUG("%d Sample min:     %d,\t%d,\t%d\n", p->numSamples, minX, minY, minZ);
    // NRF_LOG_DEBUG("%d Sample average: %d.%d, %d.%d, %d.%d.  ", p->numSamples, p->avgX_128x >> 7, p->avgX_128x & 0x7F, p->avgY_128x >> 7, p->avgY_128x & 0x7F, p->avgZ_128x >> 7, p->avgZ_128x & 0x7F);
    // NRF_LOG_DEBUG("%d Sample max:     %d,\t%d,\t%d\n", p->numSamples, maxX, maxY, maxZ);
}

#if NUM_ROLLING_BUFFERS
static void findSmoothedPoints(int16_t *pX, int16_t *pY, int16_t *pZ)
{
    // Accumulate all points, then divide for weighted average.
    int32_t avgX = 0, avgY = 0, avgZ = 0;
    int32_t totalNumSamples = 0;
    // accumulate weighted samples
    for (uint32_t i = 0; i < NUM_ROLLING_BUFFERS; i++)
    { // for each index in the array, multiply by number of samples to get weight in our new average
        avgX += (m_rollingSamples[i].avgX * m_rollingSamples[i].numSamples);
        avgY += (m_rollingSamples[i].avgY * m_rollingSamples[i].numSamples);
        avgZ += (m_rollingSamples[i].avgZ * m_rollingSamples[i].numSamples);
        totalNumSamples += m_rollingSamples[i].numSamples;
    }
    // divide to get average
    avgX /= totalNumSamples;
    *pX = (int16_t)avgX;
    avgY /= totalNumSamples;
    *pY = (int16_t)avgY;
    avgZ /= totalNumSamples;
    *pZ = (int16_t)avgZ;
    NRF_LOG_DEBUG("\nSmoothed %d points, found counts %d, %d, %d\n", totalNumSamples, *pX, *pY, *pZ);
}
#endif // #if NUM_ROLLING_BUFFERS

#if NUM_PREVIOUS_ANGLES
static int32_t findAngle_xzAxis(int32_t xCounts_128th, int32_t zCounts_128th)
{
    /* See https://www.digikey.com/en/articles/using-an-accelerometer-for-inclination-sensing
     * Single axis, in Radians, theta(angle) = arcsin(Ax,out[g]/1g)
     * We have 10 bits for full scale, with +/-2g full scale. So 9 bits is 511 max for +2g
     * if 511 is 2G, then 205.5 is +1g
     * convert 128x counts to counts, floating point */
    // float toSine = (float)counts_128th;
    // // divide by max count per 1g.
    // toSine /= (float)maxCounts_128th;
    // float theta = asinf(toSine);
    /* That had issues, try two-axis
     * Two-axis, in Radians, theta(angle) = arctan() */
    if (0 == zCounts_128th)
    { // Avoid division by zero
        zCounts_128th = 1;
    }
    float theta = (float)xCounts_128th / (float)zCounts_128th;
    theta = atanf(theta);

    // convert to degrees (multiply by 180/pi)
    theta = (theta * 180.0f) / (float)M_PI;

    // now we have an angle between 0 and 90 degrees. We need the full circle.
    /* When rolling to the left:
     * Angle starts at zero, goes up to 90.
     * Then, at 90 degrees, it flips to -90. From 90 to 180, it counts down until we are upside down (negative z)
     * Then, at 180 degrees, it reads 0. 180 to 270 counts up from zero to 90.
     * Then at 270, it flips to -90. 270 to 360 counts down to 0.
     * For real angles <90, X is positive, Z is positive. Angle will be positive, counting up from zero, and correct.
     * For real angle 90-180, X is positive and Z is negative, angle is negative Real angle is 180+angle.
     * For real angle 180-270, X and Z are negative, angle is positive. Real angle is 180+angle.
     * For 270-360, X is negative and Z is positive, angle is negative. Real angle is 360+angle (since angle is negative).
     */

    if (zCounts_128th < 0)
    { // Z negative, 180+angle for both +X and -X
        theta += 180.0f;
        return (int32_t)(theta);
    }
    else
    { // Z positive, check X to see what to do
        if (xCounts_128th >= 0)
        { // X positive: accurate angle from 0-90
            return (int32_t)(theta);
        }
        else
        { // X negative, 360+angle to get 270-360
            theta += 360.0f;
            return (int32_t)(theta);
        }
    }
}

// Angles can wrap around 360
static int32_t findAngleDiff_deg(int32_t angle1, int32_t angle2)
{
    // Consider a wrap-around if angle1 is greater than 350 and angle2 is less than 10
    if (angle1 > 315 && angle2 < 45)
    {
        int32_t diff = 360 - angle1;
        diff += angle2;
        return diff;
    }
    // Consider a wrap-around if angle2 is greater than 350 and angle1 is less than 10
    if (angle2 > 315 && angle1 < 45)
    {
        int32_t diff = 360 - angle2;
        diff += angle1;
        return -diff;
    }
    // Default case, just subtract to get difference
    return angle2 - angle1;
}

// Find speed in degrees per minute
static int32_t findLatestSpeed_degPerMin(angleTimestamp_t* pLatest, angleTimestamp_t* pOlder)
{
    if (0 == pLatest || 0 == pOlder)
    {
        NRF_LOG_ERROR("Need multiple angles to calculate speeds");
        return 0;
    }
    // Make an array less one of the points
    int32_t angleDiff_deg = findAngleDiff_deg(pOlder->angle_degree, pLatest->angle_degree);
    int32_t timeDiff_ms = utils_elapsedU32Ticks(pOlder->time_ms, pLatest->time_ms);

    if (timeDiff_ms)
    { /* Calculate in degree per minute:
     * deg/ms to deg/min: multiply deg by 1000 to get to sec, 60 to get to min
     * Divide last to perserve data bits.
     */
        return (angleDiff_deg * 60000) / timeDiff_ms;
    }
    else
    {
        return 0;
    }
}

static bool isRolloverLockedOut(uint32_t ms_now)
{
    return (utils_elapsedU32Ticks(m_lastRollover_ms, ms_now) >= ROLLOVER_LOCKOUT_MS ? false : true);
}

static void doAngleMath(int32_t xCounts_128ths, int32_t zCounts_128ths)
{
    // find roll angle.
    angleTimestamp_t thisAngle;
    thisAngle.angle_degree = findAngle_xzAxis(xCounts_128ths, zCounts_128ths);
    thisAngle.time_ms = millis();
    thisAngle.speedFromPrevious = findLatestSpeed_degPerMin(&thisAngle, &m_previousAngles[0]);
    NRF_LOG_DEBUG("%d angles, Most recent: %d, speed: %d. ", NUM_PREVIOUS_ANGLES + 1, thisAngle.angle_degree,
                  thisAngle.speedFromPrevious);
    // for (uint32_t i = 0; i < NUM_PREVIOUS_ANGLES; i++)
    // {
    //     NRF_LOG_DEBUG("%d,  %d. ", m_previousAngles[i].angle_degree, m_previousAngles[i].speedFromPrevious);
    // }
    // NRF_LOG_DEBUG("\n");
    // Traverse the array, use slopes and angles to determine if we are close to the end of a revolution
    int32_t avgSpeed = 0;
    avgSpeed += thisAngle.speedFromPrevious;
    for (uint32_t i = 0; i < NUM_PREVIOUS_ANGLES; i++)
    {
        avgSpeed += m_previousAngles[i].speedFromPrevious;
    }
    avgSpeed /= NUM_PREVIOUS_ANGLES + 1; // num angles plus our latest sample
    // Find average angle
    int32_t avgAngle = 0;
    avgAngle += thisAngle.angle_degree;
    for (uint32_t i = 0; i < NUM_PREVIOUS_ANGLES; i++)
    {
        avgAngle += m_previousAngles[i].angle_degree;
    }
    avgAngle /= NUM_PREVIOUS_ANGLES + 1; // Old array plus most recent sample
    // See if we are considered as moving
    machineState_t currentState = globalInts_getMachineState();
    if (machState_runEngineHydRev == currentState)
    {
        // if (avgSpeed >= MOVING_MIN_DEG_PER_MIN)
        // { // If speed is positive, see if we are close to 360
        int32_t angleFrom360 = findAngleDiff_deg(avgAngle, 360);
        NRF_LOG_DEBUG("CCW/REV direction, angleFrom360 %d, avgSpeed %d (should be positive), avgAngle %d. ",
                      angleFrom360,
                      avgSpeed, avgAngle);
        // Count up to 360. Difference from 360 will shrink when about to roll over. Don't try to trigger once it goes negative again though!
        if (-POST_STOP_GRACE_DEGREES <= angleFrom360 && PRE_STOP_DEGREES > angleFrom360)
        {
            if (0 > avgSpeed)
            {
                NRF_LOG_WARNING("Ignoring possible rollover because thisSpeed < 0\n");
            }
            else
            {
                if (isRolloverLockedOut(thisAngle.time_ms))
                {
                    NRF_LOG_WARNING("Ignoring possible CCW/REV rollover, only %d since last rollover\n",
                                    utils_elapsedU32Ticks(m_lastRollover_ms, thisAngle.time_ms));
                }
                else
                { // Decrement the number of revolutions til we are done.
                    NRF_LOG_INFO("Detected CCW/REV rollover at %d: ", thisAngle.time_ms);
                    int8_t rots = globalInts_getNumRotations();
                    rots++; // Increase for each reverse rotation completed
                    globalInts_setNumRotations(rots);
                    if (0 == rots)
                    {
                        NRF_LOG_INFO("IDLING ENGINE due to no more revs");
                        globalInts_setMachineState(machState_runEngineHydIdle);
                    }
                    m_lastRollover_ms = thisAngle.time_ms;
                }
            }
        }
        else
        {
            NRF_LOG_INFO("\n");
        }
        // }
    }
    else if (machState_runEngineHydFwd == currentState)
    {
        // if (avgSpeed <= -MOVING_MIN_DEG_PER_MIN)
        // {
        int32_t angleFrom0 = findAngleDiff_deg(0, avgAngle);
        NRF_LOG_DEBUG("CW/FWD direction, angleFrom0 %d, avgSpeed %d (should be negative), avgAngle %d. ", angleFrom0,
                      avgSpeed,
                      avgAngle);
        // Count down to 0 degrees
        if (-POST_STOP_GRACE_DEGREES <= angleFrom0 && PRE_STOP_DEGREES > angleFrom0)
        {
            if (0 < avgSpeed)
            {
                NRF_LOG_WARNING("Ignoring possible rollover because thisSpeed > 0\n");
            }
            else
            {
                if (isRolloverLockedOut(thisAngle.time_ms))
                {
                    NRF_LOG_WARNING("Ignoring possible CW/FWD rollover, only %d since last rollover\n",
                                    utils_elapsedU32Ticks(m_lastRollover_ms, thisAngle.time_ms));
                }
                else
                { // Decrement the number of revolutions til we are done.
                    NRF_LOG_INFO("Detected CW/FWD rollover at %d, angleFrom0 %d: ", thisAngle.time_ms, angleFrom0);
                    int8_t rots = globalInts_getNumRotations();
                    rots--; // Decrease for each forward rotation completed
                    globalInts_setNumRotations(rots);
                    if (0 == rots)
                    {
                        NRF_LOG_INFO("IDLING ENGINE due to no more revs");
                        globalInts_setMachineState(machState_runEngineHydIdle);
                    }
                    m_lastRollover_ms = thisAngle.time_ms;
                }
            }
        }
        else
        {
            NRF_LOG_INFO("\n");
        }
        // }
    }
    else
    { // Not in FWD or REV. Set rollover lockout timer to now
      // Set to now to lock out wobble on start of revolution
        m_lastRollover_ms = thisAngle.time_ms;
    }

    // Bubble the samples down
    for (uint32_t i = NUM_PREVIOUS_ANGLES - 1; i > 0; i--)
    {
        m_previousAngles[i] = m_previousAngles[i - 1];
    }
    // Store the latest at 0
    m_previousAngles[0] = thisAngle;
}
#endif // #if NUM_PREVIOUS_ANGLES

static void readSamples(void)
{
    uint8_t byte;
    //        readAccelReg(STATUS_REGADDR, &byte);
    //            NRF_LOG_DEBUG("Status reg before: 0x%x", byte);
    readAccelReg(FIFO_SRC_REGADDR, &byte);
    if (byte & FIFO_SRC_EMPTY)
    {
        NRF_LOG_DEBUG("no FIFO samples");
        return;
    }
    samplesStruct_t samps;
    samps.numSamples = FIFO_SRC_NUMUNREAD(byte);
    NRF_LOG_INFO("FIFO has %d samples", samps.numSamples);

    /* Datasheet says that when using FIFO, start by reading X_OUT_L (0x28), and read 6 bytes.
     * After reading 6 bytes, the address pointer will wrap to 0x28 again. So,
     * just read 6*numSamples bytes to read all the samples at once */
    uint8_t rawDataBuf[samps.numSamples * 6];
    // Read all
    ret_code_t ret = i2c1_readBytes(m_i2cAddr, OUT_X_L_REGADDR | AUTO_INCREMENT_MASK, rawDataBuf, 6 * samps.numSamples);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_INFO("readSamples I2C error 0x%x", ret);
        return;
    }
    // If here, we have read samples
    //    NRF_LOG_HEXDUMP_INFO(rawDataBuf, sizeof(rawDataBuf));
    //     Should be no more samples now
    //    readAccelReg(STATUS_REGADDR, &byte);
    //    NRF_LOG_DEBUG("Status reg after: 0x%x", byte);
    //    readAccelReg(FIFO_SRC_REGADDR, &byte);
    //    if (byte & FIFO_SRC_EMPTY)
    //    {
    //        NRF_LOG_DEBUG("no FIFO samples after reading %d samples", numSamples);
    //    }
    //    else
    //    {
    //        NRF_LOG_DEBUG("FIFO has %d samples after reading %d", FIFO_SRC_NUMUNREAD(byte), numSamples);
    //    }

    // Now convert samples to int16 readings. X, Y, Z so 3 axes per sample.
    int16_t convertedSamples[samps.numSamples * 3];
    convertSamples(rawDataBuf, convertedSamples, &samps);
    findMinMaxAverage(convertedSamples, &samps);
    // Put into our rolling buffers array

#if NUM_ROLLING_BUFFERS
    // Copy into buffer
    m_rollingSamples[m_rollingSampleWriteIndex] = samps;
    // Update pointer
    m_rollingSampleWriteIndex++;
    if (m_rollingSampleWriteIndex >= NUM_ROLLING_BUFFERS)
    {
        m_rollingSampleWriteIndex = 0;
    }
#endif // #if NUM_ROLLING_BUFFERS

#if NUM_PREVIOUS_ANGLES
    doAngleMath(samps.avgX_128x, samps.avgZ_128x);
#endif // #if NUM_PREVIOUS_ANGLES

#if NUM_PREVIOUS_Z_COUNTS
    NRF_LOG_DEBUG("Z latest and list: %d.%d, ", samps.avgZ_128x >> 7, samps.avgZ_128x & 0x7F);
    for (uint32_t i = 0; i < NUM_PREVIOUS_Z_COUNTS; i++)
    {
        NRF_LOG_DEBUG("%d.%d,  ", m_previousZ128ths[i] >> 7, m_previousZ128ths[i] & 0x7F);
    }
    NRF_LOG_DEBUG("\n");
    // TODO make decisions
    if (samps.avgZ_128x > m_maxCount128th_any)
    {
        m_maxCount128th_any = samps.avgZ_128x;
    }
    int32_t diff1 = samps.avgZ_128x - m_previousZ128ths[0];
    int32_t diff2 = m_previousZ128ths[0] - m_previousZ128ths[1];
    if (diff1 <= 0 && diff2 >= 0)
    { // If it stops rising, we rotated
        Serial.println("Detected a finish of revolutions");
        int8_t numRevs = globalInts_getNumRotations();
        if (numRevs > 0)
        {
            numRevs--;
            globalInts_setNumRotations(numRevs);
        }
        else if (numRevs < 0)
        {
            numRevs++;
            globalInts_setNumRotations(numRevs);
        }
    }

    // Bubble the samples down and store latest at index[0]
    for (uint32_t i = NUM_PREVIOUS_Z_COUNTS - 1; i > 0; i--)
    {
        m_previousZ128ths[i] = m_previousZ128ths[i - 1];
    }
    // Store the latest at 0
    m_previousZ128ths[0] = samps.avgZ_128x;

#endif // #if NUM_PREVIOUS_Z_COUNTS
}

static void lis2dh12Poll(void)
{
    if (uptimeCounter_elapsedSince(m_lastPoll_ms) > m_desiredPollItvl_ms)
    { // Wait a while between re-inits
        if (!m_initted)
        {
            if (NRF_SUCCESS != accelInit())
            {
                m_desiredPollItvl_ms = 1200; // retry every 1200ms

            }
            else
            {
                // Initted, init should set poll interval. Don't reset poll inter
            }
        }
        else
        { // If initted, do poll stuff
            readSamples();
        }
        m_lastPoll_ms = uptimeCounter_getUptimeMs();
    }
}

void lis2dh_init(void)
{
    m_initted = false;
    m_lastPoll_ms = uptimeCounter_getUptimeMs();
    m_desiredPollItvl_ms = 100;
    i2c1_init(); // Make sure bus is enabled.

    // Init params, set it up the way we want, if we can.
    ret_code_t ret = accelInit();
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("LIS2DH12 not found at address 0x%x, bailing out", m_i2cAddr);
        return;
        // Don't poll if we don't exist
    }

#if NUM_ROLLING_BUFFERS
    for (uint32_t i = 0; i < NUM_ROLLING_BUFFERS; i++)
    { // Zero out the ones we haven't used yet
        m_rollingSamples[i].numSamples = 0;
        m_rollingSampleWriteIndex = 0;
    }
#endif // #if NUM_ROLLING_BUFFERS

#if NUM_PREVIOUS_ANGLES
    for (uint32_t i = 0; i < NUM_PREVIOUS_ANGLES; i++)
    { // Zero out the ones we haven't used yet
        m_previousAngles[i].angle_degree = 0;
        m_previousAngles[i].time_ms = 0;
    }
    m_lastRollover_ms = millis(); // Set to now to lock out wobble on start of revolution
#endif                            // #if NUM_PREVIOUS_ANGLES

    pollers_registerPoller(lis2dh12Poll);
}
