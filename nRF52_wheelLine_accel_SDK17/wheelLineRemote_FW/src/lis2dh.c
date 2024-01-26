/*
 * lis2dh.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#include "lis2dh.h"

#include "i2c.h"
#include "pollers.h"

#define NRF_LOG_MODULE_NAME lis2dh
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/* Register definitions taken from
 * https://github.com/STMicroelectronics/lis2dh12-pid/blob/master/lis2dh12_reg.h
 */

#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN  __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN     __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER     __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

/**
 * @}
 *
 */

/** @defgroup STMicroelectronics sensors common types
 * @{
 *
 */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t bit0 :1;
    uint8_t bit1 :1;
    uint8_t bit2 :1;
    uint8_t bit3 :1;
    uint8_t bit4 :1;
    uint8_t bit5 :1;
    uint8_t bit6 :1;
    uint8_t bit7 :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
 * @brief       This section provide a set of functions used to read and
 *              write a generic register of the device.
 *              MANDATORY: return 0 -> no Error.
 * @{
 *
 */
typedef int32_t (*stmdev_write_ptr)(void*, uint8_t, const uint8_t*, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void*, uint8_t, uint8_t*, uint16_t);
typedef void (*stmdev_mdelay_ptr)(uint32_t millisec);

typedef struct
{
    /** Component mandatory fields **/
    stmdev_write_ptr write_reg;
    stmdev_read_ptr read_reg;
    /** Component optional fields **/
    stmdev_mdelay_ptr mdelay;
    /** Customizable optional pointer **/
    void* handle;
} stmdev_ctx_t;

/**
 * @}
 *
 */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
 * @brief       This structure is useful to load a predefined configuration
 *              of a sensor.
 *              You can create a sensor configuration by your own or using
 *              Unico / Unicleo tools available on STMicroelectronics
 *              web site.
 *
 * @{
 *
 */

typedef struct
{
    uint8_t address;
    uint8_t data;
} ucf_line_t;

/**
 * @}
 *
 */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
 * @}
 *
 */

/** @defgroup LIS2DH12_Infos
 * @{
 *
 */

/** I2C Device Address 8 bit format if SA0=0 -> 31 if SA0=1 -> 33 **/
#define LIS2DH12_I2C_ADD_L   0x31U
#define LIS2DH12_I2C_ADD_H   0x33U

/**
 * @}
 *
 */

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

#define CTRL_REG1_REGADDR   0x20U
#define CTRL_REG1_ODR(x)  ((x<<4)&0xF0) // b7:4 are ODR 3:0
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
    highpassCutoff_0 = 0,
    highpassCutoff_1,
    highpassCutoff_2,
    highpassCutoff_3,
} ctrl2_hpcf_t;
#define CTRL_REG2_FILTERED_DATA 0x08
#define CTRL_REG2_HP_CLICK      0x04
#define CTRL_REG2_HP_IA2        0x02
#define CTRL_REG2_HP_IA1        0x01

#define CTRL_REG3_REGADDR   0x22U
#define CTRL_REG3_
#define CTRL_REG3_
#define CTRL_REG3_
#define CTRL_REG3_
#define CTRL_REG3_
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t not_used_01 :1;
    uint8_t i1_overrun :1;
    uint8_t i1_wtm :1;
    uint8_t not_used_02 :1;
    uint8_t i1_zyxda :1;
    uint8_t i1_ia2 :1;
    uint8_t i1_ia1 :1;
    uint8_t i1_click :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t i1_click          : 1;
  uint8_t i1_ia1            : 1;
  uint8_t i1_ia2            : 1;
  uint8_t i1_zyxda          : 1;
  uint8_t not_used_02       : 1;
  uint8_t i1_wtm            : 1;
  uint8_t i1_overrun        : 1;
  uint8_t not_used_01       : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_ctrl_reg3_t;

#define CTRL_REG4_REGADDR           0x23U
#define CTRL_REG4_BLOCKDATAUPDATE   0x80
#define CTRL_REG4_DATALITTLEENDIAN  0x40 // little endian if set, big if clear
#define CTRL_REG4_FSCALE_MASK       0x30 // fs 1:0
typedef enum
{
    fullScale_2g = 0,
    fullScale_4g,
    fullScale_8g,
    fullScale_16g
} ctrl_reg4_fullscale_t;
#define CTRL_REG4_HR            0x10
#define CTRL_REG4_SELFTST_MASK  0x0C // st 1:0
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

#define STATUS_REGADDR          0x27U
#define STATUS_REG_XYZ_OVERRUN  0x80
#define STATUS_REG_Z_OVERRUN    0x40
#define STATUS_REG_Y_OVERRUN    0x20
#define STATUS_REG_X_OVERRUN    0x10
#define STATUS_REG_XYZ_NEWDATA  0x80
#define STATUS_REG_Z_NEWDATA    0x40
#define STATUS_REG_Y_NEWDATA    0x20
#define STATUS_REG_X_NEWDATA    0x10

#define OUT_X_L_REGADDR 0x28U
#define OUT_X_H_REGADDR 0x29U
#define OUT_Y_L_REGADDR 0x2AU
#define OUT_Y_H_REGADDR 0x2BU
#define OUT_Z_L_REGADDR 0x2CU
#define OUT_Z_H_REGADDR 0x2DU

#define FIFO_CTRL_REGADDR           0x2EU
#define FIFO_CTRL_MODE_MASK         0xC0
#define FIFO_CTRL_MODE_BYPASS       0x00
#define FIFO_CTRL_MODE_FIFO         0x40
#define FIFO_CTRL_MODE_STREAM       0x80
#define FIFO_CTRL_MODE_STRM_TO_FIFO 0xC0
#define FIFO_CTRL_TRIGGER_MASK      0x20
#define FIFO_CTRL_THR_MASK          0x1F

#define FIFO_SRC_REGADDR        0x2FU
#define FIFO_SRC_WTM            0x80
#define FIFO_SRC_OVRN           0x40
#define FIFO_SRC_EMPTY          0x20
#define FIFO_SRC_NUMUNREAD_MASK 0x1F

#define INT1_CFG_REGADDR    0x30U
#define INT1_CFG_AOI        0x80
#define INT1_CFG_6D         0x40
#define INT1_CFG_ZHIE       0x20
#define INT1_CFG_XLIE       0x10
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
#define int1_THS_THS_MASK   0x7F

#define INT1_DURATION_REGADDR  0x33U
// b7 not used
#define INT1_DURATION_MASK  0x7F

#define INT2_CFG_REGADDR    0x34U
#define INT2_CFG_AOI        0x80
#define INT2_CFG_6D         0x40
#define INT2_CFG_ZHIE       0x20
#define INT2_CFG_XLIE       0x10
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
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t ths :7;
    uint8_t not_used_01 :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ths               : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_int2_ths_t;

#define LIS2DH12_INT2_DURATION        0x37U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t d :7;
    uint8_t not_used_01 :1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t d                 : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_int2_duration_t;

#define LIS2DH12_CLICK_CFG            0x38U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
    uint8_t xs :1;
    uint8_t xd :1;
    uint8_t ys :1;
    uint8_t yd :1;
    uint8_t zs :1;
    uint8_t zd :1;
    uint8_t not_used_01 :2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 2;
  uint8_t zd                : 1;
  uint8_t zs                : 1;
  uint8_t yd                : 1;
  uint8_t ys                : 1;
  uint8_t xd                : 1;
  uint8_t xs                : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh12_click_cfg_t;

#define LIS2DH12_CLICK_SRC            0x39U
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

#define LIS2DH12_CLICK_THS            0x3AU
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

#define LIS2DH12_TIME_LIMIT           0x3BU
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

#define LIS2DH12_TIME_LATENCY         0x3CU
typedef struct
{
    uint8_t tla :8;
} lis2dh12_time_latency_t;

#define LIS2DH12_TIME_WINDOW          0x3DU
typedef struct
{
    uint8_t tw :8;
} lis2dh12_time_window_t;

#define LIS2DH12_ACT_THS              0x3EU
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

#define LIS2DH12_ACT_DUR              0x3FU
typedef struct
{
    uint8_t actd :8;
} lis2dh12_act_dur_t;

/**
 * @defgroup LIS2DH12_Register_Union
 * @brief    This union group all the registers having a bit-field
 *           description.
 *           This union is useful but it's not needed by the driver.
 *
 *           REMOVING this union you are compliant with:
 *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
 *
 * @{
 *
 */
typedef union
{
    lis2dh12_status_reg_aux_t status_reg_aux;
    lis2dh12_ctrl_reg0_t ctrl_reg0;
    lis2dh12_temp_cfg_reg_t temp_cfg_reg;
    lis2dh12_ctrl_reg1_t ctrl_reg1;
    lis2dh12_ctrl_reg2_t ctrl_reg2;
    lis2dh12_ctrl_reg3_t ctrl_reg3;
    lis2dh12_ctrl_reg4_t ctrl_reg4;
    lis2dh12_ctrl_reg5_t ctrl_reg5;
    lis2dh12_ctrl_reg6_t ctrl_reg6;
    lis2dh12_status_reg_t status_reg;
//    lis2dh12_fifo_ctrl_reg_t fifo_ctrl_reg;
    lis2dh12_fifo_src_reg_t fifo_src_reg;
    lis2dh12_int1_cfg_t int1_cfg;
    lis2dh12_int1_src_t int1_src;
    lis2dh12_int1_ths_t int1_ths;
    lis2dh12_int1_duration_t int1_duration;
    lis2dh12_int2_cfg_t int2_cfg;
    lis2dh12_int2_src_t int2_src;
    lis2dh12_int2_ths_t int2_ths;
    lis2dh12_int2_duration_t int2_duration;
    lis2dh12_click_cfg_t click_cfg;
    lis2dh12_click_src_t click_src;
    lis2dh12_click_ths_t click_ths;
    lis2dh12_time_limit_t time_limit;
    lis2dh12_time_latency_t time_latency;
    lis2dh12_time_window_t time_window;
    lis2dh12_act_ths_t act_ths;
    lis2dh12_act_dur_t act_dur;
    bitwise_t bitwise;
    uint8_t byte;
} lis2dh12_reg_t;

typedef enum
{
    LIS2DH12_TEMP_DISABLE = 0,
    LIS2DH12_TEMP_ENABLE = 3,
} lis2dh12_temp_en_t;

typedef enum
{
    LIS2DH12_HR_12bit = 0,
    LIS2DH12_NM_10bit = 1,
    LIS2DH12_LP_8bit = 2,
} lis2dh12_op_md_t;

typedef enum
{
    LIS2DH12_POWER_DOWN = 0x00,
    LIS2DH12_ODR_1Hz = 0x01,
    LIS2DH12_ODR_10Hz = 0x02,
    LIS2DH12_ODR_25Hz = 0x03,
    LIS2DH12_ODR_50Hz = 0x04,
    LIS2DH12_ODR_100Hz = 0x05,
    LIS2DH12_ODR_200Hz = 0x06,
    LIS2DH12_ODR_400Hz = 0x07,
    LIS2DH12_ODR_1kHz620_LP = 0x08,
    LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP = 0x09,
} lis2dh12_odr_t;

typedef enum
{
    LIS2DH12_AGGRESSIVE = 0,
    LIS2DH12_STRONG = 1,
    LIS2DH12_MEDIUM = 2,
    LIS2DH12_LIGHT = 3,
} lis2dh12_hpcf_t;

typedef enum
{
    LIS2DH12_NORMAL_WITH_RST = 0,
    LIS2DH12_REFERENCE_MODE = 1,
    LIS2DH12_NORMAL = 2,
    LIS2DH12_AUTORST_ON_INT = 3,
} lis2dh12_hpm_t;

typedef enum
{
    LIS2DH12_2g = 0,
    LIS2DH12_4g = 1,
    LIS2DH12_8g = 2,
    LIS2DH12_16g = 3,
} lis2dh12_fs_t;

/*************************************************************************************
 *  Variables
 ************************************************************************************/

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
    return i2c_writeBytes(LIS2DH12_I2C_ADD_L, toWrite, 2);
}

static ret_code_t readAccelReg(uint8_t regAddr, uint8_t* pRxByte)
{
    return i2c_readByte(LIS2DH12_I2C_ADD_L, regAddr, pRxByte);
}

static ret_code_t accel_operating_mode_set(const stmdev_ctx_t* ctx, lis2dh12_op_md_t val)
{
    lis2dh12_ctrl_reg1_t ctrl_reg1;
    lis2dh12_ctrl_reg4_t ctrl_reg4;
    ret_code_t ret;

    ret = readAccelReg(LIS2DH12_CTRL_REG1, (uint8_t*)&ctrl_reg1);

    if (ret == 0)
    {
        ret = readAccelReg(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4);
    }

    if (ret == 0)
    {
        if (val == LIS2DH12_HR_12bit)
        {
            ctrl_reg1.lpen = 0;
            ctrl_reg4.hr = 1;
        }

        if (val == LIS2DH12_NM_10bit)
        {
            ctrl_reg1.lpen = 0;
            ctrl_reg4.hr = 0;
        }

        if (val == LIS2DH12_LP_8bit)
        {
            ctrl_reg1.lpen = 1;
            ctrl_reg4.hr = 0;
        }

        ret = writeAccelReg(LIS2DH12_CTRL_REG1, (uint8_t)*((uint8_t*)&ctrl_reg1));
    }

    if (ret == 0)
    {
        ret = writeAccelReg(LIS2DH12_CTRL_REG4, (uint8_t)*((uint8_t*)&ctrl_reg4));
    }

    return ret;
}
static void accelInit(void)
{
    // Enable Stream mode, TODO bits 4:0, what should we set?
    writeAccelReg(FIFO_CTRL_REG_ADDR, FIFO_CTRL_MODE_STREAM);
    // Enable FIFO in ctrl_reg5
    uint8_t byte;
    readAccelReg (ctrl_reg5)
}
static void poll(void)
{

}
void lis2dh_init(void)
{

    i2c_init(); // Make sure bus is enabled.
    uint8_t data = 0x00;
    ret_code_t ret = readAccelReg(LIS2DH12_WHO_AM_I, &data);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("LIS2DH init failed to read I2C, error %d", ret);
        return;
    }
    if (LIS2DH12_WHOAMI_VALUE != data)
    {
        NRF_LOG_ERROR("Read LIS2DH WHOAMI 0x%x, SHOULD BE 0x%x", data, LIS2DH12_WHOAMI_VALUE);
        return;
    }
    // TODO init LIS2DH12 parameters the way we want.
    accelInit();
    pollers_registerPoller(poll);
}
