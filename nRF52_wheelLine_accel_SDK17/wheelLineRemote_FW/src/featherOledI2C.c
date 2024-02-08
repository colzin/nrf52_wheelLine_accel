/*
 * featherOledI2C.c
 *
 *  Created on: Feb 7, 2024
 *      Author: Collin Moore
 */

#include "featherOledI2C.h"

#include "i2c1.h"

#include "version.h"

#define NRF_LOG_MODULE_NAME FeatherOled
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define DEV_I2C_ADDR 0x3C

/*************************************************************************************
 *  Variables
 ************************************************************************************/

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

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

static void featherOledI2CPoll(void)
{

}

void featherOledI2C_init(void)
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
    pollers_registerPoller(featherOledI2CPoll);

}
