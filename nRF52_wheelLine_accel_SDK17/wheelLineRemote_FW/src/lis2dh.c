/*
 * lis2dh.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#include "lis2dh.h"

#include "i2c.h"

#define NRF_LOG_MODULE_NAME lis2dh
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/
// Slave Address from datasheet is 0b001100x, and the x is pulled high
#define LIS2DH_ADDRESS 0b0011001

#define LIS2DH_WHOAMI_REGADDR 0x0F
#define LIS2DH_WHOAMI_VALUE 0b00110011
/*************************************************************************************
 *  Variables
 ************************************************************************************/

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

void lis2dh_init(void)
{

    i2c_init(); // Make sure bus is enabled.
    uint8_t data;
    ret_code_t ret = i2c_readRegs(LIS2DH_ADDRESS, LIS2DH_WHOAMI_REGADDR, &data, 1);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("LIS2DH init failed to read I2C");
        return;
    }
    if (LIS2DH_WHOAMI_VALUE != data)
    {
        NRF_LOG_ERROR("Read LIS2DH WHOAMI 0x%x, SHOULD BE 0x%x", data, LIS2DH_WHOAMI_VALUE);
        return;
    }
}
