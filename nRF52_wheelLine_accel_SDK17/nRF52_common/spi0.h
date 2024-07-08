/*
 * spi0.h
 *
 *  Created on: Feb 5, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_SPI0_H_
#define SRC_SPI0_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "version.h"
#if COMPILE_RADIO_CC1101
#include "nrfx_spi.h" // Use base SPI, not fancy eDMA SPIM
#include "sdk_errors.h"

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

typedef enum
{
    spi0_cc1101,
    spi0_einkScreen,
    spi0_einkSRAM,
    spi0_sdcard,
    spi0_none,

}spi0Slave_t;

/*************************************************************************************
 *  Functions
 ************************************************************************************/
ret_code_t spi0_write(spi0Slave_t slave,uint8_t* pData, uint32_t len, bool keepCSAsserted);

ret_code_t spi0_read(spi0Slave_t slave,uint8_t* pData, uint32_t len, bool keepCSAsserted);

ret_code_t spi0_xfer(spi0Slave_t slave, nrfx_spi_xfer_desc_t* pXfer, bool keepCSAsserted);

bool spi0_isInitted(void);

void spi0_init(void);


#endif // #if COMPILE_RADIO_CC1101

#ifdef __cplusplus
}
#endif

#endif /* SRC_SPI0_H_ */
