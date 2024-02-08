/*
 * i2c1.h
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_I2C1_H_
#define SRC_I2C1_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "sdk_errors.h"

ret_code_t i2c1_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t* pData);
ret_code_t i2c1_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t* pData, uint32_t len);
ret_code_t i2c1_writeBytes(uint8_t devAddr, uint8_t* pByte, uint32_t len);
ret_code_t i2c1_tx(uint8_t devAddr, uint8_t* pByte, uint32_t len, bool repeatedStart);

ret_code_t i2c1_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_I2C1_H_ */
