/*
 * i2c.h
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sdk_errors.h"

ret_code_t i2c_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t* pData);
ret_code_t i2c_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t* pData, uint32_t len);
ret_code_t i2c_writeBytes(uint8_t devAddr, uint8_t* pByte, uint32_t len);
ret_code_t i2c_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_I2C_H_ */
