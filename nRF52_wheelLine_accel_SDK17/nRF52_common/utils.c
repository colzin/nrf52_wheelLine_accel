/*
 * utils.c
 *
 *  Created on: Jan 19, 2024
 *      Author: Collin Moore
 */

#include "utils.h"

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/*************************************************************************************
 *  Variables
 ************************************************************************************/

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/


uint8_t* utils_writeLEUint32(uint8_t* pBuffer, uint32_t data)
{
    *pBuffer++ = (uint8_t)(data & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 8) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 16) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 24) & 0xFF);
    return pBuffer;
}

uint8_t* utils_writeLEUint16(uint8_t* pBuffer, uint16_t data)
{
    *pBuffer++ = (uint8_t)(data & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 8) & 0xFF);
    return pBuffer;
}

uint8_t* utils_readBEUint32(uint8_t* pBuffer, uint32_t* data)
{
    uint32_t i;
    *data = 0;
    for (i = 0; i < sizeof(uint32_t); i++)
    {
        *data <<= 8;
        *data += (uint32_t)*pBuffer++;
    }
    return pBuffer;
}

uint8_t* utils_writeBEUint32(uint8_t* pBuffer, uint32_t data)
{
    *pBuffer++ = (uint8_t)((data >> 24) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 16) & 0xFF);
    *pBuffer++ = (uint8_t)((data >> 8) & 0xFF);
    *pBuffer++ = (uint8_t)(data & 0xFF);
    return pBuffer;
}

uint32_t utils_elapsedU32Ticks(uint32_t start, uint32_t end)
{
    if (end >= start)
    {
        return (end - start);
    }
    return (end + ((uint32_t)(-1) - start));
}
