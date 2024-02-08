/*
 * i2c.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Collin Moore
 */

#include "sdk_config.h"

#include "i2c1.h"

#include "pollers.h"
#include "nrfx_twi.h"
#include "uptimeCounter.h"

#include "version.h"

#define NRF_LOG_MODULE_NAME I2C
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define SCANNER_POLL_ITVL_MS 0 // Define non-zero to scan every ms

/*************************************************************************************
 *  Variables
 ************************************************************************************/

#if NRFX_TWI1_ENABLED
/* TWI instance. */
static const nrfx_twi_t m_twi1 = NRFX_TWI_INSTANCE(1);
static const nrfx_twi_config_t m_twi1Cfg =
        {
          .scl = I2C1_SCL_PIN,
          .sda = I2C1_SDA_PIN,
          .frequency = NRF_TWI_FREQ_400K,
          .hold_bus_uninit = false,
          .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
        };
#else
#error "Please enable NRFX_TWI1"
#endif // #if NRFX_TWI1_ENABLED

static bool m_twi1Enabled = false;

#if SCANNER_POLL_ITVL_MS
static uint32_t m_lastPoll_ms;
#include "uptimeCounter.h"
#endif // #if SCANNER_POLL_ITVL_MS
/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/
#if SCANNER_POLL_ITVL_MS
static void evtHandler(nrfx_twi_evt_t const* p_event, void* p_context)
{
    NRF_LOG_WARNING("nrfx_twi_evt_t type  %d", p_event->type);
}

static void scanAllAddresses(void)
{
    ret_code_t ret;
    uint8_t data;
    for (uint32_t i = 0; i < 256; i++)
    {
        ret = nrfx_twi_rx(&m_twi1, (uint8_t)i, &data, 1);
        if (NRF_SUCCESS == ret)
        {
            NRF_LOG_INFO("Device detected at address 0x%x", i);
        }
    }
}
static void i2c1Poll(void)
{
    if (uptimeCounter_elapsedSince(m_lastPoll_ms) > SCANNER_POLL_ITVL_MS)
    {
        scanAllAddresses();
        m_lastPoll_ms = uptimeCounter_getUptimeMs();
    }
}
#endif // #if SCANNER_POLL_ITVL_MS

ret_code_t i2c1_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t* pData)
{ // Write the address, then do a stop, then read as many bytes as caller wants
    ret_code_t ret = nrfx_twi_tx(&m_twi1, devAddr, &regAddr, 1, true);

    if (NRF_SUCCESS != ret)
    {
        if (NRFX_ERROR_DRV_TWI_ERR_ANACK == ret)
        {
            NRF_LOG_WARNING("i2c1_readByte Address 0x%x NACKed", devAddr);
        }
        else
        {
            NRF_LOG_ERROR("i2c1_readByte Error 0x%x writing devAddr 0x%x", ret, devAddr);
        }
        return ret;
    }
    ret = nrfx_twi_rx(&m_twi1, devAddr, pData, 1);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("i2c1_readByte Error 0x%x reading byte", ret);
        return ret;
    }
    return ret;
}

ret_code_t i2c1_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t* pData, uint32_t len)
{ // Write the address, then do a stop, then read as many bytes as caller wants
    ret_code_t ret = nrfx_twi_tx(&m_twi1, devAddr, &regAddr, 1, true);
    if (NRF_SUCCESS != ret)
    {
        if (NRFX_ERROR_DRV_TWI_ERR_ANACK == ret)
        {
            NRF_LOG_WARNING("i2c1_readBytes Address 0x%x NACKed", devAddr);
        }
        else
        {
            NRF_LOG_ERROR("i2c1_readByte Error 0x%x writing devAddr 0x%x", ret, devAddr);
        }
        return ret;
    }
    ret = nrfx_twi_rx(&m_twi1, devAddr, pData, len);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("i2c1_readBytes Error 0x%x reading bytes", ret);
        return ret;
    }
    return ret;
}

ret_code_t i2c1_writeBytes(uint8_t devAddr, uint8_t* pByte, uint32_t len)
{ // Write the address, then data bytes
    ret_code_t ret = nrfx_twi_tx(&m_twi1, devAddr, pByte, len, false);
    if (NRF_SUCCESS != ret)
    {
        if (NRFX_ERROR_DRV_TWI_ERR_ANACK == ret)
        {
            NRF_LOG_WARNING("i2c1_writeBytes Address 0x%x NACKed", devAddr);
        }
        else
        {
            NRF_LOG_ERROR("i2c1_writeBytes Error 0x%x writing %d bytes to devAddr 0x%x", ret, len, devAddr);
        }
        return ret;
    }
    return ret;
}

ret_code_t i2c1_init(void)
{
    if (m_twi1Enabled)
    {
        return NRF_SUCCESS;
    }
    // If we provide a handler, calls will be non-blocking.
    ret_code_t ret = nrfx_twi_init(&m_twi1, &m_twi1Cfg, NULL, NULL);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("TWI_init err 0x%x", ret);
        return ret;
    }
    else
    {
        // Mux pullups on the pins for the bus to work, ONLY if no HW pullup is on the lines
//        NRF_P0->PIN_CNF[m_twi1Cfg.scl] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
//        NRF_P0->PIN_CNF[m_twi1Cfg.sda] |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
        nrfx_twi_enable(&m_twi1);
        m_twi1Enabled = true;
#if SCANNER_POLL_ITVL_MS
        m_lastPoll_ms = uptimeCounter_getUptimeMs();
        pollers_registerPoller(i2c1Poll);
#endif // #if SCANNER_POLL_ITVL_MS
    }
    return NRF_SUCCESS;
}
