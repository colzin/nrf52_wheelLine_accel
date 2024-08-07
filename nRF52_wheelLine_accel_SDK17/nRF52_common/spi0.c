/*
 * spi0.c
 *
 *  Created on: Feb 2, 2024
 *      Author: Collin Moore
 */

#include "sdk_config.h"

#include "spi0.h"

#if COMPILE_RADIO_CC1101
#if NRFX_SPI_ENABLED

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "pollers.h"
#include "uptimeCounter.h"
#include "version.h"

#define NRF_LOG_MODULE_NAME spi0
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/*************************************************************************************
 *  Variables
 ************************************************************************************/

#if NRFX_SPI0_ENABLED
static const nrfx_spi_t m_spi = NRFX_SPI_INSTANCE(0); // TODO keep in sync with sdk_config.h
#else
#error "Please define or enable an SPI instance"
#endif // #if NRFX_SPI2_ENABLED

#define HIGH_DRIVE_PINS 0 // 1 to help with long wires

#define NONBLOCKING 0 // 1 to use non-blocking

#if NONBLOCKING
static volatile bool m_spiXferDone;
#endif // #if NONBLOCKING

static bool m_initted = false;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

#if NONBLOCKING
static void spiIsrHandler(nrfx_spi_evt_t const* p_event, void* p_context)
{
    switch (p_event->type)
    {
        case NRFX_SPI_EVENT_DONE:
            m_spiXferDone = true;
        break;
        default:
            NRF_LOG_WARNING("SPI event %d not handled", p_event->type)
            ;
        break;
    }
}
#endif // #if NONBLOCKING

static void setupCSOutputs(void)
{
#if COMPILE_FOR_FEATHER
    NRF_P0->OUTSET = (1UL << SPI0_EINK_CS_GPIO);
#if HIGH_DRIVE_PINS
    nrf_gpio_cfg(SPI0_EINK_CS_GPIO,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_H0H1,
                 NRF_GPIO_PIN_NOSENSE);
#else
    nrf_gpio_cfg_output(SPI0_EINK_CS_GPIO);
#endif // #if HIGH_DRIVE_PINS

    NRF_P0->OUTSET = (1UL << SPI0_EINK_SRAM_CS_GPIO);
#if HIGH_DRIVE_PINS
        nrf_gpio_cfg(SPI0_EINK_SRAM_CS_GPIO,
                     NRF_GPIO_PIN_DIR_OUTPUT,
                     NRF_GPIO_PIN_INPUT_DISCONNECT,
                     NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_H0H1,
                     NRF_GPIO_PIN_NOSENSE);
    #else
    nrf_gpio_cfg_output(SPI0_EINK_SRAM_CS_GPIO);
#endif // #if HIGH_DRIVE_PINS

    NRF_P0->OUTSET = (1UL << SPI0_SDCARD_CS_GPIO);
#if HIGH_DRIVE_PINS
            nrf_gpio_cfg(SPI0_SDCARD_CS_GPIO,
                         NRF_GPIO_PIN_DIR_OUTPUT,
                         NRF_GPIO_PIN_INPUT_DISCONNECT,
                         NRF_GPIO_PIN_NOPULL,
                         NRF_GPIO_PIN_H0H1,
                         NRF_GPIO_PIN_NOSENSE);
        #else
    nrf_gpio_cfg_output(SPI0_SDCARD_CS_GPIO);
#endif // #if HIGH_DRIVE_PINS

#endif // #if COMPILE_FOR_FEATHER

#if COMPILE_RADIO_CC1101
    NRF_P0->OUTSET = (1UL << SPI0_CC1101_CS_GPIO);
#if HIGH_DRIVE_PINS
    nrf_gpio_cfg(SPI0_CC1101_CS_GPIO,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_H0H1,
                 NRF_GPIO_PIN_NOSENSE);
#else
    nrf_gpio_cfg_output(SPI0_CC1101_CS_GPIO);
#endif // #if HIGH_DRIVE_PINS
#endif // #if COMPILE_RADIO_CC1101
}

static void assertCS(spi0Slave_t slave)
{ // TODO keep up with any new slaves added
    switch (slave)
    {
        case spi0_cc1101:
            #if COMPILE_RADIO_CC1101
            NRF_P0->OUTCLR = (1UL << SPI0_CC1101_CS_GPIO);
#endif // #if COMPILE_RADIO_CC1101
        break;
#if COMPILE_FOR_FEATHER
        case spi0_einkScreen:
            NRF_P0->OUTCLR = (1UL << SPI0_EINK_CS_GPIO);
        break;
        case spi0_einkSRAM:
            NRF_P0->OUTCLR = (1UL << SPI0_EINK_SRAM_CS_GPIO);
        break;
        case spi0_sdcard:
            NRF_P0->OUTCLR = (1UL << SPI0_SDCARD_CS_GPIO);
        break;
#endif // #if COMPILE_FOR_FEATHER
        default:
            NRF_LOG_ERROR("SPI0 slave %d not supported!", slave)
            ;
        break;
    }
}

static void deassertCS(void)
{    // TODO add any other slave CS lines
#if COMPILE_FOR_FEATHER
#if COMPILE_RADIO_CC1101
    NRF_P0->OUTSET = (1UL << SPI0_CC1101_CS_GPIO)
            | (1UL << SPI0_EINK_CS_GPIO)
            | (1UL << SPI0_EINK_SRAM_CS_GPIO)
            | (1UL << SPI0_SDCARD_CS_GPIO);
#else
    NRF_P0->OUTSET = (1UL << SPI0_EINK_CS_GPIO)
                | (1UL << SPI0_EINK_SRAM_CS_GPIO)
                | (1UL << SPI0_SDCARD_CS_GPIO);
#endif // #if COMPILE_RADIO_CC1101
#elif COMPILE_FOR_PCA10040
#if COMPILE_RADIO_CC1101
    NRF_P0->OUTSET = (1UL << SPI0_CC1101_CS_GPIO);
#endif // #if COMPILE_RADIO_CC1101
#endif // #if COMPILE_FOR_FEATHER
//    NRF_LOG_DEBUG("Deasserted");
}

ret_code_t spi0_write(spi0Slave_t slave, uint8_t* pData, uint32_t len, bool keepCSAsserted)
{
    if (!m_initted)
    {
        spi0_init();
    }
#if NONBLOCKING
    if (!m_spiXferDone)
    {
        NRF_LOG_ERROR("Can't send, still in progress sending a transfer");
        return NRF_ERROR_BUSY;
    }
    // Start the data Tx
    m_spiXferDone = false;
#endif // #if NONBLOCKING

    nrfx_spi_xfer_desc_t xfer;
    ret_code_t ret;
    uint32_t offset = 0;
    assertCS(slave);
    // Note, limit per xfer is 511 bytes. To send more, use loop
    while (len)
    {
        xfer.p_tx_buffer = pData + offset;
        xfer.tx_length = len > 511 ? 511 : len;
        xfer.p_rx_buffer = NULL;
        xfer.rx_length = 0;
#if NONBLOCKING
        m_spiXferDone = false;
#endif // #if NONBLOCKING
        ret = nrfx_spi_xfer(&m_spi, &xfer, 0);
//        NRF_LOG_DEBUG("Wxfer tx %d, rx %d", xfer.tx_length, xfer.rx_length);
        if (NRF_SUCCESS != ret)
        { // Bail out, release CS line
            NRF_LOG_ERROR("SPI0 write, xfer error %d, bailing out", ret);
            if (!keepCSAsserted)
            {
                deassertCS();
            }
#if NONBLOCKING
            m_spiXferDone = true;
#endif // #if NONBLOCKING
            return ret;
        }

#if NONBLOCKING
        uint32_t retries = 1000000;
        while (!m_spiXferDone)
        {
            nrf_delay_us(1);
            retries--;
            if (!retries)
            {
                break;
            }
        }
        if (!retries)
        {
            NRF_LOG_ERROR("SPI0 write: xfer Retries exceeded");
            ret = NRF_ERROR_RESOURCES;
            break;
        }
#endif // #if NONBLOCKING
        len -= xfer.tx_length;
        offset += xfer.tx_length;
    }
    // Now we are done, successfully
//    NRF_LOG_DEBUG("Wrote 0x%x, %d long", *pData, offset);
    if (!keepCSAsserted)
    {
        deassertCS();
    }
#if NONBLOCKING
    m_spiXferDone = true;
#endif // #if NONBLOCKING
    return ret;
}

ret_code_t spi0_read(spi0Slave_t slave, uint8_t* pData, uint32_t len, bool keepCSAsserted)
{
    if (!m_initted)
    {
        spi0_init();
    }
#if NONBLOCKING
    if (!m_spiXferDone)
    {
        NRF_LOG_ERROR("Can't send, still in progress sending a transfer");
        return NRF_ERROR_BUSY;
    }
    // Start the data Tx
    m_spiXferDone = false;
#endif // #if NONBLOCKING

    nrfx_spi_xfer_desc_t xfer;
    ret_code_t ret;
    uint32_t offset = 0;
    assertCS(slave);
    // Note, limit per xfer is 511 bytes. To send more, use loop
    while (len)
    {
        xfer.p_tx_buffer = NULL;
        xfer.tx_length = 0;
        xfer.p_rx_buffer = pData + offset;
        xfer.rx_length = len > 511 ? 511 : len;
#if NONBLOCKING
        m_spiXferDone = false;
#endif // #if NONBLOCKING
        ret = nrfx_spi_xfer(&m_spi, &xfer, 0);
//        NRF_LOG_DEBUG("Rxfer tx %d, rx %d", xfer.tx_length, xfer.rx_length);
        if (NRF_SUCCESS != ret)
        { // Bail out, release CS line
            NRF_LOG_ERROR("SPI0 read, xfer error %d, bailing out", ret);
            if (!keepCSAsserted)
            {
                deassertCS();
            }
#if NONBLOCKING
            m_spiXferDone = true;
#endif // #if NONBLOCKING
            return ret;
        }

#if NONBLOCKING
        uint32_t retries = 1000000;
        while (!m_spiXferDone)
        {
            nrf_delay_us(1);
            retries--;
            if (!retries)
            {
                break;
            }
        }
        if (!retries)
        {
            NRF_LOG_ERROR("SPI0 read: xfer Retries exceeded");
            ret = NRF_ERROR_RESOURCES;
            break;
        }
#endif // #if NONBLOCKING
        len -= xfer.rx_length;
        offset += xfer.rx_length;
    }
    // Now we are done, successfully
    if (!keepCSAsserted)
    {
        deassertCS();
    }
#if NONBLOCKING
    m_spiXferDone = true;
#endif // #if NONBLOCKING
    return NRF_SUCCESS;
}

ret_code_t spi0_xfer(spi0Slave_t slave, nrfx_spi_xfer_desc_t* pXfer, bool keepCSAsserted)
{
    if (!m_initted)
    {
        spi0_init();
    }
#if NONBLOCKING
    if (!m_spiXferDone)
    {
        NRF_LOG_ERROR("Can't send, still in progress sending a transfer");
        return NRF_ERROR_BUSY;
    }
    // Start the data Tx
    m_spiXferDone = false;
#endif // #if NONBLOCKING

    assertCS(slave);

#if NONBLOCKING
        m_spiXferDone = false;
#endif // #if NONBLOCKING
    ret_code_t ret = nrfx_spi_xfer(&m_spi, pXfer, 0);
//    NRF_LOG_DEBUG("xfer tx %d, rx %d", pXfer->tx_length, pXfer->rx_length);
    if (NRF_SUCCESS != ret)
    { // Bail out, release CS line
        NRF_LOG_ERROR("SPI0 write, xfer error %d, bailing out", ret);
        if (!keepCSAsserted)
        {
            deassertCS();
        }
#if NONBLOCKING
            m_spiXferDone = true;
#endif // #if NONBLOCKING
        return ret;
    }

#if NONBLOCKING
        uint32_t retries = 1000000;
        while (!m_spiXferDone)
        {
            nrf_delay_us(1);
            retries--;
            if (!retries)
            {
                break;
            }
        }
        if (!retries)
        {
            NRF_LOG_ERROR("SPI0 xfer: xfer Retries exceeded");
            ret = NRF_ERROR_RESOURCES;
            break;
        }
#endif // #if NONBLOCKING

    if (!keepCSAsserted)
    {
        deassertCS();
    }
#if NONBLOCKING
    m_spiXferDone = true;
#endif // #if NONBLOCKING
    return ret;
}

bool spi0_isInitted(void)
{
    return m_initted;
}

void spi0_init(void)
{
    if (!m_initted)
    {
        setupCSOutputs();
// Set up I2S to send data
//    nrfx_spi_uninit(&m_spi);
        nrfx_spi_config_t config;
        config.bit_order = NRF_SPI_BIT_ORDER_MSB_FIRST;
        /* CC1101 specifies max SPI CLK freq 6.5MHz, but no min.
         * TODO e-ink max SPI freq?
         * TODO other slaves max/min SPI freq?
         */
        config.frequency = NRF_SPI_FREQ_125K;
        config.irq_priority = APP_IRQ_PRIORITY_LOW;
        config.miso_pin = SPI0_MISO_PIN;
        config.mode = NRF_SPI_MODE_0; // TODO determine mode for CC1101 and e-ink
        config.mosi_pin = SPI0_MOSI_PIN;
        config.orc = 0x00; // TODO send zeros or something else?
        config.sck_pin = SPI0_SCK_PIN;
        config.ss_pin = NRFX_SPI_PIN_NOT_USED;

#if NONBLOCKING
        ret_code_t ret = nrfx_spi_init(&m_spi, &config, spiIsrHandler, NULL);
        m_spiXferDone = true;
#else
        ret_code_t ret = nrfx_spi_init(&m_spi, &config, NULL, NULL);
#endif // #if NONBLOCKING

        if (NRF_SUCCESS != ret)
        {
            NRF_LOG_ERROR("nrfx_spi_init Error 0x%x", ret);
            return;
        }

#if HIGH_DRIVE_PINS
        nrf_gpio_cfg(config.sck_pin,
                     NRF_GPIO_PIN_DIR_OUTPUT,
                     NRF_GPIO_PIN_INPUT_CONNECT,
                     NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_H0H1,
                     NRF_GPIO_PIN_NOSENSE);
        nrf_gpio_cfg(config.mosi_pin,
                     NRF_GPIO_PIN_DIR_OUTPUT,
                     NRF_GPIO_PIN_INPUT_DISCONNECT,
                     NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_H0H1,
                     NRF_GPIO_PIN_NOSENSE);

#endif // #if HIGH_DRIVE_PINS

//        NRF_LOG_INFO("SPI0 initted");
        m_initted = true;
    }
}
#endif // #if NRFX_SPI_ENABLED

#endif // #if COMPILE_RADIO_CC1101
