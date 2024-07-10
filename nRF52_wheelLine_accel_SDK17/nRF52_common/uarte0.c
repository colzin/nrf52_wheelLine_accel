/*
 * uarte0.c
 *
 *  Created on: Feb 6, 2024
 *      Author: Collin Moore
 */

#include "uarte0.h"

#if COMPILE_RADIO_CC1101
#include "cc1101.h"
#endif // #if COMPILE_RADIO_CC1101
#if COMPILE_RADIO_900T20D
#include "_900t20d.h"
#endif // #if COMPILE_RADIO_900T20D

#include "globalInts.h"

#include "nrf_delay.h" // For reboot print
#include "nrf_drv_uart.h"
#include "pollers.h"
#include "version.h"// for pindefs

#define NRF_LOG_MODULE_NAME uarte0
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define VERBOSE_ISR_ERRORS 1 // 0 for faster ISR
#define VERBOSE_ISR_TX 0 // 0 for faster ISR
#define VERBOSE_ISR_RX 0 // 0 for faster ISR

/*************************************************************************************
 *  Variables
 ************************************************************************************/
// Driver
static nrf_drv_uart_t m_uartInst = NRF_DRV_UART_INSTANCE(0);

// Ring buffer

        static volatile uint8_t m_rxData[UARTE0_MAX_UART_DATA_LEN * 2 + 1];
        static volatile int32_t m_rxDataWriteIndex;
        static int32_t m_rxDataReadIndex;
        static volatile nrf_drv_uart_event_t m_uartEvent;
        static volatile bool m_dataDiffered,
m_txInProgress, m_uartRxError;

static uint8_t m_txData[UARTE0_MAX_UART_DATA_LEN * 2 + 1];
static int32_t m_txDataWriteIndex,
        m_txDataReadIndex;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static void uartEventHandler(nrf_drv_uart_event_t* p_event, void* p_context)
{
    (void)p_context;
    switch (p_event->type)
    {
        case NRF_DRV_UART_EVT_TX_DONE:
            m_txInProgress = false;
#if VERBOSE_ISR_TX
            NRF_LOG_DEBUG("TxDone")
            ;
#endif // #if VERBOSE_ISR_TX
        break;
        case NRF_DRV_UART_EVT_RX_DONE:
            { // Increment the indexes to log received byte
            if (p_event->data.rxtx.p_data[0] != m_rxData[m_rxDataWriteIndex])
            {
                m_dataDiffered = true;
#if VERBOSE_ISR_ERRORS
                NRF_LOG_ERROR("DATA differed");
#endif // #if VERBOSE_ISR_ERRORS
            }
            m_rxDataWriteIndex++;
            m_rxDataWriteIndex %= (int32_t)sizeof(m_rxData);
            // Start another RX
            ret_code_t ret = nrf_drv_uart_rx(&m_uartInst, (uint8_t*)&m_rxData[m_rxDataWriteIndex], 1);
            if (NRF_SUCCESS != ret)
            {
                m_uartRxError = true;
//#if VERBOSE_ISR_ERRORS // Always print this!
                NRF_LOG_ERROR("UART RX evt had ERROR starting next RX");
//#endif // #if VERBOSE_ISR_ERRORS
            }
            else
            {
#if VERBOSE_ISR_RX
                NRF_LOG_DEBUG("RX");
#endif // #if VERBOSE_ISR_RX
            }
        }
        break;

        case NRF_DRV_UART_EVT_ERROR:
            m_uartEvent.type = p_event->type;
            m_uartEvent.data = p_event->data;
#if VERBOSE_ISR_ERRORS
            NRF_LOG_ERROR("UART ERR mask 0x%x", p_event->data.error.error_mask)
            ;
#endif // #if VERBOSE_ISR_ERRORS
        break;
    }
}

static void trySend(void)
{
    if (!m_txInProgress)
    { // If we are not transmitting, send a chunk to USB
        int32_t numToSend = m_txDataWriteIndex - m_txDataReadIndex;
        if (numToSend < 0)
        { // It wrapped. Just send to end of array
            numToSend = (int32_t)sizeof(m_txData) - m_txDataReadIndex;
        }
        if (numToSend)
        {
            if (255 < numToSend)
            {
                numToSend = 255; // passed as uint8 to driver
            }
            m_txInProgress = true; // Set before start, before ISR clears
            ret_code_t ret = nrf_drv_uart_tx(&m_uartInst, &m_txData[m_txDataReadIndex], (uint8_t)numToSend);
            if (NRF_SUCCESS == ret)
            {
                m_txDataReadIndex += numToSend;
                m_txDataReadIndex %= (int32_t)sizeof(m_txData);
//                NRF_LOG_DEBUG("Started send of %d bytes", numToSend);
            }
            else
            {
                m_txInProgress = false;
                NRF_LOG_ERROR("nrf_drv_uart_tx error 0x%x", ret);
                return;
            }
        }

    }
}

ret_code_t uarte0_enqueue(const uint8_t* pBytes, uint32_t numBytes)
{
#if (UART_RX_PIN && UART_TX_PIN) || (_900T20D_UART_FROM_MODULE && _900T20D_UART_TO_MODULE)
    for (uint32_t i = 0; i < numBytes; i++)
    {
        if (((m_txDataWriteIndex + 1) % (int32_t)sizeof(m_txData)) == m_txDataReadIndex)
        {
            NRF_LOG_ERROR("Tried to enqueue %d to TX, only room for %d", numBytes, i);
            return NRF_ERROR_DATA_SIZE;
        }
        else
        {
            m_txData[m_txDataWriteIndex++] = pBytes[i];
            m_txDataWriteIndex %= (int32_t)sizeof(m_txData);
        }
    }
//    NRF_LOG_INFO("Enqueued %d to TX", numBytes);
    // Done, success
    if (!m_txInProgress)
    {
        trySend();
    }
    return NRF_SUCCESS;
#else // If no UART pins, ignore data
    // return ok, ignore
    return NRF_SUCCESS;
#endif // #if (UART_RX_PIN && UART_TX_PIN)
}

static bool componentInit(void)
{
    m_uartRxError = false;
    m_rxDataWriteIndex = 0;
    m_rxDataReadIndex = 0;
    m_dataDiffered = false;
    m_txInProgress = false;
    m_txDataWriteIndex = 0;
    m_txDataReadIndex = 0;
    m_txInProgress = false;
    m_uartEvent.type = 100; // Set invalid, no error

#if (UART_RX_PIN && UART_TX_PIN)
    nrf_drv_uart_config_t uartCfg;
    uartCfg.baudrate = NRF_UARTE_BAUDRATE_921600;
    uartCfg.hwfc = NRF_UARTE_HWFC_DISABLED;
    uartCfg.interrupt_priority = APP_IRQ_PRIORITY_LOW_MID;
    uartCfg.p_context = NULL;
    uartCfg.parity = NRF_UARTE_PARITY_EXCLUDED;
    uartCfg.pselcts = NRF_UARTE_PSEL_DISCONNECTED;
    uartCfg.pselrts = NRF_UARTE_PSEL_DISCONNECTED;
    uartCfg.pselrxd = UART_RX_PIN;
    uartCfg.pseltxd = UART_TX_PIN;
#elif (_900T20D_UART_FROM_MODULE && _900T20D_UART_TO_MODULE)
    nrf_drv_uart_config_t uartCfg;
    uartCfg.baudrate = NRF_UARTE_BAUDRATE_9600; // TODO baud rate
    uartCfg.hwfc = NRF_UARTE_HWFC_DISABLED;
    uartCfg.interrupt_priority = APP_IRQ_PRIORITY_LOW_MID;
    uartCfg.p_context = NULL;
    uartCfg.parity = NRF_UARTE_PARITY_EXCLUDED;
    uartCfg.pselcts = NRF_UARTE_PSEL_DISCONNECTED;
    uartCfg.pselrts = NRF_UARTE_PSEL_DISCONNECTED;
    uartCfg.pselrxd = _900T20D_UART_FROM_MODULE;
    uartCfg.pseltxd = _900T20D_UART_TO_MODULE;
#endif // #if usb or 900T20D

#if (UART_RX_PIN && UART_TX_PIN) || (_900T20D_UART_FROM_MODULE && _900T20D_UART_TO_MODULE)
    ret_code_t ret = nrf_drv_uart_init(&m_uartInst, &uartCfg, uartEventHandler);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("Couldn't set up UART, error 0x%x\n", ret);
        return false;
    }
    ret = nrf_drv_uart_rx(&m_uartInst, (uint8_t*)m_rxData, 1);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("nrf_drv_uart_rx error 0x%x\n", ret);
        return false;
    }
    return true;
#else
#warning "UART terminal not present"
    return true;
#endif // #if (UART_RX_PIN && UART_TX_PIN) || (_900T20D_UART_FROM_MODULE && _900T20D_UART_TO_MODULE)
}

static void uartPoll(void)
{
    if (m_uartRxError)
    {
        m_uartRxError = false;
        NRF_LOG_WARNING("UART RX restart error");
    }
    if (m_dataDiffered)
    {
        m_dataDiffered = false;
        NRF_LOG_WARNING("Data differed");
    }
    if (100 != m_uartEvent.type)
    {
        NRF_LOG_ERROR("UART event error.type %d, mask 0x%x", m_uartEvent.type,
                      m_uartEvent.data.error.error_mask);
        nrf_uarte_event_clear(m_uartInst.uarte.p_reg, NRF_UARTE_EVENT_TXDRDY);
        nrf_uarte_event_clear(m_uartInst.uarte.p_reg, NRF_UARTE_EVENT_RXTO);
        nrf_uarte_event_clear(m_uartInst.uarte.p_reg, NRF_UARTE_EVENT_ERROR);
        nrf_drv_uart_uninit(&m_uartInst);
        componentInit(); // Re-init it
    }
//    if (m_rxDataReadIndex != m_rxDataWriteIndex)
//    {
//        NRF_LOG_DEBUG("uartPoll new data");
//    }
    trySend(); // Send any bytes waiting to be sent
}

bool uarte0_isTxDone(void)
{
    uartPoll();
    return !m_txInProgress;
}

bool uarte0_tryReadByte(uint8_t* pByte)
{
    uartPoll();
    if (m_rxDataReadIndex != m_rxDataWriteIndex)
    {
//        NRF_LOG_INFO("RX byte 0x%x", m_rxData[m_rxDataReadIndex]);
        *pByte = m_rxData[m_rxDataReadIndex];
        m_rxDataReadIndex++;
        m_rxDataReadIndex %= (int32_t)sizeof(m_rxData);
        return true;
    }
    return false;
}

bool uarte0_init(void)
{
    if (componentInit())
    {
        pollers_registerPoller(uartPoll);
        return true;
    }
    return false;
}
