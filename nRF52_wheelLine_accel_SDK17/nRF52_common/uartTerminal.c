/*
 * uartTerminal.c
 *
 *  Created on: Feb 6, 2024
 *      Author: Collin Moore
 */

#include "uartTerminal.h"

#if COMPILE_RADIO_900T20D
#warning "can't use terminal with 900T20D"
#else

#include "globalInts.h"

#include "nrf_delay.h" // For reboot print
#include "pollers.h"
#include "version.h"// for pindefs

#define NRF_LOG_MODULE_NAME uartTerm
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define LF_CHAR 0xA
#define CR_CHAR 0xD

typedef enum
{
    parserState_default,
    parserState_receivingPower,
    parserState_receivingCloseIn,
} parserState_t;

/*************************************************************************************
 *  Variables
 ************************************************************************************/

// Parser
static parserState_t m_parserState;
static int32_t m_accumulator;
static bool m_negative;
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
        break;
        case NRF_DRV_UART_EVT_RX_DONE:
            { // Increment the indexes to log received byte
            if (p_event->data.rxtx.p_data[0] != m_rxData[m_rxDataWriteIndex])
            {
                m_dataDiffered = true;
#if VERBOSE_ISR
                NRF_LOG_ERROR("DATA differed");
#endif // #if VERBOSE_ISR
            }
            m_rxDataWriteIndex++;
            m_rxDataWriteIndex %= (int32_t)sizeof(m_rxData);
            // Start another RX
            ret_code_t ret = nrf_drv_uart_rx(&m_uartInst, (uint8_t*)&m_rxData[m_rxDataWriteIndex], 1);
            if (NRF_SUCCESS != ret)
            {
                m_uartRxError = true;
#if VERBOSE_ISR
                NRF_LOG_ERROR("UART RX evt, error starting next RX");
#endif // #if VERBOSE_ISR
            }
        }
        break;

        case NRF_DRV_UART_EVT_ERROR:
            m_uartEvent.type = p_event->type;
            m_uartEvent.data = p_event->data;
#if VERBOSE_ISR
            NRF_LOG_ERROR("UART ERR mask 0x%x", p_event->data.error.error_mask)
            ;
#endif // #if VERBOSE_ISR
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
            ret_code_t ret = nrf_drv_uart_tx(&m_uartInst, &m_txData[m_txDataReadIndex], (uint8_t)numToSend);
            if (NRF_SUCCESS == ret)
            {
                m_txInProgress = true; // Set quickly, before ISR clears
                m_txDataReadIndex += numToSend;
                m_txDataReadIndex %= (int32_t)sizeof(m_txData);
//                NRF_LOG_DEBUG("Started send of %d bytes", numToSend);
            }
            else
            {
                NRF_LOG_ERROR("nrf_drv_uart_tx error 0x%x", ret);
                return;
            }
        }

    }
}

ret_code_t uartTerminal_enqueueToUSB(const uint8_t* pBytes, uint32_t numBytes)
{
#if (UART_RX_PIN && UART_TX_PIN)
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

static void parsePowerdigit(uint8_t byte)
{
    if ('-' == byte)
    {
        m_negative = true;
    }
    else if ('0' <= byte && '9' >= byte)
    {
        m_accumulator *= 10;
        m_accumulator += (byte - '0');
    }
    else if (LF_CHAR == byte || CR_CHAR == byte)
    {
        if (m_negative)
        {
            m_accumulator = -1 * m_accumulator;
        }
#if COMPILE_RADIO_CC1101
        cc1101_setOutputPower((int8_t)m_accumulator);
#endif // #if COMPILE_RADIO_CC1101
#if COMPILE_RADIO_900T20D
        _900t20d_setOutputPower((int8_t)m_accumulator);
#endif // #if COMPILE_RADIO_900T20D
        m_parserState = parserState_default;
    }
    else
    {
        NRF_LOG_WARNING("Error, moving to default");
        m_parserState = parserState_default;
    }
}

static void parseCloseIn(uint8_t byte)
{
    if ('0' <= byte && '3' >= byte)
    {
        cc1101_setCloseInRx(byte - '0');
    }
    else
    {
        NRF_LOG_WARNING("Error, moving to default");
        uartTerminal_enqueueToUSB((const uint8_t*)"Error, moving to default\n",
                                  strlen("Error, moving to default\n"));
    }
    m_parserState = parserState_default;
}

static void defaultParser(uint8_t byte)
{
    switch (byte)
    {
        break;
        case 'c':
            uartTerminal_enqueueToUSB((const uint8_t*)"Enter 0-3 for close-in atten:\n",
                                      strlen("Enter 0-3 for close-in atten:\n"));
            m_parserState = parserState_receivingCloseIn;
        break;
        case 'i':
            uartTerminal_enqueueToUSB((const uint8_t*)"Setting CC1101 to IDLE state\n",
                                      strlen("Setting CC1101 to IDLE state\n"));
#if COMPILE_RADIO_CC1101
            cc1101_setIdle(true);
#endif // #if COMPILE_RADIO_CC1101
        break;
        case 'o':
            uartTerminal_enqueueToUSB((const uint8_t*)"Setting engine ON idle mode\n",
                                      strlen("Setting engine ON idle mode\n"));
            globalInts_setMachineState(machState_runEngineHydIdle);
        break;
        case 'r':
            uartTerminal_enqueueToUSB((const uint8_t*)"Rebooting\n", strlen("Rebooting\n"));
            // Delay for prints to finish
            nrf_delay_ms(5);
            NVIC_SystemReset();
        break;
        case 't':
            uartTerminal_enqueueToUSB((const uint8_t*)"Enter TX power:\n", strlen("Enter TX power:\n"));
            m_accumulator = 0;
            m_negative = false;
            m_parserState = parserState_receivingPower;
        break;
        case LF_CHAR:
            break;
        case CR_CHAR:
            break;
        default:
            uartTerminal_enqueueToUSB((const uint8_t*)"Enter c, i, o, r, t\n", strlen("Enter i, o, r, t\n"));
        break;
    }
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
#endif // #if (UART_RX_PIN && UART_TX_PIN)
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
    trySend(); // Send any left over from last poll
    uint32_t numRxBytes = 0;
    while (m_rxDataReadIndex != m_rxDataWriteIndex)
    {
        NRF_LOG_INFO("Received 0x%x", m_rxData[m_rxDataReadIndex]);
        switch (m_parserState)
        {
            case parserState_default:
                defaultParser(m_rxData[m_rxDataReadIndex]);
            break;
            case parserState_receivingPower:
                parsePowerdigit(m_rxData[m_rxDataReadIndex]);
            break;
            case parserState_receivingCloseIn:
                parseCloseIn(m_rxData[m_rxDataReadIndex]);
            break;
        }
        m_rxDataReadIndex++;
        numRxBytes++;
    }
    if (numRxBytes)
    {
        NRF_LOG_DEBUG("Received %d rx bytes", numRxBytes);
    }
    trySend(); // Send any enqued this time
}

void uartTerminal_init(void)
{
    if (componentInit())
    {
        m_parserState = parserState_default;
        pollers_registerPoller(uartPoll);
    }
}

#endif // #if COMPILE_RADIO_900T20D

