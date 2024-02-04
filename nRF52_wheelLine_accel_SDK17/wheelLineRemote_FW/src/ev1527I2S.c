/*
 * ev1527I2S.c
 *
 *  Created on: Feb 2, 2024
 *      Author: Collin Moore
 */

#include "sdk_config.h"
#include "ev1527I2S.h"

#include "nrfx_i2s.h"
#include "nrf_gpio.h"
#include "pollers.h"
#include "uptimeCounter.h"
#include "version.h"

#define NRF_LOG_MODULE_NAME ev1527I2S
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/*************************************************************************************
 *  Definitions
 ************************************************************************************/

/* REMOTE HAD smallest pulse of 332-336us
 * 332us period means 3012.048 Hz pulses.
 * 336us period means 2976Hz.
 * SPI can clock at 2, 4, 5, 8, 10, 16, 20, 40, 80MHz
 * 2MHz / 3,012Hz = 664.01 bits per bit of radio. That is so many, let's use a timer.

 * Each remote pulse is 1/5 of a bit time, so bit rate is 3kHz/5 = 595Hz.
 * A data zero is 0 = 10000, and 1 = 11110.
 * Basically, 1/5 of a bit is always on, the next 3 are the value, and the last 5th is 0.
 * So we need to time 5x the bit rate, or 2976 to 3012Hz.
 * Let's go with 3005Hz timer interval. Then ISR decides what to do.
 * ISR will fire 5 times for each bit:
 * First interval will be on.
 * Second, third, fourth will be determined by bit: 1 for 1, 0 for 0.
 * Last 5th will be always off.
 *
 * EV1527 datasheet
 * When we sniffed a long sequence of 1s, it was 1.07us. 335*3=1005us
 *
 * EV1527 says it has 1 time high and 31 low for preamble, then address, then data.
 * Those data bits are as follows:
 * '0' is 1 high and 3 lows, and a '1' is 3 highs and 1 low.
 *
 * So 32 bits of preamble, then 20 address bits, then 4 data bits.
 * Since the preamble is set, we only need to send 20 address bits and 4 data bits.
 *
 * On air, it sends 335us, then stops for 11,230ms, which is 33.5x that first high pulse.
 * So, send for 335us, then stop for 34x that, then send the address bits.
 *
 */

#define SUB_BIT_WIDTH_US (334)
#define SUB_BITS_PER_BIT 4

/* Now that we know about the pulses needed to send RF data, we need to know about I2S
 * I2S clock is ~960,000Hz, so each bit of I2S would last 1.04us.
 * To send EV1527 data, we need pulses to be about ~335us wide.
 * 335/1.04 = 322.115 I2S bits per pulse of EV1527 data.
 *
 */
#define I2S_BITS_PER_SUB_BIT 322

typedef enum
{
    evSend_inactive = 0,
    evSend_sendingPreamble,
    evSend_sendingAddressBits,
    evSend_sendingDataBits,
} ev1527SendState_t;

/*************************************************************************************
 *  Variables
 ************************************************************************************/

#if NRFX_TIMER1_ENABLED
static const nrfx_i2s_t m_i2s = NRFX_I2S_INSTANCE(0);

#endif // #if NRFX_TIMER1_ENABLED

// EV1527, send 20 address and 4 data bits. Send MSBit is start of address, LSBit is D3
static uint32_t g_address,
        g_data;
static volatile uint32_t g_BitIdx, g_bitSubIdx, g_nextSubBitVal, g_numRepeatsLeft;
static volatile ev1527SendState_t g_sendState = evSend_inactive;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static uint32_t calculateNextSubBit(uint32_t bitb0, uint32_t subBitIdx)
{ // Mask off any upper bits, only care about b0
    if (bitb0 & 0x01)
    { // If a 1, send 3 highs then 1 low.
        if (3 == subBitIdx)
        {
            return 0;
        }
        return 1;
    }
    else
    { // If a 0, send 1 high then 3 lows
        if (0 == subBitIdx)
        {
            return 1;
        }
        return 0;
    }
}

static void handleTxTimeout(void)
{
    switch (g_sendState)
    {
        case evSend_inactive:
            // If called when done, turn off and stop timer.
            NRF_P0->OUTCLR = (1UL << RADIO_TX_GPIO);
            // disarm timer, stop.
            nrfx_timer_disable(&m_timer1);
        break;
        case evSend_sendingPreamble:
            if (0 == g_bitSubIdx)
            { // Set high for 1 sub-bit, then low for 34
                NRF_P0->OUTSET = (1UL << RADIO_TX_GPIO);
            }
            else
            {
                NRF_P0->OUTCLR = (1UL << RADIO_TX_GPIO);
            }
            g_bitSubIdx++;
            if (g_bitSubIdx >= 34)
            { // Send 1 sub high, 34 low, then move on to address bits.
                g_sendState = evSend_sendingAddressBits;
                g_bitSubIdx = 0;
                g_BitIdx = 0;
                g_nextSubBitVal = calculateNextSubBit(g_address >> 19, 0);
            }
        break;
        case evSend_sendingAddressBits:
            // Here we are done with the preamble, start sending the address bits.
            if (g_nextSubBitVal)
            {
                NRF_P0->OUTSET = (1UL << RADIO_TX_GPIO);
            }
            else
            {
                NRF_P0->OUTCLR = (1UL << RADIO_TX_GPIO);
            }
            // figure out what the next sub-bit will be
            g_bitSubIdx++;
            if (g_bitSubIdx > 3)
            { // only 4 sub-bit states in address and data bits
                g_BitIdx++;
                g_bitSubIdx = 0;
            }
            if (g_BitIdx > 19)
            { // We are done here, send data bits next
                g_sendState = evSend_sendingDataBits;
                g_bitSubIdx = 0;
                g_BitIdx = 0;
                // Data bits 0-3, only 4 bits
                g_nextSubBitVal = calculateNextSubBit(g_data >> 3, 0);
            }
            else
            {
                g_nextSubBitVal = calculateNextSubBit(g_address >> (19 - g_BitIdx), g_bitSubIdx);
            }
        break;
        case evSend_sendingDataBits:
            // Here we are sending data bits
            if (g_nextSubBitVal)
            {
                NRF_P0->OUTSET = (1UL << RADIO_TX_GPIO);
            }
            else
            {
                NRF_P0->OUTCLR = (1UL << RADIO_TX_GPIO);
            }
            // figure out what the next sub-bit will be
            g_bitSubIdx++;
            if (g_bitSubIdx > 3)
            { // only 4 sub-bit states in address and data bits
                g_BitIdx++;
                g_bitSubIdx = 0;
            }
            if (g_BitIdx > 3)
            { // We are done here, repeat or go to inactive
                if (g_numRepeatsLeft)
                {
                    g_numRepeatsLeft--;
                    g_sendState = evSend_sendingPreamble;
                }
                else
                {
                    g_sendState = evSend_inactive;
                }
                g_bitSubIdx = 0;
                g_BitIdx = 0;
                // Data bits 0-3, only 4 bits
                g_nextSubBitVal = 0;
            }
            else
            {
                g_nextSubBitVal = calculateNextSubBit((g_data >> (3 - g_BitIdx)) & 0x01, g_bitSubIdx);
            }
        break;
    }
}

static void timerISRHandler(nrf_timer_event_t event_type, void* p_context)
{
    NRF_P0->OUTSET = (1UL << ISR_DEBUG_GPIO); // Debug timer
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_WARNING("Timer compare0")
            ;
            handleTxTimeout();
        break;
        default:
            NRF_LOG_WARNING("Timer event %d not handled", event_type)
            ;
        break;
    }
    NRF_P0->OUTCLR = (1UL << ISR_DEBUG_GPIO);
}

static void i2sDataHandler(nrfx_i2s_buffers_t const* p_released, uint32_t status)
{

}

static void setupRadioOutputs(void)
{
    // LEDs are active low, but radio is active high
    NRF_P0->OUTSET = (1UL << ISR_DEBUG_GPIO);
    nrf_gpio_cfg_output(ISR_DEBUG_GPIO);
    NRF_P0->OUTCLR = (1UL << RADIO_TX_GPIO);
    nrf_gpio_cfg_output(RADIO_TX_GPIO);
}

#define TX_ITVL_MS 1500
static uint32_t g_lastTx_ms;

static void poll(void)
{
    if (uptimeCounter_elapsedSince(g_lastTx_ms) >= TX_ITVL_MS)
    {
        if (g_numRepeatsLeft)
        {
            NRF_LOG_WARNING("txPoll, but Still %d repeats left, don't send now", g_numRepeatsLeft);
            g_lastTx_ms = uptimeCounter_getUptimeMs() - (TX_ITVL_MS - 50);
            return;
        }
        /* Our remote to copy sends data bits (Left first, MSBit first):
         * 0b 0010 0000 1101 0111 0100 1110 0
         * EV1527 address is 20 bits, data is 4 bits. Should have 24 bits total.
         * We seem to have 25 bits...
         * Ok, the last bit is the preamble starting over again, which is a 0 then 31 empty spaces, 11.6ms
         * So, the 24 bits we need to send are
         * 0b 0010 0000 1101 0111 0100 1110 = 0x 20 D74E
         * 4 bits are data, 20 address.
         * So set address to 0x20D74, Data to 0xE
         */
        g_address = 0x020D74;
        g_data = (uptimeCounter_getUptimeMs() / 1000) % 16;
        g_sendState = evSend_sendingPreamble;
        g_bitSubIdx = 0;
        g_BitIdx = 0;
        g_numRepeatsLeft = 10;
        // Timer start
//        hw_timer_alarm_us(TIMER_ITVL_US, true);

        uint32_t ticks = nrfx_timer_us_to_ticks(&m_timer1, TIMER_ITVL_US);
        nrfx_timer_compare(&m_timer1, NRF_TIMER_CC_CHANNEL0, ticks, true);
        nrfx_timer_enable(&m_timer1);

        NRF_LOG_INFO("Set nrfx_timer time %dus, %d cycles", TIMER_ITVL_US, g_numRepeatsLeft + 1);
        g_lastTx_ms = uptimeCounter_getUptimeMs();
    }
}

void ev1527I2S_init(void)
{
    setupRadioOutputs();
    // Set up I2S to send data
    nrfx_i2s_uninit();
    nrfx_i2s_config_t config;
    config.alignment = NRF_I2S_ALIGN_LEFT; // Don't care about this
    config.channels = NRF_I2S_CHANNELS_STEREO; // ?
    config.format = NRF_I2S_FORMAT_I2S; // Use I2S mode to have it send every byte of the 32-bit words.
    config.irq_priority = APP_IRQ_PRIORITY_LOW; // Need to service this in time
    config.lrck_pin = I2S_LRCLK_PIN; // Must use a real pin, even though we don't care about this in reality.

    ret_code_t
    ret = nrfx_i2s_init(&config, i2sDataHandler);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("timerInit Error 0x%x", ret);
        return;
    }
    pollers_registerPoller(poll);

}
