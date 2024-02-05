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
 * So 32 sub-bits of preamble, then 20 address bits(4 sub-bits each), then 4 data bits (4 sub-bits each).
 * Since the preamble is set, we only need to send 20 address bits and 4 data bits.
 *
 * So, sub-bits are: 32 + 20*4 + 4*4 = 32+96 = 128 sub-bits per packet
 *
 * On air, it sends 335us, then stops for 11,230ms, which is 33.5x that first high pulse.
 * So, send for 335us, then stop for 34x that, then send the address bits.
 *
 * inter-packet delay of TODO TUNE is ok for momentary relays.
 *
 */
#define EV1527_SUB_BIT_WIDTH_US (334)
#define EV1527_SUB_BITS_PER_PACKET 128

/* Now that we know about the pulses needed to send RF data, we need to know about I2S
 * I2S clock is ~960,000Hz, so each bit of I2S would last 1.04us.
 * To send EV1527 data, we need pulses to be about ~335us wide.
 * 335/1.04 = 322.115 I2S bits per pulse of EV1527 data.
 *
 */
#define I2S_BITS_PER_SUB_BIT 8 // TODO tune

#define I2S_DATA_BLOCK_WORDS (EV1527_SUB_BITS_PER_PACKET*I2S_BITS_PER_SUB_BIT) // tune to be the size of an EV1527 packet.

/*************************************************************************************
 *  Variables
 ************************************************************************************/

#if NRFX_TIMER1_ENABLED
static const nrfx_i2s_t m_i2s = NRFX_I2S_INSTANCE(0);

#endif // #if NRFX_TIMER1_ENABLED

// EV1527, send 20 address and 4 data bits. Send MSBit is start of address, LSBit is D3

// I2S variables
__ALIGNED(4) static uint32_t g_i2sRxBuffers[2][I2S_DATA_BLOCK_WORDS]; // Rx can't be NULL
__ALIGNED(4) static uint32_t g_i2sTxBuffer0[I2S_DATA_BLOCK_WORDS];
__ALIGNED(4) static uint32_t g_i2sTxBuffer1[I2S_DATA_BLOCK_WORDS]; // DON'T USE, just for zeros
static volatile bool g_i2sTxbuffer0Free;

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

static void i2sDataHandler(nrfx_i2s_buffers_t const* p_released, uint32_t status)
{
    NRF_P0->OUTSET = (1UL << ISR_DEBUG_GPIO); // Debug isr
    NRF_LOG_INFO("ISR");
    if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    { // Shouldn't happen
        g_i2sTxbuffer0Free = true;
        NRF_LOG_WARNING("No next needed, must have stopped");
        NRF_P0->OUTCLR = (1UL << ISR_DEBUG_GPIO);
        return;
    }
    /* First call to this handler occurs right after the transfer is started.
     * No data has been transferred yet (at this call), so there is nothing to check.
     * Only the buffers for the next part of the transfer should be provided.
     */
    if (!p_released->p_rx_buffer)
    {
        nrfx_i2s_buffers_t const next_buffers = {
                                                  .p_rx_buffer = g_i2sRxBuffers[1],
                                                  .p_tx_buffer = g_i2sTxBuffer1
        };
        ret_code_t ret = nrfx_i2s_next_buffers_set(&next_buffers);
//        if (NRF_SUCCESS != ret)
//        {
        NRF_LOG_DEBUG("nrfx_i2s_next_buffers_set error 0x%x", ret);
//            return;
//        }
    }
    else
    { // It has released a buffer:
        g_i2sTxbuffer0Free = true;
        if (g_i2sTxBuffer0 == p_released->p_tx_buffer)
        { // If it released buffer 0, it is done with the transfer of that packet. Stop the I2S
            nrfx_i2s_stop();
            NRF_LOG_INFO("0 was freed, stopped");
        }
        else if (g_i2sTxBuffer1 == p_released->p_tx_buffer)
        { // If it released 1, 1 is now free (probably when stop completes)
            NRF_LOG_INFO("1 was freed");
        }
        else
        {
            NRF_LOG_ERROR("Ping-pong ptr was wrong!");
        }
    }
    NRF_P0->OUTCLR = (1UL << ISR_DEBUG_GPIO);
}

static void setupRadioOutputs(void)
{
    // LEDs are active low, but radio is active high
    NRF_P0->OUTSET = (1UL << ISR_DEBUG_GPIO);
    nrf_gpio_cfg_output(ISR_DEBUG_GPIO);
    NRF_P0->OUTCLR = (1UL << RADIO_TX_GPIO);
    nrf_gpio_cfg_output(RADIO_TX_GPIO);
}

static ret_code_t startTransfer(uint32_t address, uint8_t _4DataBits)
{
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
    if (!g_i2sTxbuffer0Free)
    {
        NRF_LOG_ERROR("Can't send, still in progress sending a transfer");
        return NRF_ERROR_BUSY;
    }
    // If here, buffer is free. Fill with desired bits.

    // Default to zeros (not transmitting)
    memset(g_i2sTxBuffer0, 0, sizeof(g_i2sTxBuffer0));
    uint32_t i2sBitIndex = 0;
    g_i2sTxBuffer0[0] = 0x80000000; // 1 high, 31 lows for preamble
    i2sBitIndex = 32;
    // TODO remove and fix the below
    for (uint32_t i = 1; i < I2S_DATA_BLOCK_WORDS; i++)
    {
        g_i2sTxBuffer0[i] = 0xFF00FF00;
    }
    goto starti2s;

    // TODO fix the below
    // 20 address bits, 4 sub-bits per bit so 80 bits for address, 3.5 words
    for (uint32_t i = 0; i < 20; i++)
    {
        // Put in 4 sub-bits for each address bit, MSBit at b31, LSBit at b0
        if ((address >> (i - 19)) & 0x01)
        { // Set a 1: sub-bits are 0b1110
          // Words on I2S are send MSBit first
//            g_i2sTxBuffer0[1 + ((80 - (i * 4)) / 32)] |= (0b1110 << ((76 - (i * 4)) % 32));
//            g_i2sTxBuffer0[i2sBitIndex/32] |= ?
        }
        else
        { // Set a zero: 0b1000
//            g_i2sTxBuffer0[1 + ((80 - (i * 4)) / 32)] |= (0b1000 << ((76 - (i * 4)) % 32));
        }
        i2sBitIndex += 4;
    }
    // i2sBitIndex should be at 32+80=112. Start inputting data bits
    for (uint32_t i = 0; i < 4; i++)
    {
        if ((_4DataBits >> i) & 0x01)
        { // Set a 1: sub-bits are 0b1110
          // Words on I2S are send MSBit first
//            g_i2sTxBuffer0[i2sBitIndex / 32] |= (0b1110 << ((76 - (i * 4)) % 32));
        }
        else
        { // Set a zero: 0b1000
//            g_i2sTxBuffer0[i2sBitIndex / 32] |= (0b1000 << ((76 - (i * 4)) % 32));
        }
    }

    starti2s:
    {
        nrfx_i2s_buffers_t const initial_buffers = {
                                                     .p_tx_buffer = g_i2sTxBuffer0,
                                                     .p_rx_buffer = g_i2sRxBuffers[0],
        };
        ret_code_t ret = nrfx_i2s_start(&initial_buffers, I2S_DATA_BLOCK_WORDS, 0);
        // Start the TX!
        NRF_LOG_INFO("Started I2S send,err 0x%x", ret);
        return ret;
    }
    return NRF_SUCCESS;
}

#define TX_ITVL_MS 1500
static uint32_t g_lastTx_ms;

static void poll(void)
{
    if (uptimeCounter_elapsedSince(g_lastTx_ms) >= TX_ITVL_MS)
    {
        if (!g_i2sTxbuffer0Free)
        {
            NRF_LOG_WARNING("txPoll, sending a transmission, wait");
            g_lastTx_ms = uptimeCounter_getUptimeMs();
            return;
        }
        ret_code_t ret = startTransfer(0x020D74, (uptimeCounter_getUptimeMs() / 1000) % 16);
        if (NRF_SUCCESS != ret)
        {
            NRF_LOG_ERROR("Tx start error 0x%x", ret);
        }
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
    config.mck_pin = NRFX_I2S_PIN_NOT_USED; // Optional
    config.mck_setup = NRF_I2S_MCK_32MDIV11; // ?? TODO tune
    config.mode = NRF_I2S_MODE_MASTER; // We control clock
    config.ratio = NRF_I2S_RATIO_96X; // 96x at MCK 32M/11 for SCK of 969,696.96Hz, TODO change to 343us
    config.sample_width = NRF_I2S_SWIDTH_16BIT; // TODO tune
    config.sck_pin = I2S_SCK_PIN;
    config.sdin_pin = NRFX_I2S_PIN_NOT_USED;
    config.sdout_pin = I2S_SDOUT_PIN; // Start with this unused if we need it to idle high

    ret_code_t ret = nrfx_i2s_init(&config, i2sDataHandler);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("nrfx_i2s_init Error 0x%x", ret);
        return;
    }
    // Immediately after I2S init, de-mux the pins we don't actually use, to avoid spurious radiation.
    // NOTE: we can NOT re-use the pins for other functions. TODO, test this hypothesis
//    nrf_gpio_cfg_default(I2S_SCK_PIN);
//    nrf_gpio_cfg_default(I2S_LRCLK_PIN);
    // fill second buffer with all zeros, so it sends nothing
    memset(g_i2sTxBuffer1, 0, sizeof(g_i2sTxBuffer1));
    g_i2sTxbuffer0Free = true;
    NRF_LOG_INFO("Init success");
    pollers_registerPoller(poll);

}
