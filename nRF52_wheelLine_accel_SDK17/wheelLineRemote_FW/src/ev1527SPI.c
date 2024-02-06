/*
 * ev1527SPI.c
 *
 *  Created on: Feb 2, 2024
 *      Author: Collin Moore
 */

#include "sdk_config.h"
#include "ev1527SPI.h"

#if NRFX_SPI_ENABLED

#include "nrfx_spi.h"
#include "nrf_gpio.h"
#include "pollers.h"
#include "uptimeCounter.h"
#include "version.h"

#define NRF_LOG_MODULE_NAME ev1527SPI
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
#define EV1527_SUB_BIT_WIDTH_US  (334)
#define EV15278_PREAMBLE_SUB_BITS  32
#define EV1527_ADDRESS_BITS        20
#define EV1527_DATA_BITS           4
#define EV1527_SUB_BITS_PER_PACKET 128

/*
 * Tuned based on SPI clock frequency:
 * 125kHz SPI clock, 332us pulse would be 3012.12Hz. 336us would be 2976.19Hz.
 * 125,000/3012.12 = 41.499 bits per pulse
 * 125,000/2976.19 = 42.00 bits per pulse
 * Looks like 41 bits may work, 42 should work!
 * Tested at 42 bits, got 330us pulse. Switch to 43.
 * Tested at 43 bits, got 340us pulse. TODO test and optimize
 * Tested at 42 bits, got 330us pulse, but actual RF was only active for 309us
 * Tested at 43 bits, RF active for 321us.
 * Tested at 45 bits, RF active for 340us, just like remote of 339us!
 */

#define SPI_BITS_PER_SUB_BIT 45

#define SPI_TX_BYTES ((EV1527_SUB_BITS_PER_PACKET*SPI_BITS_PER_SUB_BIT)/8) // tune to be the size of an EV1527 packet.

/* How long to wait between sending, to re-send. Need to keep alive
 * 1 seems to work, not longer.
 */
#define TX_REPEAT_MS 1 // How often to keep alive the receiver

#define TX_TEST_ITVL_MS 1
#define TX_TEST_NUM_REPEATS 50

uint32_t g_iterator;

/*
 * For our 6-channel remote:
 * 0 does nothing
 * 1 does nothing
 * 2 does ch 4
 * 3 does ch 4
 * 4 does ch 2
 * 5 does ch 2
 * 6 does ch 6
 * 7 does ch 6
 * 8 does ch 1
 * 9 does ch 1
 * A does ch 5
 * B does ch 5
 * C does ch 3
 * D does ch 3
 * E does nothing
 * F does nothing
 */

/*
 * For our 8-channel remote:
 * 0 does nothing
 * 1 does nothing
 * 2 does ch 5
 * 3 does ch 5
 * 4 does ch 3
 * 5 does ch 3
 * 6 does ch 7
 * 7 does ch 7
 * 8 does ch 4
 * 9 does ch 4
 * A does nothing
 * B does nothing
 * C does ch 2
 * D does ch 2
 * E does ch 1
 * F does ch 1
 */

/*************************************************************************************
 *  Variables
 ************************************************************************************/

#if NRFX_SPI2_ENABLED
static const nrfx_spi_t m_spi = NRFX_SPI_INSTANCE(2);
#else
#error "Please define or enable an SPI instance"
#endif // #if NRFX_SPI2_ENABLED

// EV1527, send 20 address and 4 data bits. Send MSBit is start of address, LSBit is D3

// SPI variables
__ALIGNED(4) static uint8_t m_spiTxBytes[SPI_TX_BYTES];
static nrfx_spi_xfer_desc_t m_xfer;
static volatile bool m_spiXferDone;
static uint32_t m_spiTransfesRemaining;

static uint32_t g_lastTx_ms;

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

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

static void setupRadioOutputs(void)
{
// LEDs are active low, but radio is active high
    NRF_P0->OUTCLR = (1UL << RADIO_TX_GPIO);
    nrf_gpio_cfg_output(RADIO_TX_GPIO);
}
static uint32_t setBits(uint32_t numBits, uint8_t* pArray, uint32_t bitIndex)
{
    if (!numBits)
    {
        return 0;
    }
//    NRF_LOG_DEBUG("Setting bits %d to %d", bitIndex, bitIndex + numBits);
    uint32_t bitsSet = 0;
    uint32_t bitsSetThisLoop;
    uint8_t setMask;
    // Set the bits in the first byte, starting at bitIndex, if it's not a byte multiple
    while (numBits)
    { // Set bits, starting at MSBit (left-most) of desired bits to set
        setMask = 0xFF; // Set a mask for the whole byte, shift left, then right to clear bits then OR in.
        bitsSetThisLoop = 8; // Could be 8 total set each byte
        // Find out how many bits in this byte to set. We can set a max of 8 bits
        if (numBits < 8)
        { // Last iteration may not have 8 bits to set. Shift right down to that number of bits, then back.
          // Shift right so we only have this many bits to set
            setMask >>= (8 - numBits);
            // Shift back left again, to fill in with zeros
            setMask <<= (8 - numBits);
            bitsSetThisLoop -= (8 - numBits);
        }
        if (bitIndex % 8)
        { // First iteration may not be byte-aligned, so only set the bits in that byte
          // Find out the index in the byte to start setting bits, in from the left
            uint8_t setStartIndex = (bitIndex % 8);
            // shift left, consider lowest/first/MSBit as bit b7.
            setMask >>= setStartIndex;
            bitsSetThisLoop -= setStartIndex;
        }
        // OR in bits to set in this byte
        pArray[bitIndex / 8] |= setMask; // Set the bits in the byte desired, to the desired bits
        bitIndex += bitsSetThisLoop;
        bitsSet += bitsSetThisLoop; // Add the number of bits we set.
        numBits -= bitsSetThisLoop;
    }
    return bitsSet;
}

static ret_code_t startTransfer(void)
{
    if (!m_spiXferDone)
    {
        NRF_LOG_ERROR("Can't send, still in progress sending a transfer");
        return NRF_ERROR_BUSY;
    }
    // Start the data Tx
    m_spiXferDone = false;
    ret_code_t ret = nrfx_spi_xfer(&m_spi, &m_xfer, 0);
    switch (ret)
    {
        case NRF_SUCCESS:
            //            NRF_LOG_INFO("EF1527 TX start")
            //            ;
        break;
        case NRFX_ERROR_NOT_SUPPORTED:
            m_spiXferDone = true; // Failed to start
            NRF_LOG_ERROR("nrfx_spi_xfer error 0x%x", ret)
            ;
        break;
        default:
            NRF_LOG_ERROR("nrfx_spi_xfer error 0x%x", ret)
            ;
            m_spiXferDone = true;
        break;
    }
    return ret;
}

static ret_code_t writeTxBuffer(uint32_t address, uint8_t _4DataBits)
{
    if (!m_spiXferDone)
    {
        NRF_LOG_ERROR("Can't write TX buffer, TX still in progress");
        return NRF_ERROR_BUSY;
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

    // If here, SPI is ready. Fill with desired bits and start transfer
    NRF_LOG_INFO("Writing addr 0x%x, data 0x%x", address, _4DataBits);
    // Default to zeros (not transmitting)
    memset(m_spiTxBytes, 0, sizeof(m_spiTxBytes));
    // Set the preamble: 1 sub-bit high, 31 sub-bits low
    uint32_t spiBitIndex = 0;
    spiBitIndex += setBits(SPI_BITS_PER_SUB_BIT, m_spiTxBytes, spiBitIndex);
    // We have now set one sub-bit, now we need 31 low sub-bits
    spiBitIndex += (31 * SPI_BITS_PER_SUB_BIT);
    // Add 2 more to space out longer
//    spiBitIndex += (EXTRA_PREAMBLE_ZEROS * SPI_BITS_PER_SUB_BIT);

    // 20 address bits, 4 sub-bits per bit, then * by spi bits per sub-bit
//    address &= 0x000FFFFF;
    for (uint32_t i = 0; i < EV1527_ADDRESS_BITS; i++)
    { // Put in 4 sub-bits for each address bit, MSBit at b31, LSBit at b0
        if ((address >> ((EV1527_ADDRESS_BITS - 1) - i)) & 0x01)
        { // Set a 1: sub-bits are 0b1110
            spiBitIndex += setBits(3 * SPI_BITS_PER_SUB_BIT, m_spiTxBytes, spiBitIndex);
            spiBitIndex += (1 * SPI_BITS_PER_SUB_BIT);
        }
        else
        { // Set a zero: 0b1000
            spiBitIndex += setBits(SPI_BITS_PER_SUB_BIT, m_spiTxBytes, spiBitIndex);
            spiBitIndex += (3 * SPI_BITS_PER_SUB_BIT);
        }
    }
// spiBitIndex should be at 32+80=112. Start inputting data bits
//    _4DataBits &= 0x0F;
    for (uint32_t i = 0; i < EV1527_DATA_BITS; i++)
    { // Put in 4 sub-bits for each data bit, MSBit at b31, LSBit at b0
        if ((_4DataBits >> ((EV1527_DATA_BITS - 1) - i)) & 0x01)
        { // Set a 1: sub-bits are 0b1110
            spiBitIndex += setBits(3 * SPI_BITS_PER_SUB_BIT, m_spiTxBytes, spiBitIndex);
            spiBitIndex += (1 * SPI_BITS_PER_SUB_BIT);
            NRF_LOG_DEBUG("D%d bit=1", i);
        }
        else
        { // Set a zero: 0b1000
            spiBitIndex += setBits(SPI_BITS_PER_SUB_BIT, m_spiTxBytes, spiBitIndex);
            spiBitIndex += (3 * SPI_BITS_PER_SUB_BIT);
            NRF_LOG_DEBUG("D%d bit=0", i);
        }

    }
    return NRF_SUCCESS;
}

static void poll(void)
{
    if (!m_spiXferDone)
    {
        g_lastTx_ms = uptimeCounter_getUptimeMs(); // Still going, update
        return;
    }
    // If here, Transfer is done
    if (m_spiTransfesRemaining)
    {
        if (m_spiXferDone && uptimeCounter_elapsedSince(g_lastTx_ms) >= TX_REPEAT_MS)
        {
            startTransfer();
            m_spiTransfesRemaining--;
        }
        return;
    }
    // If here, we are done looping, write a new set to loop
    if (uptimeCounter_elapsedSince(g_lastTx_ms) >= TX_TEST_ITVL_MS)
    {
        ret_code_t ret = writeTxBuffer(0x020D74, (uint8_t)(g_iterator++));
        if (g_iterator > 0xF)
        {
            g_iterator = 0;
        }
        if (NRF_SUCCESS != ret)
        {
            NRF_LOG_ERROR("Tx write error 0x%x", ret);
        }
        else
        {
            m_spiTransfesRemaining = TX_TEST_NUM_REPEATS; // Loop this many times
        }
        g_lastTx_ms = uptimeCounter_getUptimeMs();
    }
}

void ev1527SPI_init(void)
{
    setupRadioOutputs();
// Set up I2S to send data
//    nrfx_spi_uninit(&m_spi);
    nrfx_spi_config_t config;
    config.bit_order = NRF_SPI_BIT_ORDER_MSB_FIRST; // TODO test and tune
    config.frequency = NRF_SPI_FREQ_125K;
    config.irq_priority = APP_IRQ_PRIORITY_MID;
    config.miso_pin = NRFX_SPI_PIN_NOT_USED; // unused
    config.mode = NRF_SPI_MODE_1; // Mode 1 makes MOSI idle low, which is what we need for radio
    config.mosi_pin = RADIO_TX_GPIO; // Route to radio TX pin
    config.orc = 0x00; // Send zeros, don't turn on radio if we don't know what we are doing
    config.sck_pin = SPI_SCK_PIN; // TODO try and not use, but nrfx driver needs it
    config.ss_pin = NRFX_SPI_PIN_NOT_USED;
    ret_code_t ret = nrfx_spi_init(&m_spi, &config, spiIsrHandler, NULL);
    if (NRF_SUCCESS != ret)
    {
        NRF_LOG_ERROR("nrfx_spi_init Error 0x%x", ret);
        return;
    }
// Un-mux the SCK pin to lower radiation
    nrf_gpio_cfg_default(SPI_SCK_PIN);
    // Put strong drive on MOSI pin
    nrf_gpio_cfg(
    RADIO_TX_GPIO,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_DISCONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_H0H1,
                 NRF_GPIO_PIN_NOSENSE);
    memset(m_spiTxBytes, 0, sizeof(m_spiTxBytes));
    m_spiXferDone = true;

    m_xfer.p_rx_buffer = NULL;
    m_xfer.p_tx_buffer = m_spiTxBytes;
    m_xfer.rx_length = 0;
    m_xfer.tx_length = SPI_TX_BYTES;

    NRF_LOG_INFO("Init success");
    pollers_registerPoller(poll);
    g_lastTx_ms = uptimeCounter_getUptimeMs();

}
#endif // #if NRFX_SPI_ENABLED
