/*
 * main.c
 *
 *  Created on: Jan 17, 2024
 *      Author: Collin Moore
 */

#include "sdk_config.h"

#include "SEGGER_RTT.h"

#include "nrf_delay.h"

#include <stdint.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

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

static void log_init(void)
{
    // TODO put in a timestamping function
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

int main(void)
{

    log_init();
    NRF_LOG_DEBUG("WheelLineRemote start");
    uint32_t counter = 0;
    for (;;)
    {
        SEGGER_RTT_printf(0, "Hello number %d\n", counter);
        NRF_LOG_ERROR("")
        nrf_delay_ms(1200);
        counter++;
    }
}
