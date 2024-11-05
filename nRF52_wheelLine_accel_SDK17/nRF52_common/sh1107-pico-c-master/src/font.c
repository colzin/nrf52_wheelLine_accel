#include "font.h"

#include <stdio.h>
#include <stdlib.h>

#define NRF_LOG_MODULE_NAME font
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
/*************************************************************************************
 *  Definitions
 ************************************************************************************/

#define NTH_BIT(val, n) ((val) & (1 << (n)))

/*************************************************************************************
 *  Variables
 ************************************************************************************/

/*************************************************************************************
 *  Prototypes
 ************************************************************************************/

/*************************************************************************************
 *  Functions
 ************************************************************************************/

static uint8_t hex_to_uint(char hex)
{
    if (hex >= 'a' && hex <= 'f')
    {
        return hex - 'a' + 10;
    }
    else if (hex >= '0' && hex <= '9')
    {
        return hex - '0';
    }
    NRF_LOG_ERROR("unsupported hex char\n");
    return 0;
}

bool font_char_get_pixel(font_char_t* font_char, uint16_t row, uint16_t col)
{
    if (*font_char->buff == '\0')
        return false;
    uint16_t idx = row * font_char->width + col;
    uint8_t nibble = hex_to_uint(font_char->buff[idx / 4]);
    return NTH_BIT(nibble, idx % 4);
}

int font_char_get_start_col(font_char_t* font_char, uint16_t col)
{
    return (int)col + (int)font_char->offset[0];
}

int font_char_get_end_col(font_char_t* font_char, uint16_t col)
{
    return (int)font_char_get_start_col(font_char, col) + (int)font_char->width;
}

int font_char_get_start_row(font_char_t* font_char, uint16_t row)
{
    return (int)row + font_char->offset[1];
}

int font_char_get_end_row(font_char_t* font_char, uint16_t row)
{
    return font_char_get_start_row(font_char, row) + (int)font_char->height;
}

font_char_t* font_get_char(font_t* font, uint16_t size, char chr)
{
    return &(*font)[size / 8][chr];
}
