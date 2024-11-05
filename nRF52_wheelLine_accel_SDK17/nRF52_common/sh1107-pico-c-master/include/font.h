#ifndef SH1107_PICO_C_FONT_H
#define SH1107_PICO_C_FONT_H

//#include "pico.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct font_char {
	uint16_t width;
	uint16_t height;
	const char *buff;
	int offset[2];
} font_char_t;

typedef font_char_t font_t[][128];

enum text_alignment { text_align_left = 0, text_align_center, text_align_right };

font_char_t *font_get_char(font_t *font, uint16_t size, char chr);
bool font_char_get_pixel(font_char_t *font_char, uint16_t row, uint16_t col);
int font_char_get_start_col(font_char_t *font_char, uint16_t col);
int font_char_get_end_col(font_char_t *font_char, uint16_t col);
int font_char_get_start_row(font_char_t *font_char, uint16_t row);
int font_char_get_end_row(font_char_t *font_char, uint16_t row);

#endif // SH1107_PICO_C_FONT_H
