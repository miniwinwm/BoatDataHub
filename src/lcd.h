/*
MIT License

Copyright (c) John Blaiklock 2020 Boat Data Hub

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#define LCD_WIDTH_CHARS			16U
#define LCD_QUEUE_CAPACITY		6U
#define LCD_DEGREE_CHARACTER	"\x01"
#define LCD_TURN_CHARACTER		"\x02"
#define LCD_MOVE_CHARACTER		"\x03"
#define LCD_UP_CHARACTER		"\x04"
#define LCD_DOWN_CHARACTER		"\x05"

typedef enum
{
	CLEAR_SCREEN,
	WRITE_TEXT
} lcd_command_t;

typedef struct
{
	lcd_command_t command;
	uint8_t row;
	uint8_t column;
	uint8_t length;
	char text[LCD_WIDTH_CHARS];
} lcd_packet_t;

void lcd_init(void);
void lcd_puts(uint8_t row, uint8_t column, const char *s);
void lcd_clear(void);
void lcd_task(void *parameters);

#endif
