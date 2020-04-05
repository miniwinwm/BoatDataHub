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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f10x.h"
#include "lcd.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "main.h"

#define E_PORT					GPIOB
#define E_PIN					GPIO_Pin_0
#define	RS_PORT					GPIOA
#define RS_PIN					GPIO_Pin_8
#define RW_PORT					GPIOA
#define RW_PIN					GPIO_Pin_7
#define D7_PORT					GPIOC
#define D7_PIN					GPIO_Pin_13
#define D6_PORT					GPIOA
#define D6_PIN					GPIO_Pin_0
#define D5_PORT					GPIOA
#define D5_PIN					GPIO_Pin_1
#define D4_PORT					GPIOA
#define D4_PIN					GPIO_Pin_2

#define CG_CHARACTER_SIZE_BYTES	8U

static const uint8_t custom_characters[] = {0x04, 0x0a, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
												0x0e, 0x06, 0x0a, 0x10, 0x10, 0x11, 0x0e, 0x00,
												0x04, 0x0e, 0x1f, 0x04, 0x04, 0x1f, 0x0e, 0x04,
												0x04, 0x0e, 0x1f, 0x04, 0x04, 0x04, 0x04, 0x04,
												0x04, 0x04, 0x04, 0x04, 0x04, 0x1f, 0x0e, 0x04};

static QueueHandle_t lcd_queue_handle;

static void lcd_wait_until_not_busy();
static void lcd_write_command(uint8_t c);
static void lcd_write_data(uint8_t c);
static void lcd_wait_until_not_busy();
static void lcd_strobe(void);
static void lcd_set_cg_character(uint8_t position, const uint8_t *bytes);
static void lcd_write_nibble(uint8_t c);

static void lcd_strobe(void)
{
	// E = ENABLED;
	GPIO_SetBits(E_PORT, E_PIN);

	// E = DISABLED;
	GPIO_ResetBits(E_PORT, E_PIN);
}

static void lcd_wait_until_not_busy()
{
	volatile uint32_t i;

	for (i = 0UL; i < 1400UL; i++)
	{
		__asm__("NOP");
	}
}

void lcd_set_cg_character(uint8_t position, const uint8_t *bytes)
{
	uint8_t row;
	uint8_t cg_address;

	cg_address = position * 8U + 0x40;
	lcd_write_command(cg_address);

	for (row = 0U; row < 8; row++)
	{
		lcd_write_data(bytes[row]);
	}
}

void lcd_clear(void)
{
	lcd_packet_t lcd_packet;

	lcd_packet.command = CLEAR_SCREEN;
	(void)xQueueSend(lcd_queue_handle, (const void *)&lcd_packet, (TickType_t)0);
}

void lcd_puts(uint8_t row, uint8_t column, const char *s)
{
	lcd_packet_t lcd_packet;
	size_t length = strlen(s);

	lcd_packet.command = WRITE_TEXT;
	lcd_packet.length = (uint8_t)length;
	lcd_packet.row = row;
	lcd_packet.column = column;
	(void)memcpy(lcd_packet.text, s, length);
	(void)xQueueSend(lcd_queue_handle, (const void *)&lcd_packet, (TickType_t)0);
}

void lcd_task(void *parameters)
{
	uint8_t i;
	lcd_packet_t lcd_packet;

	(void)memcpy(&lcd_queue_handle, parameters, sizeof(QueueHandle_t));

	// signal main task that this task has started
	(void)xTaskNotifyGive(*get_main_task_handle());

	while (true)
	{
		(void)xQueueReceive(lcd_queue_handle, &lcd_packet, portMAX_DELAY);

		switch (lcd_packet.command)
		{
		case CLEAR_SCREEN:
			lcd_write_command(0x01U);
			break;

		case WRITE_TEXT:
			if (lcd_packet.row == 0U)
			{
				lcd_write_command(0x80U + lcd_packet.column);
			}
			else
			{
		        lcd_write_command(0xc0U + lcd_packet.column);
		    }

		    for (i = 0U; i < lcd_packet.length; i++)
		    {
		        lcd_write_data(lcd_packet.text[i]);
		    }
		    break;
		}
	}
}

static void lcd_write_nibble(uint8_t c)
{
	GPIO_WriteBit(D4_PORT, D4_PIN, (BitAction)(c & 0x01U));
	GPIO_WriteBit(D5_PORT, D5_PIN, (BitAction)((c >> 1) & 0x01U));
	GPIO_WriteBit(D6_PORT, D6_PIN, (BitAction)((c >> 2) & 0x01U));
	GPIO_WriteBit(D7_PORT, D7_PIN, (BitAction)((c >> 3) & 0x01U));
}

static void lcd_write_command(uint8_t c)
{
	lcd_wait_until_not_busy();

	// RS = COMMAND
	GPIO_ResetBits(RS_PORT, RS_PIN);

	lcd_write_nibble(c >> 4);
    lcd_strobe();
	lcd_write_nibble(c);
    lcd_strobe();
}

static void lcd_write_data(uint8_t c)
{
	// wait until previous operation is finished
	lcd_wait_until_not_busy();

	// RS = DATA
	GPIO_SetBits(RS_PORT, RS_PIN);

	lcd_write_nibble(c >> 4);
    lcd_strobe();
	lcd_write_nibble(c);
    lcd_strobe();
}

void lcd_init(void)
{
	uint8_t i;

	// pins E_PORT:E_PIN, RS_PORT:RS_PIN and RW_PORT:RW_PIN all default to zero on reset so don't need setting here

    delay_ms(15U);
    lcd_write_nibble(0x03U);
    lcd_strobe();
    delay_ms(5U);
    lcd_strobe();
	delay_us(200U);
    lcd_strobe();
 	delay_us(200U);
	lcd_write_nibble(0x02U);
    lcd_strobe();
	lcd_write_command(0x28U);
    delay_us(40U);
    lcd_write_command(0x06U);
    lcd_write_command(0x0cU);

    for (i = 0U; i < sizeof(custom_characters) / CG_CHARACTER_SIZE_BYTES; i++)
    {
    	lcd_set_cg_character(i + 1U, &(custom_characters[i * CG_CHARACTER_SIZE_BYTES]));
    }

	// clear screen
	lcd_write_command(0x01U);
}
