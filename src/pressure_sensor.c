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
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "timer.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "main.h"

#define I2C_PRESSURE_SENSOR_ADDRESS			0x76U

typedef struct
{
	uint8_t register_address;
	uint8_t *coefficient;
} coefficients_table_entry;

static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;
static int32_t t_fine;
static QueueHandle_t pressure_sensor_queue_handle;

static const coefficients_table_entry coefficients_table[] = {{0x88U, (uint8_t *)&dig_T1},
		{0x8aU, (uint8_t *)&dig_T2},
		{0x8cU, (uint8_t *)&dig_T3},
		{0x8eU, (uint8_t *)&dig_P1},
		{0x90U, (uint8_t *)&dig_P2},
		{0x92U, (uint8_t *)&dig_P3},
		{0x94U, (uint8_t *)&dig_P4},
		{0x96U, (uint8_t *)&dig_P5},
		{0x98U, (uint8_t *)&dig_P6},
		{0x9aU, (uint8_t *)&dig_P7},
		{0x9cU, (uint8_t *)&dig_P8},
		{0x9eU, (uint8_t *)&dig_P9}
};

static void bmp280_compensate_T_int32(int32_t adc_T);
static uint32_t bmp280_compensate_P_int64(int32_t adc_P);
static bool i2c_send(uint8_t address, uint8_t reg, uint8_t data);
static bool i2c_receive(uint8_t address, uint8_t reg, uint8_t *read_value);
static bool i2c_receive_multi(uint8_t address, uint8_t reg, uint8_t *read_value, uint8_t length);

static bool i2c_send(uint8_t address, uint8_t reg, uint8_t data)
{
	uint32_t start_time_ms = timer_get_time_ms();

    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
    {
    	if (timer_get_time_ms() > start_time_ms + 100UL)
    	{
    		return false;
    	}
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
    }

    I2C_Send7bitAddress(I2C1, address << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
    }

    I2C_SendData(I2C1, reg);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
    }

    I2C_SendData(I2C1, data);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF))
    {
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
    }

    return true;
}

static bool i2c_receive_multi(uint8_t address, uint8_t reg, uint8_t *read_value, uint8_t length)
{
	uint8_t i;
	bool result = true;

	for (i = 0U; i < length; i++)
	{
		result = result && i2c_receive(address, reg + i, read_value + i);
	}

	return result;
}


static bool i2c_receive(uint8_t address, uint8_t reg, uint8_t *read_value)
{
	uint32_t start_time_ms = timer_get_time_ms();

    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
    {
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
    }

    I2C_Send7bitAddress(I2C1, address << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
    }

    I2C_SendData(I2C1, reg);
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF) == RESET)
    {
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
    }

    I2C_Send7bitAddress(I2C1, address << 1, I2C_Direction_Receiver);
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET)
	{
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
	}

	I2C_AcknowledgeConfig(I2C1, DISABLE);
	__disable_irq();
	(void)I2C1->SR2;

	I2C_GenerateSTOP(I2C1, ENABLE);
	__enable_irq();
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
	{
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
	}

	*read_value = I2C_ReceiveData(I2C1);
	while (I2C1->CR1 & I2C_CR1_STOP)
	{
    	if (timer_get_time_ms() > start_time_ms + 5UL)
    	{
    		return false;
    	}
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	return true;
}

static void bmp280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1;
	int64_t var2;

	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
}

static uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
	int64_t varl;
	int64_t var2;
	int64_t p;

	varl = ((int64_t)t_fine) - 128000LL;
	var2 = varl * varl * (int64_t)dig_P6;
	var2 = var2 + ((varl * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	varl = ((varl * varl * (int64_t)dig_P3) >> 8) + ((varl * (int64_t)dig_P2) << 12);
	varl = (((((int64_t)1LL) << 47) + varl)) * ((int64_t)dig_P1) >> 33;
	if (varl == 0LL)
	{
		return 0UL; // avoid exception caused by division by zero
	}

	p = 1048576LL - (int64_t)adc_P;
	p = (((p << 31) - var2) * 3125LL) / varl;
	varl = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + varl + var2) >> 8) + (((int64_t)dig_P7) << 4);

	return (uint32_t)p;
}

void pressure_sensor_init(void)
{
    I2C_InitTypeDef  I2C_InitStructure = {0};		// sets I2C_Mode = I2C_Mode_I2C, I2C_OwnAddress1 = 0x00U

    // I2C1 configuration
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000UL;
    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &I2C_InitStructure);
}

bool pressure_sensor_start_measurement_mb(void)
{
    return i2c_send(I2C_PRESSURE_SENSOR_ADDRESS, 0xf4U, 0x25U);
}

bool pressure_sensor_get_measurement_mb(float *read_measurement)
{
    int32_t adc_T;
    int32_t adc_P;
    uint8_t msb;
    uint8_t lsb;
    uint8_t xlsb;
    uint32_t P;

    if (!i2c_receive(I2C_PRESSURE_SENSOR_ADDRESS, 0xfaU, &msb))
    {
    	return false;
    }
    if (!i2c_receive(I2C_PRESSURE_SENSOR_ADDRESS, 0xfbU, &lsb))
    {
    	return false;
    }
    if (!i2c_receive(I2C_PRESSURE_SENSOR_ADDRESS, 0xfcU, &xlsb))
    {
    	return false;
    }

    adc_T = (uint32_t)(xlsb >> 4);
    adc_T |= ((uint32_t)lsb) << 4;
    adc_T |= ((uint32_t)msb) << 12;

    bmp280_compensate_T_int32((int32_t)adc_T);

    if (!i2c_receive(I2C_PRESSURE_SENSOR_ADDRESS, 0xf7U, &msb))
	{
		return false;
	}
    if (!i2c_receive(I2C_PRESSURE_SENSOR_ADDRESS, 0xf8U, &lsb))
	{
		return false;
	}
    if (!i2c_receive(I2C_PRESSURE_SENSOR_ADDRESS, 0xf9U, &xlsb))
	{
		return false;
	}

    adc_P = (uint32_t)(xlsb >> 4);
    adc_P |= ((uint32_t)lsb) << 4;
    adc_P |= ((uint32_t)msb) << 12;

    P = bmp280_compensate_P_int64((int32_t)adc_P);
    *read_measurement = (float)P / 25600.0f;

    return true;
}

void pressure_sensor_task(void *parameters)
{
	bool pressure_sensor_init_ok;
	float latest_pressure_reading;
	uint8_t i;

	(void)memcpy(&pressure_sensor_queue_handle, parameters, sizeof(QueueHandle_t));

    // read calibration data from pressure sensor
	pressure_sensor_init_ok = true;
	for (i = 0U; i < sizeof(coefficients_table) / sizeof(coefficients_table_entry); i++)
	{
		pressure_sensor_init_ok = pressure_sensor_init_ok && i2c_receive_multi(I2C_PRESSURE_SENSOR_ADDRESS,
				coefficients_table[i].register_address,
				coefficients_table[i].coefficient,
				2U);
	}

	// signal main task that this task has started
	(void)xTaskNotifyGive(*get_main_task_handle());

    while (true)
    {
		if (pressure_sensor_init_ok && pressure_sensor_start_measurement_mb())
		{
			vTaskDelay((TickType_t)500);

			if (pressure_sensor_get_measurement_mb(&latest_pressure_reading))
			{
				if (latest_pressure_reading > 920UL && latest_pressure_reading < 1080UL)
				{
					(void)xQueueSend(pressure_sensor_queue_handle, (const void *)&latest_pressure_reading, (TickType_t)100);
				}
			}
		}

		vTaskDelay((TickType_t)9500);
    }
}
