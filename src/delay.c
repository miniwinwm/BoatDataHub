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
#include "stm32f10x_rcc.h"
#include "delay.h"

void delay_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};	// sets .TIM_ClockDivision = 0U, .TIM_CounterMode = TIM_CounterMode_Up, .TIM_RepetitionCounter = 0U

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 23U;
	TIM_TimeBaseStructure.TIM_Period = UINT16_MAX;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* Enable counter */
	TIM_Cmd(TIM4, ENABLE);
}

void delay_ms(uint16_t ms)
{
	uint16_t i;

	for (i = 0U; i < 305U; i++)
	{
		delay_us(ms);
	}
}

void delay_us(uint32_t us)
{
	TIM4->CNT = 0U;
	while (TIM4->CNT <= us)
	{
	}
}
