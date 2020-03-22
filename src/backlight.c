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

#include "stm32f10x.h"
#include "backlight.h"

static TIM_OCInitTypeDef TIM_OCInitStructure = {0};
static const uint16_t levels[BACKLIGHT_MAX + 1] = {0U, 20U, 200U, 500U};

void backlight_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0}; // sets TIM_CounterMode = TIM_CounterMode_Up, TIM_ClockDivision = 0U

	TIM_TimeBaseStructure.TIM_Prescaler = 1U;
	TIM_TimeBaseStructure.TIM_Period = 500U;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	//TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 250U;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void backlight_set(uint8_t level)
{
	if (level > BACKLIGHT_MAX)
	{
		return;
	}

	TIM_OCInitStructure.TIM_Pulse = levels[level];
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
}
