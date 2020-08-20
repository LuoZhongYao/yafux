/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */

#include <lcd.h>
#include <nes.h>
#include "stm32f10x.h"
#include <logging.h>

extern const unsigned char raw_rom[];
volatile uint32_t jiffies;

void SysTick_Handler(void)
{
	jiffies++;
}

int sdl_keystate(unsigned pos)
{
	return 0;
}

void sdl_render(uint32_t *pixes, unsigned count)
{
}

void delay(unsigned ms)
{
	ms += jiffies;
	while (ms > jiffies);
}

int main(void)
{
	SYS_LOG_INIT();
	BLOGD("Booting...\n");

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_Config(72000);

	lcd_init();
	nes_init(raw_rom);
	BLOGD("jiffies = %d\n", jiffies);

	while (1) {
		nes_eval();
	}

	return 0;
}
