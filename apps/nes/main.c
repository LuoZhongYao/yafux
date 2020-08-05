/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */

#include "lcd.h"
#include <nes.h>
#include "stm32f10x.h"
#include <logging.h>

extern const unsigned char raw_rom[];
int sdl_keystate(unsigned pos)
{
	return 0;
}

void sdl_render(uint32_t *pixes, unsigned count)
{
}

void delay(unsigned ms)
{
	uint32_t tmp;

	SysTick->LOAD = ms * 72 * 1000;
	SysTick->VAL = 0x00;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

	do {
		tmp = SysTick->CTRL;
	} while ((tmp & 1) && !(tmp & (1 << 16)));

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL = 0x00;
}

int main(void)
{
	SYS_LOG_INIT();

	BLOGD("Booting...\n");

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

	lcd_init();
	nes_init(raw_rom);

	while (1) {
		nes_eval();
	}

	return 0;
}
