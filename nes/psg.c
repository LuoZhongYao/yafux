/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */

#include "psg.h"

static uint8_t bit = 10;
static uint8_t prev_write;

int sdl_keystate(unsigned pos);
uint8_t nes_psg_read(uint8_t addr)
{
	if (addr == 0) {
		if (bit++ < 9) {
			/* button state */
			return sdl_keystate(bit);
		}
	}

	return 0;
}

void nes_psg_write(uint8_t addr, uint8_t val)
{
	if (addr == 0) {
		if (!(val & 1) && prev_write == 1) {
			bit = 0;
		}
	}

	prev_write = val & 1;
}
