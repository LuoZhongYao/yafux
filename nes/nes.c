/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */

#include "nes.h"
#include <stddef.h>
struct mapper *mapper = NULL;
static int cpu_cycle;
static int ppu_cycle;

void nes_init(const uint8_t *rom)
{
	mapper = nes_rom_load(rom);
	if (mapper == NULL)
		return;

	nes_cpu_init();
	nes_ppu_init();
}

void nes_eval(void)
{
	if (--ppu_cycle <= 0) {
	  ppu_cycle = nes_ppu_eval();
	}

	if (--cpu_cycle <= 0) {
	  cpu_cycle = nes_cpu_eval() * 3;
	}
}

