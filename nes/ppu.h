/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */


#ifndef __PPU_H__
#define __PPU_H__

#include <stdint.h>
#include "pt-1.4/pt.h"

#define SCANLINE	240
#define	WIDTH		256

struct ppu {
	struct pt pt;
	int scanline, dot;
	uint16_t v,t;

	uint8_t w:1;
	uint8_t x:3;

	uint8_t ctrl;
	uint8_t mask;
	uint8_t status;
	uint8_t oamaddr;
	uint8_t latch;

	uint8_t pale[0x20];
};

extern struct ppu ppu;
uint8_t ppu_io_read(uint16_t addr);
void ppu_io_write(uint16_t addr, uint8_t val);
void ppu_oamdma_write(uint8_t val);

int nes_ppu_eval(void);
void nes_ppu_init(void);

#endif /* __PPU_H__*/

