/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */


#ifndef __ROM_H__
#define __ROM_H__

#include <stdint.h>

struct nesrom;
struct ines_hdr {
	uint8_t nes[4];
	uint8_t prg;
	uint8_t chr;
	uint8_t flag6;
	uint8_t flag7;
	uint8_t flag8;
	uint8_t flag9;
	uint8_t flag10;
	uint8_t unused[5];
};

struct mapper *nes_rom_load(const char *file);
void nes_rom_unload(struct mapper *mapper);

#endif /* __ROM_H__*/

