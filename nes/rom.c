/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */

#include "rom.h"
#include "mapper.h"
#include <logging.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#define MAPPER_NUMBER(hdr)	((((hdr)->flag6 & 0xf0) >> 4) | ((hdr)->flag7 & 0xf0))

struct mapper *nes_rom_load(const uint8_t *raw)
{
	int chr, prg;
	const uint8_t *ptr;
	struct mapper *mapper;
	const struct ines_hdr *hdr = (const struct ines_hdr *)raw;

	if (memcmp(hdr->nes, (uint8_t[]){'N', 'E', 'S', 0x1a}, 4)) {
		BLOGE("Invalid ROM format: %02x %02x %02x %02x",
			hdr->nes[0], hdr->nes[1], hdr->nes[2], hdr->nes[3]);
		return NULL;
	}

	chr = hdr->chr ?: 1;
	prg = hdr->prg ?: 1;

	BLOGD("PRG ROM: %d x 16KiB\n", prg);
	BLOGD("CHG ROM: %d x  8KiB\n", chr);
	BLOGD("MAPPER : %d\n", MAPPER_NUMBER(hdr));
	BLOGD("Mirroring: %s, PRG RAM: %s, 4 Screen: %s\n",
		(hdr->flag6 & 1) ? "vertical" : "horizontal",
		(hdr->flag6 & 2) ? "True" : "False",
		(hdr->flag6 & 8) ? "True" : "False");

	switch (MAPPER_NUMBER(hdr)) {
	default: mapper = mapper0_instance(); break;
	case 3: mapper = mapper3_instance(); break;
	}

	mapper->hdr = hdr;
	ptr = raw + sizeof(struct ines_hdr);
	if (hdr->flag6 & 0x04) {
		ptr = raw + 16 + 512;
	}

	mapper->prg = ptr;
	ptr += prg * 0x4000;
	mapper->chr = ptr;
	return mapper;
}

void nes_rom_unload(struct mapper *mapper)
{
}
