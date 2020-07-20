/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */

#include "rom.h"
#include "mapper.h"
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <string.h>

#define MAPPER_NUMBER(hdr)	((((hdr).flag6 & 0xf0) >> 4) | ((hdr).flag7 & 0xf0))

struct mapper *nes_rom_load(const char *file)
{
	int chr, prg;
	struct mapper *mapper;
	struct ines_hdr hdr;
	int fd = open(file,O_RDONLY);

	if (fd < 0) {
		perror(file);
		exit(1);
	}

	read(fd, &hdr, sizeof(hdr));
	if (memcmp(hdr.nes, (uint8_t[]){'N', 'E', 'S', 0x1a}, 4)) {
		close(fd);
		return NULL;
	}

	chr = hdr.chr ?: 1;
	prg = hdr.prg ?: 1;

	printf("PRG ROM: %d x 16KiB\n", prg);
	printf("CHG ROM: %d x  8KiB\n", chr);
	printf("MAPPER : %d\n", MAPPER_NUMBER(hdr));
	printf("Mirroring: %s, PRG RAM: %s, 4 Screen: %s\n",
		(hdr.flag6 & 1) ? "vertical" : "horizontal",
		(hdr.flag6 & 2) ? "True" : "False",
		(hdr.flag6 & 8) ? "True" : "False");

	switch (MAPPER_NUMBER(hdr)) {
	default: mapper = mapper0_instance(); break;
	case 3: mapper = mapper3_instance(); break;
	}

	memcpy(&mapper->hdr, &hdr, sizeof(hdr));
	mapper->chr = malloc(0x2000 * chr);
	mapper->prg = malloc(0x4000 * prg);

	if (mapper->hdr.flag6 & 0x04) {
		lseek(fd, 16 + 512, SEEK_SET);
	}

	if (mapper->hdr.flag6 & 8) {
		mapper->nt = malloc(0x400 * 4);
	} else {
		mapper->nt = malloc(0x400 * 2);
	}

	read(fd, mapper->prg, prg * 0x4000);
	read(fd, mapper->chr, chr * 0x2000);
	
	close(fd);
	return mapper;
}

void nes_rom_unload(struct mapper *mapper)
{
	free(mapper->prg);
	free(mapper->chr);
}
