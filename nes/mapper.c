/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */
#include "mapper.h"
#include <stdio.h>
#include <stddef.h>

#ifndef container_of
# define container_of(ptr, type, member) ({ \
	const typeof( ((type *)0)->member ) *__mptr = (ptr); \
	(type *)( (char *)__mptr - offsetof(type,member) );})
#endif

struct mapper3 {
	uint8_t chr_bank;
	struct mapper mapper;
};

static union {
	struct mapper generic;
	struct mapper3 mapper3;
} mapper_instance;

static uint8_t mapper_generic_prg_read(struct mapper *mapper, uint16_t addr)
{
	addr &= 0x7fff;
	if (mapper->hdr->prg < 2) {
		addr &= 0x3fff;
	}

	return mapper->prg[addr];
}

/*
static void mapper_generic_prg_write(struct mapper *mapper, uint16_t addr, uint8_t val)
{
	addr &= 0x7fff;
	if (mapper->hdr->prg < 2) {
		addr &= 0x3fff;
	}

	mapper->prg[addr] = val;
}

*/

static uint8_t mapper_generic_nt_read(struct mapper *mapper, uint16_t addr)
{
	addr &= 0xfff;
	if (!(mapper->hdr->flag6 & 0x08)) {
		if (mapper->hdr->flag6 & 1) {
			addr = (addr & 0x7ff);
		} else {
			addr = (addr & 0x3ff) | ((addr & 0x800) >> 1);
		}
	}

	return mapper->nt[addr];
}

static void mapper_generic_nt_write(struct mapper *mapper, uint16_t addr, uint8_t val)
{
	addr &= 0xfff;
	if (!(mapper->hdr->flag6 & 0x08)) {
		if (mapper->hdr->flag6 & 1) {
			addr = (addr & 0x7ff);
		} else {
			addr = (addr & 0x3ff) | ((addr & 0x800) >> 1);
		}
	}

	mapper->nt[addr] = val;
}

static uint16_t mapper_generic_chr_read(struct mapper *mapper, uint16_t addr)
{
	return mapper->chr[addr & 0x1fff] | mapper->chr[(addr + 8) & 0x1fff] << 8;
}

/*
static void mapper_generic_chr_write(struct mapper *mapper, uint16_t addr, uint8_t val)
{
	mapper->chr[addr & 0x1fff] = val;
}
*/

static void mapper3_prg_write(struct mapper *mapper, uint16_t addr, uint8_t val)
{
	struct mapper3 *mapper3 = container_of(mapper, struct mapper3, mapper);
	printf("MAPPER3 PRG Write: %04x, %02x\n", addr, val);
	if ((val & 3) < mapper->hdr->chr)
		mapper3->chr_bank = val & 3;
}

static uint16_t mapper3_chr_read(struct mapper *mapper, uint16_t addr)
{
	struct mapper3 *mapper3 = container_of(mapper, struct mapper3, mapper);
	uint16_t page = mapper3->chr_bank * 0x2000;

	return mapper->chr[page + (addr & 0x1fff)] | mapper->chr[page + ((addr + 8) & 0x1fff)] << 8;
}

static const struct mapper_ops mapper_generic_ops = {
	.prg_read = mapper_generic_prg_read,
	.chr_read = mapper_generic_chr_read,
	.nt_write = mapper_generic_nt_write,
	.nt_read = mapper_generic_nt_read,
};

static const struct mapper_ops mapper3_ops = {
	.prg_read = mapper_generic_prg_read,
	.prg_write = mapper3_prg_write,
	.chr_read = mapper3_chr_read,
	.nt_write = mapper_generic_nt_write,
	.nt_read = mapper_generic_nt_read,
};

struct mapper *mapper0_instance(void)
{
	struct mapper *generic = &mapper_instance.generic;

	generic->ops = &mapper_generic_ops;

	return generic;
}

struct mapper *mapper3_instance(void)
{
	struct mapper3 *mapper3 = &mapper_instance.mapper3;

	mapper3->chr_bank = 0;
	mapper3->mapper.ops = &mapper3_ops;

	return &mapper3->mapper;
}
