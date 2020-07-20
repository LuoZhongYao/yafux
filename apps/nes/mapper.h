/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */


#ifndef __MAPPER_H__
#define __MAPPER_H__

#include "rom.h"

struct mapper;
struct mapper_ops {
	uint8_t (*prg_read)(struct mapper *mapper, uint16_t addr);
	void (*prg_write)(struct mapper *mapper, uint16_t addr, uint8_t val);

	uint16_t (*chr_read)(struct mapper *mapper, uint16_t addr);
	void (*chr_write)(struct mapper *mapper, uint16_t addr, uint8_t val);

	uint8_t (*nt_read)(struct mapper *mapper, uint16_t addr);
	void (*nt_write)(struct mapper *mapper, uint16_t addr, uint8_t val);
};

struct mapper {
	uint8_t *nt;
	uint8_t *prg;
	uint8_t *chr;
	struct ines_hdr hdr;
	const struct mapper_ops *ops;
};

static inline void mapper_prg_write(struct mapper *mapper, uint16_t addr, uint8_t val)
{
	if (mapper->ops && mapper->ops->prg_write) {
		mapper->ops->prg_write(mapper, addr, val);
	}
}

static inline uint8_t mapper_prg_read(struct mapper *mapper, uint16_t addr)
{
	if (mapper->ops && mapper->ops->prg_read) {
		return mapper->ops->prg_read(mapper, addr);
	}

	return 0;
}

static inline void mapper_chr_write(struct mapper *mapper, uint16_t addr, uint8_t val)
{
	if (mapper->ops && mapper->ops->chr_write) {
		mapper->ops->chr_write(mapper, addr, val);
	}
}

static inline uint16_t mapper_chr_read(struct mapper *mapper, uint16_t addr)
{
	if (mapper->ops && mapper->ops->chr_read) {
		return mapper->ops->chr_read(mapper, addr);
	}

	return 0;
}

static inline void mapper_nt_write(struct mapper *mapper, uint16_t addr, uint8_t val)
{
	if (mapper->ops && mapper->ops->nt_write) {
		mapper->ops->nt_write(mapper, addr, val);
	}
}

static inline uint8_t mapper_nt_read(struct mapper *mapper, uint16_t addr)
{
	if (mapper->ops && mapper->ops->nt_read) {
		return mapper->ops->nt_read(mapper, addr);
	}

	return 0;
}

struct mapper *mapper0_instance(void);
struct mapper *mapper3_instance(void);

#endif /* __MAPPER_H__*/

