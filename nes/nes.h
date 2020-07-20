/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */


#ifndef __NES_H__
#define __NES_H__

#include "cpu.h"
#include "ppu.h"
#include "rom.h"
#include "psg.h"
#include "mapper.h"

void nes_eval(void);
void nes_init(const char *rom);

extern struct mapper *mapper;

#endif /* __NES_H__*/

