/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */


#ifndef __PSG_H__
#define __PSG_H__

#include <stdint.h>

uint8_t nes_psg_read(uint8_t addr);
void nes_psg_write(uint8_t addr, uint8_t val);

#endif /* __PSG_H__*/

