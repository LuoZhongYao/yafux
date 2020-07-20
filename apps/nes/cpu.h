/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */


#ifndef __CPU_H__
#define __CPU_H__

#include <stdint.h>

int nes_cpu_eval(void);
void nes_cpu_init(void);
void nes_cpu_nmi(void);
uint8_t nes_get_val8(uint32_t addr);

#endif /* __CPU_H__*/

