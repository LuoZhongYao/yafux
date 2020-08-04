/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */

#include <nes.h>
#include <logging.h>

int main(void)
{
	SYS_LOG_INIT();

	BLOGE("Booting...\n");

	nes_init("super.nes");

	while (1) {
		nes_eval();
	}
	return 0;
}
