/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */

#include <logging.h>

int main(void)
{
	SYS_LOG_INIT();

	BLOGE("\nBooting...\n");

	while (1);
	return 0;
}
