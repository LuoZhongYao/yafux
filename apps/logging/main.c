/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */

#include <logging.h>

int main(void)
{
	logging_init();
	logging_led_init();

	logging_led_value(0);
	BLOGD("Booting...\n");
	logging_led_value(1);

	_write(0, "Booting...\n", 11);

	while (1);
	return 0;
}
