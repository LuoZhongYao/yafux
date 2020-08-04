/*
 * Written by ZhongYao Luo <luozhongyao@gmail.com>
 *
 * Copyright 2020 ZhongYao Luo
 */

#include <stdio.h>
#include <logging.h>

#ifdef CONFIG_LOGGING_SEMIHOSTING

static void semihosting_log_init(void)
{
	;
}

SYS_LOG_DEF(semihosting_log_init, vprintf);

#endif
