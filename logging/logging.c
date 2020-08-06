/*
 * Written by ZhongYao Luo <luozhongyao@gmail.com>
 *
 * Copyright 2020 ZhongYao Luo
 */

#include <logging.h>

#ifdef CONFIG_LOGGING

extern const struct log_module __start_log_backend[];
extern const struct log_module __stop_log_backend[];

static void dummy_init(void)
{
}

static int dummy_print(const char *fmt, va_list ap)
{
	return 0;
}

SYS_LOG_DEF(dummy_init, dummy_print);

void logging_init(void)
{
	const struct log_module *log;

	for (log = __start_log_backend; log < __stop_log_backend; log++) {
		log->init();
	}
}

void logging_print(int level, const char *fmt, ...)
{
	va_list ap;
	const struct log_module *log;

	va_start(ap, fmt);

	for (log = __start_log_backend; log < __stop_log_backend; log++) {
		log->print(fmt, ap);
	}

	va_end(ap);
}

#endif
