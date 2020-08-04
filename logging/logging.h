/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */


#ifndef __LOGGING_H__
#define __LOGGING_H__

#include <stdarg.h>
#include <config.h>

#define LOGGING_ERR	1
#define LOGGING_WRN 2
#define LOGGING_INF	3
#define LOGGING_DBG 4

#ifdef CONFIG_LOGGING

struct log_module {
	void (*init)(void);
	int (*print)(const char *fmt, va_list ap);
};


extern struct log_module __start_log_backend;
extern struct log_module __stop_log_backend;

static inline void logging_init(void)
{
	struct log_module *log;

	for (log = &__start_log_backend; log < &__stop_log_backend; log++) {
		log->init();
	}
}

static inline void logging_print(int level, const char *fmt, ...)
{
	va_list ap;
	struct log_module *log;

	va_start(ap, fmt);

	for (log = &__start_log_backend; log < &__stop_log_backend; log++) {
		log->print(fmt, ap);
	}

	va_end(ap);
}

# define SYS_LOG_DEF(log_init, log_print)	\
	static const struct log_module __attribute__((section("log_backend"))) __log_backend_##__LINE__ = {.init = log_init, .print = log_print}
#define SYS_LOG_INIT() logging_init()
#define SYS_LOG_PRINT(level, fmt, ...) do {	\
	if (level <= CONFIG_LOGGING_LEVEL) {				\
		logging_print(level, fmt, ##__VA_ARGS__);	\
	}	\
} while(0)

#else	/* CONFIG_LOGGING */

# define SYS_LOG_DEF(log_init, log_print)
# define SYS_LOG_INIT()
# define SYS_LOG_PRINT(level, fmt, ...)

#endif

#define BLOGD(fmt, ...) SYS_LOG_PRINT(LOGGING_DBG, fmt, ##__VA_ARGS__)
#define BLOGW(fmt, ...) SYS_LOG_PRINT(LOGGING_WRN, fmt, ##__VA_ARGS__)
#define BLOGE(fmt, ...) SYS_LOG_PRINT(LOGGING_ERR, fmt, ##__VA_ARGS__)
#define BLOGI(fmt, ...) SYS_LOG_PRINT(LOGGING_INF, fmt, ##__VA_ARGS__)

#endif /* __LOGGIG_H__ */
