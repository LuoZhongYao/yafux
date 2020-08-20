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

void logging_init(void);
void logging_print(int level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

#define _LOG_NAME(log, suffix) __log_backed_##log##_##suffix
#define LOG_NAME(log, suffix)	_LOG_NAME(log, suffix)
#define SYS_LOG_DEF(log_init, log_print)	\
	static const struct log_module __attribute__((section("log_backend"), used)) LOG_NAME(log_init, __LINE__) = {.init = log_init, .print = log_print}
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
