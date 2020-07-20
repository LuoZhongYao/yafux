/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */


#ifndef __LOGGING_H__
#define __LOGGING_H__

#define ENABLE_LOGGING
#define ENABLE_LED_LOGGING

#ifdef ENABLE_LOGGING

#include <stdio.h>

extern void logging_init(void);

#define __LOG	printf
#define BLOGD(fmt, ...) __LOG(fmt, ##__VA_ARGS__)
#define BLOGW(fmt, ...) __LOG(fmt, ##__VA_ARGS__)
#define BLOGE(fmt, ...) __LOG(fmt, ##__VA_ARGS__)
#define BLOGV(fmt, ...) __LOG(fmt, ##__VA_ARGS__)
#define BLOGI(fmt, ...) __LOG(fmt, ##__VA_ARGS__)

#else

#define BLOGD(...)
#define BLOGW(...)
#define BLOGE(...)
#define BLOGV(...)
#define BLOGI(...)
#define logging_init()

#endif

#ifdef ENABLE_LED_LOGGING
extern void logging_led_init(void);
extern void logging_led_value(unsigned char value);
#else
#define logging_led_init()
#define logging_value(...)
#endif

#endif /* __LOGGIG_H__*/

