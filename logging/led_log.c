/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 *
 * SPDX-License-Identifier:
 */
#include <logging.h>
#include "stm32f10x.h"

#ifdef CONFIG_LOGGING_LED

static void logging_led_init(void)
{
	/*打开GPIOC的时钟*/
	RCC->APB2ENR |= 0x00000010;

    /* 设置GPIOC的低8位为通用推挽输出 */
    GPIOC->CRL |= 0x33333333;

    /* 初始化为Io口 */
    GPIOC->ODR |= 0xFF00;
}

static int logging_led_print(const char *fmt, va_list ap)
{
    /* 设置LED灯的状态, GPIO一次设置16位，将其值强制转换位16位 */
    GPIOC->BSRR = fmt[0] & 0x00FF;
    GPIOC->BRR =  ~(fmt[0] & 0x00FF);

	return 0;
}

SYS_LOG_DEF(logging_led_init, logging_led_print);

#endif
