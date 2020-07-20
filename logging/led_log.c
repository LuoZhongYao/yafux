/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */
#include <logging.h>
#include "stm32f10x.h"

#ifdef ENABLE_LED_LOGGING

void logging_led_init(void)
{
	/*打开GPIOC的时钟*/
	RCC->APB2ENR |= 0x00000010;

    /* 设置GPIOC的低8位为通用推挽输出 */
    GPIOC->CRL |= 0x33333333;
    
    /* 初始化为Io口 */
    GPIOC->ODR |= 0xFF00; 
}

void logging_led_value(unsigned char value)
{    
    /* 设置LED灯的状态, GPIO一次设置16位，将其值强制转换位16位 */
    GPIOC->BSRR = (unsigned short)value & 0x00FF;
    GPIOC->BRR =  ~((unsigned short)value & 0x00FF);                  
}

#endif
