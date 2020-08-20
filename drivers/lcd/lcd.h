/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */


#ifndef __LCD_H__
#define __LCD_H__

#include <stdint.h>

void lcd_init(void);
void lcd_draw(uint16_t *rgb, unsigned size);
void lcd_draw_point(uint16_t x, uint16_t y, uint16_t rgb);
void lcd_fill(uint16_t x, uint16_t y, uint16_t heigth, uint16_t width,uint16_t rgb);
void lcd_set_window(uint16_t x, uint16_t y, uint16_t heigth, uint16_t width);

#endif /* __LCD_H__*/

