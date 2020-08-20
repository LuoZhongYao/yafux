/*
 * Written by ZhongYao Luo <luozhongyao@gmail.com>
 *
 * Copyright 2020 ZhongYao Luo
 */

#ifndef __MIPI_H__
#define __MIPI_H__

#include <stdint.h>

void mipi_init(void);
void mipi_read(uint16_t reg, uint16_t *data, unsigned size);
void mipi_data_read(uint16_t *data, unsigned size);

void mipi_reg_write(uint16_t reg);
void mipi_data_write(const uint16_t *data, unsigned size);
void mipi_write(uint16_t reg, const uint16_t *data, unsigned size);

#define mipi_auto_write(reg, ...)								\
	do {														\
		const uint16_t _buf[] = {__VA_ARGS__};					\
		mipi_write(reg, _buf, sizeof(_buf) / sizeof(uint16_t));	\
	} while (0)


#endif /* __MIPI_H__*/

