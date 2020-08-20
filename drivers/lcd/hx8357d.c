/*
 * Written by ZhongYao Luo <luozhongyao@gmail.com>
 *
 * Copyright 2020 ZhongYao Luo
 */
#include <lcd.h>
#include <mipi.h>
#include <logging.h>

/* Color definitions */
#define	HX8357_BLACK   0x0000
#define	HX8357_BLUE    0x001F
#define	HX8357_RED     0xF800
#define	HX8357_GREEN   0x07E0
#define HX8357_CYAN    0x07FF
#define HX8357_MAGENTA 0xF81F
#define HX8357_YELLOW  0xFFE0
#define HX8357_WHITE   0xFFFF

#define HX8357_REG_SWRESET		0x01
#define HX8357_REG_RDDIDIF		0x04
#define HX8357_REG_RDICID		0xd0
#define HX8357_REG_SET_IMG		0xe9
#define HX8357_REG_SLEEP_OUT	0x11
#define HX8357_REG_DISPLAY_OFF	0x28
#define HX8357_REG_DISPLAY_ON	0x29
#define HX8357_REG_CASET		0x2a
#define HX8357_REG_PASET		0x2b
#define HX8357_REG_RAMWR		0x2c
#define HX8357_REG_IFACE_PIXEL_FORMAT	0x3a
#	define DBI_12BIT	0x03
#	define DBI_16BIT	0x05
#	define DBI_18BIT	0x06
#	define DBI_24BIT	0x07
#	define DPI_16BIT	0x50
#	define DPI_18BIT	0x60
#	define DPI_24BIT	0x70

#define HX8357_REG_MEM_CTL	0x36
#	define HX8357_MEM_MY	0x80
#	define HX8357_MEM_MX	0x40
#	define HX8357_MEM_MV	0x20
#	define HX8357_MEM_ML	0x10
#	define HX8357_MEM_BGR	0x08
#	define HX8357_MEM_MH	0x04

#define HX8357_REG_SETOSC	0xb0
#define HX8357_REG_SETPWR1	0xb1
#define HX8357_REG_SETRGB	0xb3

#define HX8357_REG_SETSTBA	0xc0
#define HX8357_REG_SETDGC	0xc1
#define HX8357_REG_SETDDB	0xc4
#define HX8357_REG_GAMMASET	0xc8
#define HX8357_REG_SETCABC	0xc9
#define HX8357_REG_SETPANEL	0xcc


void delay(unsigned ms);

void lcd_init(void)
{
	uint16_t val[2];

	mipi_init();

	delay(50);
	mipi_auto_write(HX8357_REG_SWRESET);
	delay(50);

	mipi_read(HX8357_REG_RDICID, val, 2);
	BLOGD("Chip ID: %x\n", val[1]);

	mipi_auto_write(HX8357_REG_SETRGB, 0x00, 0x00, 0x06, 0x06);
	mipi_auto_write(HX8357_REG_SETOSC, 0x68, 0x01);
	mipi_auto_write(HX8357_REG_SETPANEL, 0x05);

	mipi_auto_write(HX8357_REG_SET_IMG, 0x20);

	mipi_auto_write(HX8357_REG_IFACE_PIXEL_FORMAT, DPI_16BIT | DBI_16BIT);
	mipi_auto_write(HX8357_REG_MEM_CTL, HX8357_MEM_MX | HX8357_MEM_BGR);
	mipi_auto_write(HX8357_REG_SETDGC, 0x10, 0x10, 0x02, 0x02);
	mipi_auto_write(HX8357_REG_SETSTBA, 0x00, 0x35, 0x00, 0x00, 0x01, 0x02);
	mipi_auto_write(HX8357_REG_SETDDB, 0x03);
	mipi_auto_write(HX8357_REG_SETCABC, 0x01);
	mipi_auto_write(HX8357_REG_GAMMASET,
		0x02, 0x0A, 0x11, 0x1d, 0x23, 0x35,
		0x41, 0x4b, 0x4b, 0x42, 0x3A, 0x27,
		0x1B, 0x08, 0x09, 0x03, 0x02, 0x0A,
		0x11, 0x1d, 0x23, 0x35, 0x41, 0x4b,
		0x4b, 0x42, 0x3A, 0x27, 0x1B, 0x08,
		0x09, 0x03, 0x00, 0x01);
	mipi_auto_write(HX8357_REG_SLEEP_OUT);
	delay(50);
	mipi_auto_write(HX8357_REG_DISPLAY_ON);
	lcd_fill(0, 0, 480, 320, ~HX8357_BLACK);
}

static void lcd_set_rect(uint16_t x, uint16_t y, uint16_t heigth, uint16_t width)
{
	uint16_t xl = x & 0xff;
	uint16_t xh = x >> 8;
	uint16_t wl = (x + width - 1) & 0xff;
	uint16_t wh = (x + width - 1) >> 8;
	uint16_t yl = y & 0xff;
	uint16_t yh = y >> 8;
	uint16_t hl = (y + heigth - 1) & 0xff;
	uint16_t hh = (y + heigth - 1) >> 8;

	mipi_auto_write(HX8357_REG_CASET, xh, xl, wh, wl);
	mipi_auto_write(HX8357_REG_PASET, yh, yl, hh, hl);
}

void lcd_set_window(uint16_t x, uint16_t y, uint16_t heigth, uint16_t width)
{
	lcd_set_rect(x, y, heigth, width);
	mipi_auto_write(HX8357_REG_RAMWR);
}

void lcd_draw(uint16_t *rgb, unsigned size)
{
	mipi_data_write(rgb, size);
}

void lcd_draw_point(uint16_t x, uint16_t y, uint16_t rgb)
{
	uint16_t xl = x & 0xff;
	uint16_t xh = x >> 8;
	uint16_t yl = y & 0xff;
	uint16_t yh = y >> 8;

	lcd_set_rect(x, y, 1, 1);
	mipi_auto_write(HX8357_REG_RAMWR, rgb);
}

void lcd_fill(uint16_t x, uint16_t y, uint16_t heigth, uint16_t width,uint16_t rgb)
{
	unsigned i;

	lcd_set_rect(x, y, heigth, width);
	mipi_auto_write(HX8357_REG_RAMWR);

	for (i = 0; i < (heigth * width); i++) {
		mipi_data_write(&rgb, 1);
	}
}
