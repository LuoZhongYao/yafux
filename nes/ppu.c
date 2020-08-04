/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 *
 * SPDX-License-Identifier:
 */

#include "nes.h"
#include <string.h>
#include <stdbool.h>
#include <logging.h>

struct ppu ppu;
void sdl_render(uint32_t *, unsigned);
void sdl_dbg_render(uint32_t *, unsigned, unsigned);

#define HBLANK	85

#define V_FLIP	0x80
#define H_FLIP	0x40
#define SP_BACK	0x20

#define SPR_ATTR_HIT_POS		0
#define SPR_ATTR_OVERFLOW_POS	1
#define SPR_ATTR_BEHIND_POS		2
#define SPR_ATTR_HIT		(1 << SPR_ATTR_HIT_POS)
#define SPR_ATTR_OVERFLOW	(1 << SPR_ATTR_OVERFLOW_POS)
#define SPR_ATTR_BEHIND		(1 << SPR_ATTR_BEHIND_POS)

#define PPU_INTERNAL_REG_W			0x01

#define ST_VBLANK		0x80
#define ST_SPR0_HIT		0x40
#define ST_SPR_OVERFLOW	0x20

#define MASK_SHOW_BACKGROUND	0x08
#define MASK_SHOW_SPR			0x10

#define PPU_IS_SHOW_BACKGROUND()	(ppu.mask & MASK_SHOW_BACKGROUND)
#define PPU_IS_SHOW_SPR()	(ppu.mask & MASK_SHOW_SPR)

struct sprite {
	uint8_t y, no, attr, x;
};

static union {
	uint8_t oamdata[256];
	struct sprite spr[64];
} OAM;

static uint8_t spr_count = 0;
static struct sprite *secondary_spr[8];

static const uint32_t palette[64] = {
	0x757575, 0x271B8F, 0x0000AB, 0x47009F, 0x8F0077, 0xAB0013, 0xA70000, 0x7F0B00,
	0x432F00, 0x004700, 0x005100, 0x003F17, 0x1B3F5F, 0x000000, 0x000000, 0x000000,
	0xBCBCBC, 0x0073EF, 0x233BEF, 0x8300F3, 0xBF00BF, 0xE7005B, 0xDB2B00, 0xCB4F0F,
	0x8B7300, 0x009700, 0x00AB00, 0x00933B, 0x00838B, 0x000000, 0x000000, 0x000000,
	0xFFFFFF, 0x3FBFFF, 0x5F97FF, 0xA78BFD, 0xF77BFF, 0xFF77B7, 0xFF7763, 0xFF9B3B,
	0xF3BF3F, 0x83D313, 0x4FDF4B, 0x58F898, 0x00EBDB, 0x000000, 0x000000, 0x000000,
	0xFFFFFF, 0xABE7FF, 0xC7D7FF, 0xD7CBFF, 0xFFC7FF, 0xFFC7DB, 0xFFBFB3, 0xFFDBAB,
	0xFFE7A3, 0xE3FFA3, 0xABF3BF, 0xB3FFCF, 0x9FFFF3, 0x000000, 0x000000, 0x000000,
};

static inline uint8_t ppu_fetch_sprite(int scanline, int pixel, uint8_t *attr)
{
	uint8_t low = 0;
	uint8_t idx = 0;
	int vofs, spr_count = 0;
	uint8_t H = 8;

	*attr = 0;
	vofs = (ppu.ctrl & 0x08 & (((ppu.ctrl & 0x20) ^ 0x20) >> 2)) << 9;
	H += (ppu.ctrl & 0x20) >> 2;

	for (int i = 63; i >= 0; i--) {
		int vadr;
		uint16_t C;
		struct sprite *spr = OAM.spr + i;
		uint8_t sx = spr->x, sy = spr->y + 1;
		int oy, ox, po;

		if ((uint32_t)(scanline - sy) >= H)
			continue;

		if ((uint32_t)(pixel - sx) >= 8)
			continue;

		*attr |= ((!i) << SPR_ATTR_HIT_POS);
		*attr |= (!!(++spr_count >= 8)) << SPR_ATTR_OVERFLOW_POS;
		*attr |= (!!(spr->attr & SP_BACK)) << SPR_ATTR_BEHIND_POS;

		ox = pixel - sx;
		oy = scanline - sy;
		po = pixel & 7;

		if (ppu.ctrl & 0x20) {
			vadr = ((spr->no & 1) << 12) + ((spr->no & 0xfe) << 4);
		} else  {
			vadr = (spr->no << 4) + vofs;
		}

		if (spr->attr & V_FLIP) {
			vadr += 7;
			vadr -= oy;
			vadr += (ppu.ctrl & 0x20) >> 1;
			vadr -= oy & 8;
		} else {
			vadr += oy;
			vadr += oy & 8;
		}

		C = mapper_chr_read(mapper, vadr);

		if (spr->attr & H_FLIP) {
			low = ((C >> po) & 1) | ((C >> (7 + po)) & 2);
		} else {
			low = ((C >> (7 - po)) & 1) | ((C >> (14 - po)) & 2);
		}

		idx = (spr->attr & 0x3 << 2) | low;
	}

	return low ? idx : 0;
}

static inline void ppu_fetch_nametable(uint32_t *pixels, int nt, int scanline, int dot)
{
	int wt = dot >> 3, vt = scanline >> 3;
	uint8_t name = mapper_nt_read(mapper, 0x2000 + nt * 0x400 + vt * 32 + wt);
	uint8_t attr = mapper_nt_read(mapper, 0x2000 + nt * 0x400 + ((vt >> 2) << 3) + (wt >> 2) + 0x3c0);
	uint16_t bgrd = (name << 4) + (scanline & 7) + ((ppu.ctrl & 0x10) ? 0x1000 : 0);
	uint16_t C = mapper_chr_read(mapper, bgrd);

	if (vt & 2)
		attr >>= 4;
	if (wt & 2)
		attr >>= 2;
	attr = (attr & 3) << 2;

	for (int no = 0; no < 8; no++) {
		uint8_t low = ((C >> (7 - no)) & 1) | ((C >> (14 - no)) & 2);
		*pixels++ = palette[ppu.pale[low ? (attr | low) : 0]];
	}
}

static void fetch_sprite(void)
{
	uint8_t H = 8 + ((ppu.ctrl & 0x20) >> 2);

	spr_count = 0;
	memset((void*)secondary_spr, 0, sizeof(secondary_spr));
	for (int i = 0; i < 64; i++) {
		struct sprite *spr = OAM.spr + i;
		if ((uint16_t)(ppu.scanline - spr->y) >= H)
			continue;

		secondary_spr[spr_count++] = spr;
		if (spr_count >= 8)
			break;
	}
}

static void render_scanline_sprite(uint32_t *pixels)
{
	int vofs;
	int vadr;
	uint16_t C;
	uint32_t bgrc = palette[ppu.pale[0]];

	vofs = (ppu.ctrl & 0x08 & (((ppu.ctrl & 0x20) ^ 0x20) >> 2)) << 9;

	while (spr_count--) {
		struct sprite *spr = secondary_spr[spr_count];
		int oy = ppu.scanline - spr->y;
		uint8_t low, high = (spr->attr & 3) << 2;

		if (ppu.ctrl & 0x20) {
			vadr = ((spr->no & 1) << 12) + ((spr->no & 0xfe) << 4);
		} else  {
			vadr = (spr->no << 4) + vofs;
		}

		if (spr->attr & V_FLIP) {
			vadr += 7;
			vadr -= oy;
			vadr += (ppu.ctrl & 0x20) >> 1;
			vadr -= oy & 8;
		} else {
			vadr += oy;
			vadr += oy & 8;
		}

		C = mapper_chr_read(mapper, vadr);
		for (int no = 0; no < 8; no++) {
			if (spr->attr & H_FLIP) {
				low = ((C >> no) & 1) | ((C >> (7 + no)) & 2);
			} else {
				low = ((C >> (7 - no)) & 1) | ((C >> (14 - no)) & 2);
			}

			if (low == 0)
				continue;

			if (spr == OAM.spr && !(ppu.status & ST_SPR0_HIT) && pixels[spr->x + no] != bgrc) {
				ppu.status |= ST_SPR0_HIT;
			}

			if (!(spr->attr & SP_BACK)) {
				pixels[spr->x + no] = palette[ppu.pale[(high | low) + 0x10]];
			}
		}
	}
}

static void render_scanline_tile(uint32_t *pixels, uint8_t off, uint8_t size)
{
	uint8_t name = mapper_nt_read(mapper, 0x2000 | (ppu.v & 0x0fff));
	uint8_t attr = mapper_nt_read(mapper, 0x23C0 | (ppu.v & 0x0C00) | ((ppu.v >> 4) & 0x38) | ((ppu.v >> 2) & 0x07));
	uint16_t bgrd = (name << 4) + (ppu.scanline & 7) + ((ppu.ctrl & 0x10) ? 0x1000 : 0);
	uint16_t C = mapper_chr_read(mapper, bgrd);

	if (ppu.v & 0x40)
		attr >>= 4;
	if (ppu.v & 2)
		attr >>= 2;
	attr = (attr & 3) << 2;

	for (; off < size; off++) {
		uint8_t low = ((C >> (7 - off)) & 1) | ((C >> (14 - off)) & 2);
		*pixels++ = palette[ppu.pale[low ? (attr | low) : 0]];
	}
}

static void ppu_vram_put(uint16_t addr, uint8_t val)
{
	addr &= 0x3fff;

	if (addr < 0x2000) {
		mapper_chr_write(mapper, addr, val);
	} else if (addr < 0x3f00) {
		mapper_nt_write(mapper, addr & 0xfff, val);
	} else if(0x3f00 <= addr) {
		switch (addr & 0x1f) {
		case 0x00:
		case 0x10:
			ppu.pale[0x00] = ppu.pale[0x10] = val;
		break;
		case 0x04:
		case 0x14:
			ppu.pale[0x04] = ppu.pale[0x14] = val;
		break;
		case 0x08:
		case 0x18:
			ppu.pale[0x08] = ppu.pale[0x18] = val;
		break;
		case 0x0c:
		case 0x1c:
			ppu.pale[0x0c] = ppu.pale[0x1c] = val;
		break;
		default:
			ppu.pale[addr & 0x1f] = val;
		break;
		}
	}
}

static uint8_t ppu_vram_get(uint16_t addr)
{
	uint8_t val = 0xff;

	addr &= 0x3fff;

	if (addr < 0x2000) {
		val = mapper_chr_read(mapper, addr) & 0xff;
	} else if (addr < 0x3f00) {
		val = mapper_nt_read(mapper, addr & 0xfff);
	} else if(addr < 0x4000) {
		val = ppu.pale[addr & 0x1f];
	}

	return val;
}

uint8_t ppu_io_read(uint16_t addr)
{
	uint8_t res = 0xff;
	switch (addr & 0x7) {
	case 2: {
		res = ppu.status;
		ppu.status &= ~(ST_SPR0_HIT | ST_VBLANK);
		ppu.w = 0;
	} break;
	case 4: ppu.latch = res = OAM.oamdata[ppu.oamaddr]; break;
	case 7: {
		res = ppu_vram_get(ppu.v);
		ppu.latch = (ppu.v < 0x3f00) ? res : 0;
		if (!ppu.w) {
			ppu.w = 1;
		} else  {
			ppu.v += (ppu.ctrl & 0x04) ? 32 : 1;
		}
	} break;
	}

	return res;
}

void ppu_io_write(uint16_t addr, uint8_t val)
{
	switch (addr & 7) {
	case 0:
		ppu.ctrl = val;
		ppu.t = (ppu.t & ~(3 << 10)) | ((val & 3) << 10);
	break;
	case 1: ppu.mask = val; break;
	case 2: ppu.status = val; break;
	case 3: ppu.oamaddr = val; break;
	case 4: OAM.oamdata[ppu.oamaddr++] = val; break;
	case 5: {
		if (!ppu.w) {
			ppu.t = (ppu.t & ~0x1f) | (val >> 3);
			ppu.x = val & 7;
		} else {
			ppu.t = (ppu.t & ~0x73e0) | (val & 7) << 12 | (((val >> 3) & 0x1f) << 5);
		}

		ppu.w ^= 1;
	} break;

	case 6: {
		if (!ppu.w) {
			ppu.t = (val & 0x3f) << 8;
		} else {
			ppu.t |= val;
			ppu.v = ppu.t;
		}
		ppu.w ^= 1;
	} break;

	case 7: {
		ppu_vram_put(ppu.v, val);
		ppu.v += (ppu.ctrl & 0x04) ? 32 : 1;
	} break;
	}
}

void ppu_oamdma_write(uint8_t val)
{
	int i;
	for (i = 0; i < 256; i++) {
		OAM.oamdata[i] = nes_get_val8((val << 8) + i);
	}
}

static void ppu_coarse_x_inc(void)
{
	if ((ppu.v & 0x001f) == 31) {
		ppu.v &= ~0x001f;
		ppu.v ^= 0x0400;
	} else {
		ppu.v++;
	}
}

static void ppu_y_inc(void)
{
	int y;
	if ((ppu.v & 0x7000) != 0x7000) {
		ppu.v += 0x1000;
	} else {
		ppu.v &= ~0x7000;
		y = (ppu.v & 0x03e0) >> 5;
		if (y == 29) {
			y = 0;
			ppu.v ^= 0x0800;
		} else if (y == 31) {
			y = 0;
		} else {
			y += 1;
		}
		ppu.v = (ppu.v & ~0x3e0) | (y << 5);
	}
}

// static void nes_ppu_nametable(void)
// {
// 	static uint32_t dbg_pixels[4][WIDTH * SCANLINE];
// 	for (int nt = 0; nt < 4; nt++) {
// 		for (int y = 0; y < SCANLINE; y++) {
// 			for (int x = 0; x < WIDTH; x += 8) {
// 				ppu_fetch_nametable(dbg_pixels[nt] + y * WIDTH + x, nt, y, x);
// 			}
// 		}
// 		sdl_dbg_render(dbg_pixels[nt], WIDTH, nt);
// 	}
// }

static PT_THREAD(ppu_working(int *cycle))
{
	uint32_t color;
	static uint32_t pixels[1 + 0 * WIDTH * SCANLINE];
	static uint32_t *pix;
	PT_BEGIN(&ppu.pt);

#define RETURN(n) do {*cycle = (n); PT_YIELD(&ppu.pt);} while (0)

	while (1) {
		ppu.status &= ~(ST_SPR_OVERFLOW | ST_VBLANK | ST_SPR0_HIT);
		/* pre-render scanline */
		RETURN(280);
		if (PPU_IS_SHOW_BACKGROUND())
			ppu.v = ppu.t;

		for (ppu.scanline = 0; ppu.scanline < SCANLINE; ppu.scanline++) {

			fetch_sprite();

			pix = pixels + (ppu.scanline << 8);
			if (PPU_IS_SHOW_BACKGROUND()) {
				render_scanline_tile(pix, ppu.x, 8 - ppu.x);
				ppu_coarse_x_inc();
			}
			RETURN(8 - ppu.x);

			for (ppu.dot = 8 - ppu.x; ppu.dot < (WIDTH - ppu.x); ppu.dot+=8) {
				if (PPU_IS_SHOW_BACKGROUND()) {
					render_scanline_tile(pix + ppu.dot, 0, 8);
					ppu_coarse_x_inc();
				}
				RETURN(8);
			}

			if (PPU_IS_SHOW_BACKGROUND()) {
				render_scanline_tile(pix + ppu.dot, 0, ppu.x);
				ppu_coarse_x_inc();
			}
			RETURN(ppu.x);

			if (PPU_IS_SHOW_SPR()) {
				render_scanline_sprite(pix);
			}

			if (PPU_IS_SHOW_BACKGROUND()) {
				ppu_y_inc();
			}

			RETURN(1);

			if (PPU_IS_SHOW_BACKGROUND()) {
				ppu.v = (ppu.v & ~0x41f) | (ppu.t & 0x41f);
			}

			RETURN(HBLANK - 1);
		}

		ppu.scanline++;
		RETURN(WIDTH + HBLANK);

		ppu.status |= ST_VBLANK;

		if (ppu.ctrl & 0x80) {
			nes_cpu_nmi();
		}

		// nes_ppu_nametable();
		sdl_render(pixels, WIDTH);
		while (ppu.scanline++ < SCANLINE + 20) {
			RETURN(WIDTH + HBLANK);
		}
	}

	PT_END(&ppu.pt);
}

int nes_ppu_eval(void)
{
	int cycle;
	ppu_working(&cycle);
	return cycle;
}

void nes_ppu_init(void)
{
	PT_INIT(&ppu.pt);
	ppu.w = 0;
}
