/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */

#include "nes.h"
#include <errno.h>
#include <unistd.h>
#include <stdbool.h>
#include <SDL2/SDL.h>
#include <sys/timerfd.h>

#define SPR_WIDTH	8
#define SPR_HIGHT	8
#define SDL_INTERVAL	3

static SDL_Window *win;
static SDL_Renderer *ren; 
static SDL_Texture *bgr_texture;

static SDL_Window *dbg_win;
static SDL_Renderer *dbg_ren;
static SDL_Texture *dbg_texture;
static unsigned keystate;

void sdl_render(uint32_t *pixels, unsigned nr)
{
	SDL_RenderClear(ren);
	SDL_UpdateTexture(bgr_texture, NULL, pixels, nr * sizeof(uint32_t));
	SDL_RenderCopy(ren, bgr_texture, NULL, NULL);
	SDL_RenderPresent(ren);
}

void sdl_dbg_render(uint32_t *pixels, unsigned nr, unsigned nt)
{
	SDL_Rect rect = {
		.w = 256,
		.h = 240,
		.x = 256 * (nt & 1),
		.y = 240 * ((nt >> 1) & 1)
	};

	if (nt == 0) {
		SDL_RenderClear(dbg_ren);
	}
	SDL_UpdateTexture(dbg_texture, &rect, pixels, nr * sizeof(uint32_t));

	if (nt == 3) {
		SDL_RenderCopy(dbg_ren, dbg_texture, NULL, NULL);
		SDL_RenderPresent(dbg_ren);
	}
}

static void sdl_debug_init(void)
{
	dbg_win = SDL_CreateWindow("Debug", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 2 * WIDTH, 2 * SCANLINE, SDL_WINDOW_SHOWN);
	dbg_ren = SDL_CreateRenderer(dbg_win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	dbg_texture = SDL_CreateTexture(dbg_ren, SDL_PIXELFORMAT_ARGB8888,
		SDL_TEXTUREACCESS_STREAMING, 2 * WIDTH, 2 * SCANLINE);
}

static void sdl_init(void)
{
	if (SDL_Init(SDL_INIT_EVERYTHING) == -1) {
	}

	win = SDL_CreateWindow("Nes", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, SCANLINE, SDL_WINDOW_SHOWN);
	ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	bgr_texture = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
		SDL_TEXTUREACCESS_STREAMING, WIDTH, SCANLINE);
}

static void sdl_destroy(void)
{
	SDL_DestroyTexture(bgr_texture);
	SDL_DestroyRenderer(ren);
	SDL_DestroyWindow(win);
}

int sdl_keystate(unsigned pos)
{
	return (keystate >> pos) & 1;
}

static void sdl_handle_input(SDL_Event *ev)
{
	int pos = 0;
	switch (ev->key.keysym.sym) {
	case SDLK_d: pos++; /* Up */
	case SDLK_a: pos++; /* Left */
	case SDLK_s: pos++; /* Down */
	case SDLK_w: pos++; /* Up */
	case SDLK_l: pos++; /* Start */
	case SDLK_h: pos++;	/* Select */
	case SDLK_k: pos++;	/* B */
	case SDLK_j: pos++;	/* A */
	break;
	}

	if (ev->type == SDL_KEYDOWN) {
		keystate |= (1 << pos);
	} else {
		keystate &= ~(1 << pos);
	}
}

int main(int argc, char **argv)
{
	int timerfd;
	uint64_t expired;
	bool quit = false;
	struct itimerspec it;
	SDL_Event ev;

	nes_init(argv[1]);
	sdl_init();
	sdl_debug_init();

	it.it_value.tv_sec = 0;
	it.it_value.tv_nsec = 47;
	it.it_interval.tv_sec = 0;
	it.it_interval.tv_nsec = 47 * 4;

	timerfd = timerfd_create(CLOCK_REALTIME, 0);
	timerfd_settime(timerfd, 0, &it, NULL);

	while (!quit) {
		if (sizeof(expired) != read(timerfd, &expired, sizeof(expired))) {
			expired = 0;
		}

		while (SDL_PollEvent(&ev)) {
			switch (ev.type)  {
			case SDL_QUIT:
				quit = true;
			break;

			case SDL_KEYUP:
			case SDL_KEYDOWN:
				sdl_handle_input(&ev);
			break;
			}
		}

		while (expired--) {
			nes_eval();
		}
	}

	sdl_destroy();
	return 0;
}
