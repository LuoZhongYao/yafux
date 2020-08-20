/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 * 
 * SPDX-License-Identifier: 
 */


#include "nes.h"
#include <stdint.h>
#include <string.h>
#include <assert.h>

#define NES_PPU_CTRL	0x2000
#define NES_PPU_MASK	0x2001
#define NES_PPU_STATUS	0x2002
#define NES_PPU_OAMADDR	0x2003
#define NES_PPU_OAMDATA	0x2004
#define NES_PPU_SCROLL	0x2005
#define NES_PPU_ADDR	0x2006
#define NES_PPU_DATA	0x2007
#define NES_IO_OAMDMA	0x4014
#define NES_IO_JOY1		0x4016
#define NES_IO_JOY2		0x4017

#define NES_ST_C	0x01
#define NES_ST_Z	0x02
#define NES_ST_I	0x04
#define NES_ST_D	0x08
#define NES_ST_N	0x80

#define NES_ST_NZ	(NES_ST_N | NES_ST_Z)
#define NES_ST_NZC	(NES_ST_NZ | NES_ST_C)

#define NES_PC_IMM	2
#define NES_PC_ZP	2
#define NES_PC_ZPX	2
#define NES_PC_ZPY	2
#define NES_PC_IZX	2
#define NES_PC_IZY	2
#define NES_PC_ABS	3
#define NES_PC_ABX	3
#define NES_PC_ABY	3
#define NES_PC_IND	3
#define NES_PC_REL	2
#define NES_PC_IMP	1

union NES_ST {
	uint8_t val;
	struct {
		uint8_t C:1;        /* 进位 */
		uint8_t Z:1;        /* 零标志 */
		uint8_t I:1;        /* 中断禁止 */
		uint8_t D:1;        /* 十进制运算标志 */
		uint8_t B:1;	    /* */ 
		uint8_t U:1;	    /* */ 
		uint8_t V:1;        /* 溢出标志 */
		uint8_t N:1;        /* 负数标志 */
	};
};

struct nescpu {
	uint8_t A;
	uint8_t Y;
	uint8_t X;
	uint8_t S;
	uint8_t Z;
	union NES_ST P;
	uint16_t PC;

	uint8_t ram[0x800];		/* RAM[0x0000 - 0x07ff] */
};

static struct nescpu cpu;

void nes_cpu_reset(void)
{
	cpu.S -= 3;
}

uint8_t nes_get_val8(uint32_t addr)
{
	uint8_t v = 0;

	switch (addr) {
	case NES_IO_JOY1:
	case NES_IO_JOY2: v = nes_psg_read(addr - NES_IO_JOY1); break;
	case 0x0000 ... 0x1fff: v = cpu.ram[addr & 0x7fff]; break;
	case 0x2000 ... 0x3fff: v = ppu_io_read(addr); break;
	case 0x8000 ... 0xffff: v = mapper_prg_read(mapper, addr - 0x8000); break;
	}

	return v;
}

uint16_t nes_get_val16(uint32_t addr)
{
	return nes_get_val8(addr) | (nes_get_val8(addr + 1) << 8);
}

static inline void nes_put_val8(uint32_t addr, uint8_t val)
{
	switch (addr) {
	case NES_IO_JOY1:
	case NES_IO_JOY2: nes_psg_write(addr - NES_IO_JOY1, val); break;
	case NES_IO_OAMDMA: ppu_oamdma_write(val); break;
	case 0x0000 ... 0x1fff: cpu.ram[addr & 0x7ff] = val; break;
	case 0x2000 ... 0x3fff: ppu_io_write(addr, val); break;
	case 0x8000 ... 0xffff: mapper_prg_write(mapper, addr - 0x8000, val); break;
	}
}

/* Immediate addressing */
static uint32_t nes_imm(uint16_t pc)
{
	return pc + 1;
}

/* zero page addressing */
static uint32_t nes_zp(uint16_t pc)
{
	return nes_get_val8(pc + 1);
}

/* zero page X index */
static uint32_t nes_zpx(uint16_t pc)
{
	return (nes_get_val8(pc + 1) + cpu.X) & 0xFF;
}

/* zero page Y index */
static uint32_t nes_zpy(uint16_t pc)
{
	return (nes_get_val8(pc + 1) + cpu.Y) & 0xFF;
}

/* zero page X index and then indirect addressing */
static uint32_t nes_izx(uint16_t pc)
{
	uint8_t idx = nes_get_val8(pc + 1) + cpu.X;
	return cpu.ram[idx] | cpu.ram[(idx + 1) & 0xff] << 8;
}

/* zero page Y index and then indirect addressing */
static uint32_t nes_izy(uint16_t pc)
{
	uint8_t idx = nes_get_val8(pc + 1);
	return ((cpu.ram[idx] | (cpu.ram[(idx + 1) & 0xff] << 8)) + cpu.Y) & 0xFFFF;
}

static uint32_t nes_abs(uint16_t pc)
{
	return nes_get_val8(pc + 1) | nes_get_val8(pc + 2) << 8;
}

static uint32_t nes_abx(uint16_t pc)
{
	return ((nes_get_val8(pc + 1) | nes_get_val8(pc + 2) << 8) + cpu.X) & 0xffff;
}

static uint32_t nes_aby(uint16_t pc)
{
	return ((nes_get_val8(pc + 1) | nes_get_val8(pc + 2) << 8) + cpu.Y) & 0xffff;
}

static uint32_t nes_ind(uint16_t pc)
{
	uint16_t addr = nes_get_val8(pc + 1) | nes_get_val8(pc + 2) << 8;

	return nes_get_val8(addr) | (nes_get_val8((addr & 0xff00) | ((addr + 1) & 0xff)) << 8);
}

static uint32_t nes_rel(uint16_t pc)
{
	return cpu.PC + (int8_t)nes_get_val8(pc + 1);
}

static uint32_t nes_RESEST(uint16_t pc)
{
	return 0xfffc;
}

static void NES_ST_P(uint8_t mask, uint16_t v)
{
	union NES_ST P;
	P.C = !!(v & 0x100);
	P.Z = !(v & 0xff);
	P.N = !!(v & 0x80);
	cpu.P.val &= ~mask;
	cpu.P.val |= (P.val & mask);
}

static void ORA(uint32_t addr)
{
	cpu.A |= nes_get_val8(addr);
	NES_ST_P(NES_ST_NZ, cpu.A);
}

static void AND(uint32_t addr)
{
	cpu.A &= nes_get_val8(addr);
	NES_ST_P(NES_ST_NZ, cpu.A);
}

static void XOR(uint32_t addr)
{
	cpu.A ^= nes_get_val8(addr);
	NES_ST_P(NES_ST_NZ, cpu.A);
}

static void ADC(uint32_t addr)
{
	uint8_t v2;
	uint16_t v3;

	v2 = nes_get_val8(addr);

	v3 = cpu.A + v2 + cpu.P.C;
	cpu.P.V = !!(~(cpu.A ^ v2) & (cpu.A ^ v3) & 0x80);
	cpu.A = v3 & 0xff;
	NES_ST_P(NES_ST_NZC, v3);
}

static void SBC(uint32_t addr)
{
	uint8_t v2;
	uint16_t v3;

	v2 = nes_get_val8(addr);

	v3 = cpu.A - v2 - (1 - cpu.P.C);
	cpu.P.V = !!((cpu.A ^ v2) & (cpu.A ^ v3) & 0x80);
	cpu.A = v3 & 0xff;
	NES_ST_P(NES_ST_NZC, cpu.A | (~v3 & 0x100));
}

static void INC(uint32_t addr)
{
	uint16_t v = nes_get_val8(addr) + 1;
	nes_put_val8(addr, v);
	NES_ST_P(NES_ST_NZ, v);
}

static void INC_X(void)
{
	uint16_t v = cpu.X + 1;
	cpu.X = v;
	NES_ST_P(NES_ST_NZ, v);
}

static void INC_Y(void)
{
	uint16_t v = cpu.Y + 1;
	cpu.Y = v;
	NES_ST_P(NES_ST_NZ, v);
}

static void DEC(uint32_t addr)
{
	uint16_t v = nes_get_val8(addr) - 1;
	nes_put_val8(addr, v);
	NES_ST_P(NES_ST_NZ, v);
}

static void DEC_X(void)
{
	uint16_t v = cpu.X - 1;
	cpu.X = v;
	NES_ST_P(NES_ST_NZ, v);
}

static void DEC_Y(void)
{
	uint16_t v = cpu.Y - 1;
	cpu.Y = v;
	NES_ST_P(NES_ST_NZ, v);
}

static void CMP(uint8_t v1, uint32_t addr)
{
	uint8_t v2 = nes_get_val8(addr);
	uint16_t v3 = ((v1 - v2) & 0xff) | ((v1 >= v2) ? 0x100 : 0);
	NES_ST_P(NES_ST_NZC, v3);
}

static void ASL(uint32_t addr)
{
	uint16_t v = nes_get_val8(addr) << 1;
	nes_put_val8(addr, v);
	NES_ST_P(NES_ST_NZC, v);
}

static void ASL_A(void)
{
	uint16_t v = cpu.A << 1;
	cpu.A = v;
	NES_ST_P(NES_ST_NZC, v);
}

static void ROL(uint32_t addr)
{
	uint16_t v = (nes_get_val8(addr) << 1) + cpu.P.C;
	nes_put_val8(addr, v);
	NES_ST_P(NES_ST_NZC, v);
}

static void ROL_A(void)
{
	uint16_t v = (cpu.A << 1) + cpu.P.C;
	cpu.A = v;
	NES_ST_P(NES_ST_NZC, v);
}

static void LSR(uint32_t addr)
{
	uint16_t v = nes_get_val8(addr);
	v = (v >> 1) | ((v & 1) << 8);
	nes_put_val8(addr, v);
	NES_ST_P(NES_ST_NZC, v);
}

static void LSR_A(void)
{
	uint16_t v = (cpu.A >> 1) | ((cpu.A & 1) << 8);
	cpu.A = v;
	NES_ST_P(NES_ST_NZC, v);
}

static void ROR(uint32_t addr)
{
	uint16_t v = nes_get_val8(addr);
	v = ((v >> 1) | + cpu.P.C * 128) | ((v & 1) << 8);
	nes_put_val8(addr, v);
	NES_ST_P(NES_ST_NZC, v);
}

static void ROR_A(void)
{
	uint16_t v = ((cpu.A >> 1) | + cpu.P.C * 128) | ((cpu.A & 1) << 8);
	cpu.A = v;
	NES_ST_P(NES_ST_NZC, v);
}

static void LDA(uint32_t addr)
{
	cpu.A = nes_get_val8(addr);
	NES_ST_P(NES_ST_NZ, cpu.A);
}

static void LDX(uint32_t addr)
{
	cpu.X = nes_get_val8(addr);
	NES_ST_P(NES_ST_NZ, cpu.X);
}

static void LDY(uint32_t addr)
{
	cpu.Y = nes_get_val8(addr);
	NES_ST_P(NES_ST_NZ, cpu.Y);
}

static void STA(uint32_t addr)
{
	nes_put_val8(addr, cpu.A);
}

static void STX(uint32_t addr)
{
	nes_put_val8(addr, cpu.X);
}

static void STY(uint32_t addr)
{
	nes_put_val8(addr, cpu.Y);
}

static void SAX(uint32_t addr)
{
	nes_put_val8(addr, cpu.A & cpu.X);
}

static void PHA(void)
{
	nes_put_val8(0x100 + cpu.S--, cpu.A);
}

static void PHP(void)
{
	uint8_t v = cpu.P.val;
	v |= 0x30;
	nes_put_val8(0x100 + cpu.S--, v);
}

static void PLA(void)
{
	cpu.A =nes_get_val8(0x100 + ++cpu.S);
	NES_ST_P(NES_ST_NZ, cpu.A);
}

static void PLP(void)
{
	cpu.P.val = (nes_get_val8(0x100 + ++cpu.S) & 0xEF) | 0x20;
}

static void BPL(uint32_t addr)
{
	if (cpu.P.N == 0) {
		cpu.PC = addr;
	}
}

static void BMI(uint32_t addr)
{
	if (cpu.P.N == 1) {
		cpu.PC = addr;
	}
}

static void BVC(uint32_t addr)
{
	if (cpu.P.V == 0) {
		cpu.PC = addr;
	}
}

static void BVS(uint32_t addr)
{
	if (cpu.P.V == 1) {
		cpu.PC = addr;
	}
}

static void BCC(uint32_t addr)
{
	if (cpu.P.C == 0) {
		cpu.PC = addr;
	}
}

static void BCS(uint32_t addr)
{
	if (cpu.P.C == 1) {
		cpu.PC = addr;
	}
}

static void BNE(uint32_t addr)
{
	if (cpu.P.Z == 0) {
		cpu.PC = addr;
	}
}

static void BEQ(uint32_t addr)
{
	if (cpu.P.Z == 1) {
		cpu.PC = addr;
	}
}

static void INT(uint32_t addr)
{
	nes_put_val8(0x100 + cpu.S--, (cpu.PC >> 8) & 0xff);
	nes_put_val8(0x100 + cpu.S--, cpu.PC & 0xff);
	nes_put_val8(0x100 + cpu.S--, cpu.P.val | 0x20);

	cpu.PC = nes_get_val16(addr);
}

static void BRK(uint32_t addr)
{
	nes_put_val8(0x100 + cpu.S--, (cpu.PC >> 8) & 0xff);
	nes_put_val8(0x100 + cpu.S--, cpu.PC & 0xff);
	nes_put_val8(0x100 + cpu.S--, cpu.P.val | 0x30);

	cpu.PC = nes_get_val16(addr);
}

static void RTI(void)
{
	cpu.P.val = nes_get_val8(0x100 + ++cpu.S) | 0x20;
	cpu.PC = nes_get_val8(0x100    + ++cpu.S);
	cpu.PC |= nes_get_val8(0x100   + ++cpu.S) << 8;
}

static void JSR(uint32_t addr)
{
	uint16_t v = cpu.PC - 1;

	nes_put_val8(0x100 + cpu.S--, (v >> 8) & 0xff);
	nes_put_val8(0x100 + cpu.S--, v & 0xff);

	cpu.PC = addr & 0xffff;
}

static void RTS(void)
{
	uint16_t v;

	v = nes_get_val8(0x100 + ++cpu.S);
	v |= nes_get_val8(0x100  + ++cpu.S) << 8;
	cpu.PC = v + 1;
}

static void JMP(uint32_t addr)
{
	cpu.PC = addr & 0xffff;
}

static void BIT(uint32_t addr)
{
	uint8_t v = nes_get_val8(addr);
	cpu.P.N = !!(v & 0x80);
	cpu.P.V = !!(v & 0x40);
	cpu.P.Z = !(cpu.A & v);
}

static void CLC(void)
{
	cpu.P.C = 0;
}

static void SEC(void)
{
	cpu.P.C = 1;
}

static void CLD(void)
{
	cpu.P.D = 0;
}

static void SED(void)
{
	cpu.P.D = 1;
}

static void CLI(void)
{
	cpu.P.I = 0;
}

static void SEI(void)
{
	cpu.P.I = 1;
}

static void CLV(void)
{
	cpu.P.V = 0;
}

static void SLO(uint32_t addr)
{
	uint16_t v = nes_get_val8(addr) << 1;
	nes_put_val8(addr, v & 0xff);
	cpu.A = (v |= cpu.A);

	NES_ST_P(NES_ST_NZC, v);
}

static void RLA(uint32_t addr)
{
	uint16_t v;

	v = (nes_get_val8(addr) << 1) | cpu.P.C;
	nes_put_val8(addr, v & 0xff);

	cpu.A  &= v;
	NES_ST_P(NES_ST_NZC, v & (cpu.A | 0x100));
}

static void SRE(uint32_t addr)
{
	uint16_t v;

	v = nes_get_val8(addr);
	v = ((v & 1) << 8) | (v >> 1);
	nes_put_val8(addr, v & 0xff);

	cpu.A = (v ^= cpu.A);
	NES_ST_P(NES_ST_NZC, v);
}

static void RRA(uint32_t addr)
{
	uint8_t v, c;
	uint16_t r;

	v = nes_get_val8(addr);
	c = v & 1;
	v = (v >> 1) | (cpu.P.C << 7);
	nes_put_val8(addr, v & 0xff);

	r = cpu.A + v + c;
	cpu.P.V = !!(~(cpu.A ^ v) & (cpu.A ^ r) & 0x80);
	NES_ST_P(NES_ST_NZC, r);
	cpu.A = r;
}

static void LAX(uint32_t addr)
{
	uint8_t v = nes_get_val8(addr);
	cpu.A = cpu.X = v;
	NES_ST_P(NES_ST_NZ, v);
}

static void DCP(uint32_t addr)
{
	nes_put_val8(addr, nes_get_val8(addr) - 1);
	CMP(cpu.A, addr);
}

static void ISC(uint32_t addr)
{
	nes_put_val8(addr, nes_get_val8(addr) + 1);
	SBC(addr);
}

static void ALR(uint32_t addr)
{
	cpu.A = (cpu.A & nes_get_val8(addr)) >> 1;
	NES_ST_P(NES_ST_NZC, cpu.A);
}

static void XAA(uint32_t addr)
{
	cpu.A = (cpu.X & nes_get_val8(addr)) >> 1;
	NES_ST_P(NES_ST_NZC, cpu.A);
}

static void ANC(uint32_t addr)
{
	cpu.A = cpu.A & nes_get_val8(addr);
	NES_ST_P(NES_ST_NZC, cpu.A);
}

static void AXS(uint32_t addr)
{
	cpu.X = (cpu.A & cpu.X) - nes_get_val8(addr);
	NES_ST_P(NES_ST_NZC, cpu.X);
}

static void AHX(uint32_t addr, uint16_t pc)
{
	uint16_t v = cpu.A & cpu.X & pc >> 8;
	nes_put_val8(addr, v);
}

static void SHXY(uint32_t addr, uint16_t pc)
{
	nes_put_val8(addr, cpu.Y & pc >> 8);
}

static void TAS(uint32_t addr, uint16_t pc)
{
	cpu.S = cpu.A & cpu.X;
	nes_put_val8(addr, cpu.S & pc >> 8);
}

static void LAS(uint32_t addr)
{
	cpu.S &= nes_get_val8(addr);
	cpu.X = cpu.S;

	cpu.A = cpu.S;
	NES_ST_P(NES_ST_NZ, cpu.A);
}

void nes_cpu_init(void)
{
	cpu.S = 0xFD;
	cpu.P.val = 0x24;
	cpu.PC = /* 0xc000;// */nes_get_val16(0xfffc);
	memset(cpu.ram, 0xff, sizeof(cpu.ram));
}

#include <stdio.h>

static void dump_stack(uint8_t sp)
{
	for (uint8_t i = sp;i < 0xff;i++) {
		printf("[%02x]: %02x\n", i, nes_get_val8(0x100 + i));
	}
}

void nes_cpu_nmi(void)
{
	INT(0xfffa);
}

int nes_cpu_eval(void)
{
	uint8_t op;
	uint16_t pc = cpu.PC;

	op = nes_get_val8(pc);

	// printf("%04X  %02X A:%02X X:%02X Y:%02X P:%02X SP:%02X"
	// 	//" %c%c%c%c%c%c%c",
	// 	"\n",
	// 	pc, op, cpu.A, cpu.X, cpu.Y, cpu.P.val, cpu.S
	// 	//cpu.P.N?'N':' ',
	// 	//cpu.P.V?'V':' ',
	// 	//cpu.P.B?'B':' ',
	// 	//cpu.P.D?'D':' ',
	// 	//cpu.P.I?'I':' ',
	// 	//cpu.P.Z?'Z':' ',
	// 	//cpu.P.C?'C':' '
	// 	);


#define _(cyc, step, ...) { cpu.PC += step; __VA_ARGS__; return cyc; }

	switch (op) {
	default:
		cpu.PC += 1;
		return 2;
	break;
	case 0x80: _(2, NES_PC_IMM); break;
	case 0x82: _(2, NES_PC_IMM); break;
	case 0x89: _(2, NES_PC_IMM); break;
	case 0xc2: _(2, NES_PC_IMM); break;
	case 0xe2: _(2, NES_PC_IMM); break;
	case 0x0c: _(4, NES_PC_ABS); break;
	case 0x1c: _(4, NES_PC_ABX); break;
	case 0x3c: _(4, NES_PC_ABX); break;
	case 0x5c: _(4, NES_PC_ABX); break;
	case 0x7c: _(4, NES_PC_ABX); break;
	case 0xdc: _(4, NES_PC_ABX); break;
	case 0xfc: _(4, NES_PC_ABX); break;
	case 0x04: _(3, NES_PC_ZP ); break;
	case 0x44: _(3, NES_PC_ZP ); break;
	case 0x64: _(3, NES_PC_ZP ); break;
	case 0x14: _(5, NES_PC_ZPX); break;
	case 0x34: _(5, NES_PC_ZPX); break;
	case 0x54: _(5, NES_PC_ZPX); break;
	case 0x74: _(5, NES_PC_ZPX); break;
	case 0xd4: _(5, NES_PC_ZPX); break;
	case 0xf4: _(5, NES_PC_ZPX); break;
	case 0x09: _(2, NES_PC_IMM, ORA(nes_imm(pc))); break;
	case 0x05: _(3, NES_PC_ZP , ORA(nes_zp (pc))); break;
	case 0x15: _(4, NES_PC_ZPX, ORA(nes_zpx(pc))); break;
	case 0x01: _(6, NES_PC_IZX, ORA(nes_izx(pc))); break;
	case 0x11: _(5, NES_PC_IZY, ORA(nes_izy(pc))); break;
	case 0x0d: _(4, NES_PC_ABS, ORA(nes_abs(pc))); break;
	case 0x1d: _(4, NES_PC_ABX, ORA(nes_abx(pc))); break;
	case 0x19: _(4, NES_PC_ABY, ORA(nes_aby(pc))); break;

	case 0x29: _(2, NES_PC_IMM, AND(nes_imm(pc))); break;
	case 0x25: _(3, NES_PC_ZP , AND(nes_zp (pc))); break;
	case 0x35: _(4, NES_PC_ZPX, AND(nes_zpx(pc))); break;
	case 0x21: _(6, NES_PC_IZX, AND(nes_izx(pc))); break;
	case 0x31: _(5, NES_PC_IZY, AND(nes_izy(pc))); break;
	case 0x2d: _(4, NES_PC_ABS, AND(nes_abs(pc))); break;
	case 0x3d: _(4, NES_PC_ABX, AND(nes_abx(pc))); break;
	case 0x39: _(4, NES_PC_ABY, AND(nes_aby(pc))); break;

	case 0x49: _(2, NES_PC_IMM, XOR(nes_imm(pc))); break;
	case 0x45: _(3, NES_PC_ZP , XOR(nes_zp (pc))); break;
	case 0x55: _(4, NES_PC_ZPX, XOR(nes_zpx(pc))); break;
	case 0x41: _(6, NES_PC_IZX, XOR(nes_izx(pc))); break;
	case 0x51: _(5, NES_PC_IZY, XOR(nes_izy(pc))); break;
	case 0x4d: _(4, NES_PC_ABS, XOR(nes_abs(pc))); break;
	case 0x5d: _(4, NES_PC_ABX, XOR(nes_abx(pc))); break;
	case 0x59: _(4, NES_PC_ABY, XOR(nes_aby(pc))); break;

	case 0x69: _(2, NES_PC_IMM, ADC(nes_imm(pc))); break;
	case 0x65: _(3, NES_PC_ZP , ADC(nes_zp (pc))); break;
	case 0x75: _(4, NES_PC_ZPX, ADC(nes_zpx(pc))); break;
	case 0x61: _(6, NES_PC_IZX, ADC(nes_izx(pc))); break;
	case 0x71: _(5, NES_PC_IZY, ADC(nes_izy(pc))); break;
	case 0x6d: _(4, NES_PC_ABS, ADC(nes_abs(pc))); break;
	case 0x7d: _(4, NES_PC_ABX, ADC(nes_abx(pc))); break;
	case 0x79: _(4, NES_PC_ABY, ADC(nes_aby(pc))); break;

	case 0xe9: _(2, NES_PC_IMM, SBC(nes_imm(pc))); break;
	case 0xe5: _(3, NES_PC_ZP , SBC(nes_zp (pc))); break;
	case 0xf5: _(4, NES_PC_ZPX, SBC(nes_zpx(pc))); break;
	case 0xe1: _(6, NES_PC_IZX, SBC(nes_izx(pc))); break;
	case 0xf1: _(5, NES_PC_IZY, SBC(nes_izy(pc))); break;
	case 0xed: _(4, NES_PC_ABS, SBC(nes_abs(pc))); break;
	case 0xfd: _(4, NES_PC_ABX, SBC(nes_abx(pc))); break;
	case 0xf9: _(4, NES_PC_ABY, SBC(nes_aby(pc))); break;

	case 0xc9: _(2, NES_PC_IMM, CMP(cpu.A, nes_imm(pc))); break;
	case 0xc5: _(3, NES_PC_ZP , CMP(cpu.A, nes_zp (pc))); break;
	case 0xd5: _(4, NES_PC_ZPX, CMP(cpu.A, nes_zpx(pc))); break;
	case 0xc1: _(6, NES_PC_IZX, CMP(cpu.A, nes_izx(pc))); break;
	case 0xd1: _(5, NES_PC_IZY, CMP(cpu.A, nes_izy(pc))); break;
	case 0xcd: _(4, NES_PC_ABS, CMP(cpu.A, nes_abs(pc))); break;
	case 0xdd: _(4, NES_PC_ABX, CMP(cpu.A, nes_abx(pc))); break;
	case 0xd9: _(4, NES_PC_ABY, CMP(cpu.A, nes_aby(pc))); break;

	case 0xc0: _(2, NES_PC_IMM, CMP(cpu.Y, nes_imm(pc))); break;
	case 0xc4: _(3, NES_PC_ZP , CMP(cpu.Y, nes_zp (pc))); break;
	case 0xcc: _(4, NES_PC_ABS, CMP(cpu.Y, nes_abs(pc))); break;

	case 0xe0: _(2, NES_PC_IMM, CMP(cpu.X, nes_imm(pc))); break;
	case 0xe4: _(3, NES_PC_ZP , CMP(cpu.X, nes_zp (pc))); break;
	case 0xec: _(4, NES_PC_ABS, CMP(cpu.X, nes_abs(pc))); break;

	case 0xc6: _(5, NES_PC_ZP , DEC(nes_zp (pc))); break;
	case 0xd6: _(6, NES_PC_ZPX, DEC(nes_zpx(pc))); break;
	case 0xce: _(6, NES_PC_ABS, DEC(nes_abs(pc))); break;
	case 0xde: _(7, NES_PC_ABX, DEC(nes_abx(pc))); break;

	case 0xca: _(2, NES_PC_IMP, DEC_X()); break;
	case 0x88: _(2, NES_PC_IMP, DEC_Y()); break;

	case 0xe6: _(5, NES_PC_ZP , INC(nes_zp (pc))); break;
	case 0xf6: _(6, NES_PC_ZPX, INC(nes_zpx(pc))); break;
	case 0xee: _(6, NES_PC_ABS, INC(nes_abs(pc))); break;
	case 0xfe: _(7, NES_PC_ABX, INC(nes_abx(pc))); break;

	case 0xe8: _(2, NES_PC_IMP, INC_X()); break;
	case 0xc8: _(2, NES_PC_IMP, INC_Y()); break;

	case 0x0a: _(2, NES_PC_IMP, ASL_A()); break;
	case 0x06: _(5, NES_PC_ZP , ASL(nes_zp (pc))); break;
	case 0x16: _(6, NES_PC_ZPX, ASL(nes_zpx(pc))); break;
	case 0x0e: _(6, NES_PC_ABS, ASL(nes_abs(pc))); break;
	case 0x1e: _(7, NES_PC_ABX, ASL(nes_abx(pc))); break;


	case 0x2a: _(2, NES_PC_IMP, ROL_A()); break;
	case 0x26: _(5, NES_PC_ZP , ROL(nes_zp (pc))); break;
	case 0x36: _(6, NES_PC_ZPX, ROL(nes_zpx(pc))); break;
	case 0x2e: _(6, NES_PC_ABS, ROL(nes_abs(pc))); break;
	case 0x3e: _(7, NES_PC_ABX, ROL(nes_abx(pc))); break;

	case 0x4a: _(2, NES_PC_IMP, LSR_A()); break;
	case 0x46: _(5, NES_PC_ZP , LSR(nes_zp (pc))); break;
	case 0x56: _(6, NES_PC_ZPX, LSR(nes_zpx(pc))); break;
	case 0x4e: _(6, NES_PC_ABS, LSR(nes_abs(pc))); break;
	case 0x5e: _(7, NES_PC_ABX, LSR(nes_abx(pc))); break;

	case 0x6a: _(2, NES_PC_IMP, ROR_A()); break;
	case 0x66: _(5, NES_PC_ZP , ROR(nes_zp (pc))); break;
	case 0x76: _(6, NES_PC_ZPX, ROR(nes_zpx(pc))); break;
	case 0x6e: _(6, NES_PC_ABS, ROR(nes_abs(pc))); break;
	case 0x7e: _(7, NES_PC_ABX, ROR(nes_abx(pc))); break;

	case 0xa9: _(2, NES_PC_IMM, LDA(nes_imm(pc))); break;
	case 0xa5: _(3, NES_PC_ZP , LDA(nes_zp (pc))); break;
	case 0xb5: _(4, NES_PC_ZPX, LDA(nes_zpx(pc))); break;
	case 0xa1: _(6, NES_PC_IZX, LDA(nes_izx(pc))); break;
	case 0xb1: _(5, NES_PC_IZY, LDA(nes_izy(pc))); break;
	case 0xad: _(4, NES_PC_ABS, LDA(nes_abs(pc))); break;
	case 0xbd: _(4, NES_PC_ABX, LDA(nes_abx(pc))); break;
	case 0xb9: _(4, NES_PC_ABY, LDA(nes_aby(pc))); break;

	case 0x85: _(3, NES_PC_ZP , STA(nes_zp (pc))); break;
	case 0x95: _(4, NES_PC_ZPX, STA(nes_zpx(pc))); break;
	case 0x81: _(6, NES_PC_IZX, STA(nes_izx(pc))); break;
	case 0x91: _(6, NES_PC_IZY, STA(nes_izy(pc))); break;
	case 0x8d: _(4, NES_PC_ABS, STA(nes_abs(pc))); break;
	case 0x9d: _(5, NES_PC_ABX, STA(nes_abx(pc))); break;
	case 0x99: _(5, NES_PC_ABY, STA(nes_aby(pc))); break;

	case 0xa2: _(2, NES_PC_IMM, LDX(nes_imm(pc))); break;
	case 0xa6: _(3, NES_PC_ZP , LDX(nes_zp (pc))); break;
	case 0xb6: _(4, NES_PC_ZPY, LDX(nes_zpy(pc))); break;
	case 0xae: _(4, NES_PC_ABS, LDX(nes_abs(pc))); break;
	case 0xbe: _(4, NES_PC_ABY, LDX(nes_aby(pc))); break;

	case 0x86: _(3, NES_PC_ZP , STX(nes_zp (pc))); break;
	case 0x96: _(4, NES_PC_ZPY, STX(nes_zpy(pc))); break;
	case 0x8e: _(4, NES_PC_ABS, STX(nes_abs(pc))); break;

	case 0xa0: _(2, NES_PC_IMM, LDY(nes_imm(pc))); break;
	case 0xa4: _(3, NES_PC_ZP , LDY(nes_zp (pc))); break;
	case 0xb4: _(4, NES_PC_ZPX, LDY(nes_zpx(pc))); break;
	case 0xac: _(4, NES_PC_ABS, LDY(nes_abs(pc))); break;
	case 0xbc: _(4, NES_PC_ABX, LDY(nes_abx(pc))); break;

	case 0x84: _(3, NES_PC_ZP , STY(nes_zp (pc))); break;
	case 0x94: _(4, NES_PC_ZPX, STY(nes_zpx(pc))); break;
	case 0x8c: _(4, NES_PC_ABS, STY(nes_abs(pc))); break;

#define LDR(a, b, c) do { a = b; NES_ST_P(c, a); } while(0)
	case 0xaa: _(2, NES_PC_IMP, LDR(cpu.X, cpu.A, NES_ST_NZ)); break;
	case 0x8a: _(2, NES_PC_IMP, LDR(cpu.A, cpu.X, NES_ST_NZ)); break;
	case 0xa8: _(2, NES_PC_IMP, LDR(cpu.Y, cpu.A, NES_ST_NZ)); break;
	case 0x98: _(2, NES_PC_IMP, LDR(cpu.A, cpu.Y, NES_ST_NZ)); break;
	case 0xba: _(2, NES_PC_IMP, LDR(cpu.X, cpu.S, NES_ST_NZ)); break;
	case 0x9a: _(2, NES_PC_IMP, LDR(cpu.S, cpu.X, 0)); break;

	case 0x48: _(3, NES_PC_IMP, PHA()); break;
	case 0x68: _(4, NES_PC_IMP, PLA()); break;

	case 0x08: _(3, NES_PC_IMP, PHP()); break;
	case 0x28: _(4, NES_PC_IMP, PLP()); break;

	case 0x10: _(2, NES_PC_REL, BPL(nes_rel(pc))); break;
	case 0x30: _(2, NES_PC_REL, BMI(nes_rel(pc))); break;
	case 0x50: _(2, NES_PC_REL, BVC(nes_rel(pc))); break;
	case 0x70: _(2, NES_PC_REL, BVS(nes_rel(pc))); break;
	case 0x90: _(2, NES_PC_REL, BCC(nes_rel(pc))); break;
	case 0xb0: _(2, NES_PC_REL, BCS(nes_rel(pc))); break;
	case 0xd0: _(2, NES_PC_REL, BNE(nes_rel(pc))); break;
	case 0xf0: _(2, NES_PC_REL, BEQ(nes_rel(pc))); break;

	case 0x00: _(7, NES_PC_IMP, BRK(0xfffe)); break;
	case 0x40: _(6, NES_PC_IMP, RTI()); break;
	case 0x20: _(6, NES_PC_ABS, JSR(nes_abs(pc))); break;
	case 0x60: _(6, NES_PC_IMP, RTS()); break;

	case 0x4c: _(3, NES_PC_ABS, JMP(nes_abs(pc))); break;
	case 0x6c: _(5, NES_PC_IND, JMP(nes_ind(pc))); break;

	case 0x24: _(3, NES_PC_ZP , BIT(nes_zp (pc))); break;
	case 0x2c: _(4, NES_PC_ABS, BIT(nes_abs(pc))); break;

	case 0x18: _(2, NES_PC_IMP, CLC()); break;
	case 0x38: _(2, NES_PC_IMP, SEC()); break;

	case 0xd8: _(2, NES_PC_IMP, CLD()); break;
	case 0xf8: _(2, NES_PC_IMP, SED()); break;

	case 0x58: _(2, NES_PC_IMP, CLI()); break;
	case 0x78: _(2, NES_PC_IMP, SEI()); break;
	case 0xb8: _(2, NES_PC_IMP, CLV()); break;

	case 0x07: _(5, NES_PC_ZP , SLO(nes_zp (pc))); break;
	case 0x17: _(6, NES_PC_ZPX, SLO(nes_zpx(pc))); break;
	case 0x03: _(8, NES_PC_IZX, SLO(nes_izx(pc))); break;
	case 0x13: _(8, NES_PC_IZY, SLO(nes_izy(pc))); break;
	case 0x0f: _(6, NES_PC_ABS, SLO(nes_abs(pc))); break;
	case 0x1f: _(7, NES_PC_ABX, SLO(nes_abx(pc))); break;
	case 0x1b: _(7, NES_PC_ABY, SLO(nes_aby(pc))); break;

	case 0x27: _(5, NES_PC_ZP , RLA(nes_zp (pc))); break;
	case 0x37: _(6, NES_PC_ZPY, RLA(nes_zpx(pc))); break;
	case 0x23: _(8, NES_PC_IZX, RLA(nes_izx(pc))); break;
	case 0x33: _(8, NES_PC_IZY, RLA(nes_izy(pc))); break;
	case 0x2f: _(6, NES_PC_ABS, RLA(nes_abs(pc))); break;
	case 0x3f: _(7, NES_PC_ABX, RLA(nes_abx(pc))); break;
	case 0x3b: _(7, NES_PC_ABY, RLA(nes_aby(pc))); break;

	case 0x47: _(5, NES_PC_ZP , SRE(nes_zp (pc))); break;
	case 0x57: _(6, NES_PC_ZPY, SRE(nes_zpx(pc))); break;
	case 0x43: _(8, NES_PC_IZX, SRE(nes_izx(pc))); break;
	case 0x53: _(8, NES_PC_IZY, SRE(nes_izy(pc))); break;
	case 0x4f: _(6, NES_PC_ABS, SRE(nes_abs(pc))); break;
	case 0x5f: _(7, NES_PC_ABX, SRE(nes_abx(pc))); break;
	case 0x5b: _(7, NES_PC_ABY, SRE(nes_aby(pc))); break;

	case 0x67: _(5, NES_PC_ZP , RRA(nes_zp (pc))); break;
	case 0x77: _(6, NES_PC_ZPX, RRA(nes_zpx(pc))); break;
	case 0x63: _(8, NES_PC_IZX, RRA(nes_izx(pc))); break;
	case 0x73: _(8, NES_PC_IZY, RRA(nes_izy(pc))); break;
	case 0x6f: _(6, NES_PC_ABS, RRA(nes_abs(pc))); break;
	case 0x7f: _(7, NES_PC_ABX, RRA(nes_abx(pc))); break;
	case 0x7b: _(7, NES_PC_ABY, RRA(nes_aby(pc))); break;

	case 0x87: _(3, NES_PC_ZP , SAX(nes_zp (pc))); break;
	case 0x97: _(4, NES_PC_ZPY, SAX(nes_zpy(pc))); break;
	case 0x83: _(6, NES_PC_IZX, SAX(nes_izx(pc))); break;
	case 0x8f: _(4, NES_PC_ABS, SAX(nes_abs(pc))); break;

	case 0xa7: _(3, NES_PC_ZP , LAX(nes_zp (pc))); break;
	case 0xb7: _(4, NES_PC_ZPY, LAX(nes_zpy(pc))); break;
	case 0xa3: _(6, NES_PC_IZX, LAX(nes_izx(pc))); break;
	case 0xb3: _(5, NES_PC_IZY, LAX(nes_izy(pc))); break;
	case 0xaf: _(4, NES_PC_ABS, LAX(nes_abs(pc))); break;
	case 0xbf: _(4, NES_PC_ABY, LAX(nes_aby(pc))); break;

	case 0xc7: _(5, NES_PC_ZP , DCP(nes_zp (pc))); break;
	case 0xd7: _(6, NES_PC_ZPX, DCP(nes_zpx(pc))); break;
	case 0xc3: _(8, NES_PC_IZX, DCP(nes_izx(pc))); break;
	case 0xd3: _(8, NES_PC_IZY, DCP(nes_izy(pc))); break;
	case 0xcf: _(6, NES_PC_ABS, DCP(nes_abs(pc))); break;
	case 0xdf: _(7, NES_PC_ABX, DCP(nes_abx(pc))); break;
	case 0xdb: _(7, NES_PC_ABY, DCP(nes_aby(pc))); break;

	case 0xe7: _(5, NES_PC_ZP , ISC(nes_zp (pc))); break;
	case 0xf7: _(6, NES_PC_ZPX, ISC(nes_zpx(pc))); break;
	case 0xe3: _(8, NES_PC_IZX, ISC(nes_izx(pc))); break;
	case 0xf3: _(8, NES_PC_IZY, ISC(nes_izy(pc))); break;
	case 0xef: _(6, NES_PC_ABS, ISC(nes_abs(pc))); break;
	case 0xff: _(7, NES_PC_ABX, ISC(nes_abx(pc))); break;
	case 0xfb: _(7, NES_PC_ABY, ISC(nes_aby(pc))); break;

	case 0x0b: _(2, NES_PC_IMM, ANC(nes_imm(pc))); break;
	case 0x2b: _(2, NES_PC_IMM, ANC(nes_imm(pc))); break;
	case 0x4b: _(2, NES_PC_IMM, ALR(nes_imm(pc))); break;
	case 0x6b: _(2, NES_PC_IMM, ALR(nes_imm(pc))); break;
	case 0x8b: _(2, NES_PC_IMM, XAA(nes_imm(pc))); break;
	case 0xab: _(2, NES_PC_IMM, LAX(nes_imm(pc))); break;
	case 0xcb: _(2, NES_PC_IMM, AXS(nes_imm(pc))); break;
	case 0xeb: _(2, NES_PC_IMM, SBC(nes_imm(pc))); break;

	case 0x93: _(6, NES_PC_IZY, AHX(nes_izy(pc), pc)); break;
	case 0x9f: _(5, NES_PC_ABY, AHX(nes_aby(pc), pc)); break;
	case 0x9c: _(5, NES_PC_ABX, SHXY(nes_abx(pc), pc)); break;
	case 0x9e: _(5, NES_PC_ABY, SHXY(nes_aby(pc), pc)); break;
	case 0x9b: _(5, NES_PC_ABY, TAS(nes_aby(pc), pc)); break;
	case 0xbb: _(4, NES_PC_ABY, LAS(nes_aby(pc))); break;
	}
#undef _
}
