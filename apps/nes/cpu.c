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

#define NES_REG_A	0x10000
#define NES_REG_X	0x10001
#define NES_REG_Y	0x10002
#define NES_REG_S	0x10003
#define NES_REG_P	0x10004
#define NES_REG_Z	0x10010

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


typedef uint32_t (*nes_addressing)(uint16_t pc);
union opfunc {
	uint16_t (*fnc)(uint16_t pc, ...);
	uint16_t (*fn1)(uint16_t pc, uint32_t a1, ...);
	uint16_t (*fn2)(uint16_t pc, uint32_t a1, uint32_t a2, ...);
	uint16_t (*fn3)(uint16_t pc, uint32_t a1, uint32_t a2, uint32_t a3, ...);
	uint16_t (*fn4)(uint16_t pc, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t a4, ...);
};

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
static uint8_t do_opcode(union opfunc op, uint16_t pc, uint8_t mask,
	nes_addressing adr1, nes_addressing adr2, nes_addressing adr3);

void nes_cpu_reset(void)
{
	cpu.S -= 3;
}

static inline uint8_t *nes_ram_addr(uint16_t addr)
{
	assert(addr < 0x2000);

	return &cpu.ram[addr & 0x7ff];
}

uint8_t nes_get_val8(uint32_t addr)
{
	uint8_t v = 0;

	switch (addr) {
	default:
		if (addr < 0x2000) {
			v = *nes_ram_addr(addr);
		} else if (addr >= 0x8000 && addr < 0x10000) {
			v = mapper_prg_read(mapper, addr - 0x8000);
		}
	break;
	case NES_IO_JOY1:
	case NES_IO_JOY2: v = nes_psg_read(addr - NES_IO_JOY1); break;
	case NES_REG_A: v = cpu.A; break;
	case NES_REG_X: v = cpu.X; break;
	case NES_REG_Y: v = cpu.Y; break;
	case NES_REG_S: v = cpu.S; break;
	case NES_REG_Z: v = 0; break;
	case NES_REG_P: v = cpu.P.val; break;
	case 0x2000 ... 0x3fff: v = ppu_io_read(addr); break;
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
	default:
		if (addr < 0x2000) {
			*nes_ram_addr(addr) = val;
		} else if (addr >= 0x8000 && addr < 0x10000) {
			mapper_prg_write(mapper, addr - 0x8000, val);
		}
	break;
	case NES_REG_A: cpu.A = val; break;
	case NES_REG_X: cpu.X = val; break;
	case NES_REG_Y: cpu.Y = val; break;
	case NES_REG_S: cpu.S = val; break;
	case NES_REG_P: cpu.P.val = val; break;
	case NES_IO_JOY1:
	case NES_IO_JOY2: nes_psg_write(addr - NES_IO_JOY1, val); break;
	case NES_IO_OAMDMA: ppu_oamdma_write(val); break;
	case 0x2000 ... 0x3fff: ppu_io_write(addr, val); break;
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

static uint32_t nes_a(uint16_t pc)
{
	return NES_REG_A;
}

static uint32_t nes_x(uint16_t pc)
{
	return NES_REG_X;
}

static uint32_t nes_y(uint16_t pc)
{
	return NES_REG_Y;
}

static uint32_t nes_p(uint16_t pc)
{
	return NES_REG_P;
}

static uint32_t nes_s(uint16_t pc)
{
	return NES_REG_S;
}

static uint32_t nes_z(uint16_t pc)
{
	return NES_REG_Z;
}

static uint32_t nes_NMI(uint16_t pc)
{
	return 0xfffa;
}

static uint32_t nes_RESEST(uint16_t pc)
{
	return 0xfffc;
}

static uint32_t nes_IRQ(uint16_t pc)
{
	return 0xfffe;
}

static uint16_t nes_ORA(uint16_t pc, nes_addressing adr1, nes_addressing adr2, ...)
{
	return nes_get_val8(adr1(pc)) | nes_get_val8(adr2(pc));
}

static uint16_t nes_AND(uint16_t pc, nes_addressing adr1, nes_addressing adr2, ...)
{
	return nes_get_val8(adr1(pc)) & nes_get_val8(adr2(pc));
}

static uint16_t nes_XOR(uint16_t pc, nes_addressing adr1, nes_addressing adr2, ...)
{
	return nes_get_val8(adr1(pc)) ^ nes_get_val8(adr2(pc));
}

static uint16_t nes_ADC(uint16_t pc, nes_addressing adr1, nes_addressing adr2, ...)
{
	uint8_t v1, v2;
	uint16_t v3;

	v1 = nes_get_val8(adr1(pc));
	v2 = nes_get_val8(adr2(pc));

	v3 = v1 + v2 + cpu.P.C;
	cpu.P.V = !!(~(v1 ^ v2) & (v1 ^ v3) & 0x80);

	return v3;
}

static uint16_t nes_SBC(uint16_t pc, nes_addressing adr1, nes_addressing adr2, ...)
{
	uint8_t v1, v2;
	uint16_t v3;

	v1 = nes_get_val8(adr1(pc));
	v2 = nes_get_val8(adr2(pc));

	v3 = v1 - v2 - (1 - cpu.P.C);
	cpu.P.V = !!((v1 ^ v2) & (v1 ^ v3) & 0x80);

	return (v3 & 0xff) | (~v3 & 0x100);
}

static uint16_t nes_INC(uint16_t pc, nes_addressing adr1, ...)
{
	return nes_get_val8(adr1(pc)) + 1;
}

static uint16_t nes_DEC(uint16_t pc, nes_addressing adr1, ...)
{
	return nes_get_val8(adr1(pc)) - 1;
}

static uint16_t nes_CMP(uint16_t pc, nes_addressing adr1, nes_addressing adr2, ...)
{
	uint8_t v1 = nes_get_val8(adr1(pc));
	uint8_t v2 = nes_get_val8(adr2(pc));

	return ((v1 - v2) & 0xff) | ((v1 >= v2) ? 0x100 : 0);
}

static uint16_t nes_ASL(uint16_t pc, nes_addressing adr1, ...)
{
	return nes_get_val8(adr1(pc)) << 1;
}

static uint16_t nes_ROL(uint16_t pc, nes_addressing adr1, ...)
{
	return (nes_get_val8(adr1(pc)) << 1) + cpu.P.C;
}

static uint16_t nes_LSR(uint16_t pc, nes_addressing adr1, ...)
{
	uint8_t v = nes_get_val8(adr1(pc));

	return (v >> 1) | ((v & 1) << 8);
}

static uint16_t nes_ROR(uint16_t pc, nes_addressing adr1, ...)
{
	uint8_t v = nes_get_val8(adr1(pc));

	return ((v >> 1) | + cpu.P.C * 128) | ((v & 1) << 8);
}

static uint16_t nes_LD(uint16_t pc, nes_addressing adr1, ...)
{
	return nes_get_val8(adr1(pc));
}

static uint16_t nes_PHA(uint16_t pc, nes_addressing adr1, ...)
{
	uint8_t v = nes_get_val8(adr1(pc));

	nes_put_val8(0x100 + cpu.S--, v);

	return 0;
}

static uint16_t nes_PHP(uint16_t pc, nes_addressing adr1, ...)
{
	uint8_t v = nes_get_val8(adr1(pc));

	v |= (adr1 == nes_p) ? 0x30 : 0;
	nes_put_val8(0x100 + cpu.S--, v);

	return 0;
}

static uint16_t nes_PLA(uint16_t pc, ...)
{
	return nes_get_val8(0x100 + ++cpu.S);
}

static uint16_t nes_PLP(uint16_t pc, ...)
{
	return (nes_get_val8(0x100 + ++cpu.S) & 0xEF) | 0x20;
}

static uint16_t nes_BPL(uint16_t pc, nes_addressing adr1, ...)
{
	if (cpu.P.N == 0) {
		cpu.PC = adr1(pc);
	}

	return 0;
}

static uint16_t nes_BMI(uint16_t pc, nes_addressing adr1, ...)
{
	if (cpu.P.N == 1) {
		cpu.PC = adr1(pc);
	}

	return 0;
}

static uint16_t nes_BVC(uint16_t pc, nes_addressing adr1, ...)
{
	if (cpu.P.V == 0) {
		cpu.PC = adr1(pc);
	}

	return 0;
}

static uint16_t nes_BVS(uint16_t pc, nes_addressing adr1, ...)
{
	if (cpu.P.V == 1) {
		cpu.PC = adr1(pc);
	}

	return 0;
}

static uint16_t nes_BCC(uint16_t pc, nes_addressing adr1, ...)
{
	if (cpu.P.C == 0) {
		cpu.PC = adr1(pc);
	}

	return 0;
}

static uint16_t nes_BCS(uint16_t pc, nes_addressing adr1, ...)
{
	if (cpu.P.C == 1) {
		cpu.PC = adr1(pc);
	}

	return 0;
}

static uint16_t nes_BNE(uint16_t pc, nes_addressing adr1, ...)
{
	if (cpu.P.Z == 0) {
		cpu.PC = adr1(pc);
	}

	return 0;
}

static uint16_t nes_BEQ(uint16_t pc, nes_addressing adr1, ...)
{
	if (cpu.P.Z == 1) {
		cpu.PC = adr1(pc);
	}

	return 0;
}

static uint16_t nes_INT(uint16_t pc, nes_addressing adr1, ...)
{
	nes_put_val8(0x100 + cpu.S--, (cpu.PC >> 8) & 0xff);
	nes_put_val8(0x100 + cpu.S--, cpu.PC & 0xff);
	nes_put_val8(0x100 + cpu.S--, cpu.P.val | 0x20);

	cpu.PC = nes_get_val16(adr1(pc));

	return 0;
}

static uint16_t nes_BRK(uint16_t pc, nes_addressing adr1, ...)
{
	nes_put_val8(0x100 + cpu.S--, (cpu.PC >> 8) & 0xff);
	nes_put_val8(0x100 + cpu.S--, cpu.PC & 0xff);
	nes_put_val8(0x100 + cpu.S--, cpu.P.val | 0x30);

	cpu.PC = nes_get_val16(adr1(pc));

	return 0;
}

static uint16_t nes_RTI(uint16_t pc, ...)
{
	cpu.P.val = nes_get_val8(0x100 + ++cpu.S);
	cpu.PC = nes_get_val8(0x100    + ++cpu.S);
	cpu.PC |= nes_get_val8(0x100   + ++cpu.S) << 8;

	return 0;
}

static uint16_t nes_JSR(uint16_t pc, nes_addressing adr1, ...)
{
	uint16_t v = cpu.PC - 1;

	nes_put_val8(0x100 + cpu.S--, (v >> 8) & 0xff);
	nes_put_val8(0x100 + cpu.S--, v & 0xff);

	cpu.PC = adr1(pc) & 0xffff;

	return 0;
}

static uint16_t nes_RTS(uint16_t pc, ...)
{
	uint16_t v;

	v = nes_get_val8(0x100 + ++cpu.S);
	v |= nes_get_val8(0x100  + ++cpu.S) << 8;
	cpu.PC = v + 1;

	return 0;
}

static uint16_t nes_JMP(uint16_t pc, nes_addressing adr1, ...)
{
	cpu.PC = adr1(pc) & 0xffff;

	return 0;
}

static uint16_t nes_BIT(uint16_t pc, nes_addressing adr1, ...)
{
	uint8_t v = nes_get_val8(adr1(pc));
	cpu.P.N = !!(v & 0x80);
	cpu.P.V = !!(v & 0x40);
	cpu.P.Z = !(cpu.A & v);

	return 0;
}

static uint16_t nes_CLC(uint16_t pc, ...)
{
	cpu.P.C = 0;

	return 0;
}

static uint16_t nes_SEC(uint16_t pc, ...)
{
	cpu.P.C = 1;

	return 0;
}

static uint16_t nes_CLD(uint16_t pc, ...)
{
	cpu.P.D = 0;

	return 0;
}

static uint16_t nes_SED(uint16_t pc, ...)
{
	cpu.P.D = 1;

	return 0;
}
static uint16_t nes_CLI(uint16_t pc, ...)
{
	cpu.P.I = 0;

	return 0;
}

static uint16_t nes_SEI(uint16_t pc, ...)
{
	cpu.P.I = 1;

	return 0;
}

static uint16_t nes_CLV(uint16_t pc, ...)
{
	cpu.P.V = 0;

	return 0;
}

static uint16_t nes_NOP(uint16_t pc, ...)
{
	return 0;
}

static uint16_t nes_SLO(uint16_t pc, nes_addressing adr1, ...)
{
	uint16_t v;
	uint32_t addr;

	addr = adr1(pc);
	v = nes_get_val8(addr) << 1;
	nes_put_val8(addr, v & 0xff);

	return v | cpu.A;
}

static uint16_t nes_RLA(uint16_t pc, nes_addressing adr1, ...)
{
	uint16_t v;
	uint32_t addr;

	addr = adr1(pc);
	v = (nes_get_val8(addr) << 1) | cpu.P.C;
	nes_put_val8(addr, v & 0xff);

	return v & (cpu.A | 0x100);
}

static uint16_t nes_SRE(uint16_t pc, nes_addressing adr1, ...)
{
	uint16_t v;
	uint32_t addr;

	addr = adr1(pc);
	v = nes_get_val8(addr);
	v = ((v & 1) << 8) | (v >> 1);
	nes_put_val8(addr, v & 0xff);

	return v ^ cpu.A;
}

static uint16_t nes_RRA(uint16_t pc, nes_addressing adr1, ...)
{
	uint8_t v, c;
	uint16_t r;
	uint32_t addr;

	addr = adr1(pc);
	v = nes_get_val8(addr);
	c = v & 1;
	v = (v >> 1) | (cpu.P.C << 7);
	nes_put_val8(addr, v & 0xff);

	r = cpu.A + v + c;
	cpu.P.V = !!(~(cpu.A ^ v) & (cpu.A ^ r) & 0x80);

	return r;
}

static uint16_t nes_LAX(uint16_t pc, nes_addressing adr1, nes_addressing adr2, ...)
{
	uint8_t v;

	v = nes_get_val8(adr2(pc));
	nes_put_val8(adr1(pc), v);

	return v;
}

static uint16_t nes_DCP(uint16_t pc, nes_addressing adr1, ...)
{
	uint32_t addr = adr1(pc);

	nes_put_val8(addr, nes_get_val8(addr) - 1);
	return nes_CMP(pc, nes_a, adr1);
}

static uint16_t nes_ISC(uint16_t pc, nes_addressing adr1, ...)
{
	uint32_t addr = adr1(pc);

	nes_put_val8(addr, nes_get_val8(addr) + 1);
	return nes_SBC(pc, nes_a, adr1);
}

static uint16_t nes_ALR(uint16_t pc, nes_addressing adr1, nes_addressing adr2, ...)
{
	return (nes_get_val8(adr1(pc)) & nes_get_val8(adr2(pc))) >> 1;
}

static uint16_t nes_AXS(uint16_t pc, nes_addressing adr1, nes_addressing adr2, ...)
{
	uint8_t v1, v2;

	v1 = nes_get_val8(adr1(pc));
	v2 = nes_get_val8(adr2(pc));

	return (cpu.A & v1) - v2;
}

static uint16_t nes_AHX(uint16_t pc, nes_addressing adr1, nes_addressing adr2, ...)
{
	return nes_get_val8(adr1(pc)) & nes_get_val8(adr2(pc)) & pc >> 8;
}

static uint16_t nes_SHXY(uint16_t pc, nes_addressing adr1, ...)
{
	return nes_get_val8(adr1(pc)) & pc >> 8;
}

static uint16_t nes_TAS(uint16_t pc, ...)
{
	cpu.S = cpu.A & cpu.X;

	return cpu.S & pc >> 8;
}

static uint16_t nes_LAS(uint16_t pc, nes_addressing adr1, ...)
{
	cpu.S &= nes_get_val8(adr1(pc));
	cpu.X = cpu.S;

	return cpu.S;
}

#define OPCODE(cycle, pc, mask, fn, ...)	_OPCODE(pc, cycle, mask, fn, ##__VA_ARGS__, nes_z, nes_z, nes_z, nes_z, nes_z)
#define _OPCODE(_pc, _cycle, _mask, _fn, _adrr, _adr1, _adr2, _adr3, _adr4, ...)	\
{								\
	.pc	  = _pc,				\
	.mask = _mask,				\
	.op.fnc   = (void*)_fn,		\
	.adrr = _adrr,				\
	.adr1 = _adr1,				\
	.adr2 = _adr2,				\
	.adr3 = _adr3,				\
	.cycle	  = _cycle,			\
}

static const struct {
	uint8_t pc:2;
	uint8_t cycle:6;
	uint8_t mask;
	union opfunc op;
	nes_addressing adrr;
	nes_addressing adr1;
	nes_addressing adr2;
	nes_addressing adr3;
} opcodes[256] = {
	[0 ... 0xff] = OPCODE(2, NES_PC_IMP, 0, nes_NOP),
	[0x1a] = OPCODE(2, NES_PC_IMP, 0, nes_NOP),
	[0x3a] = OPCODE(2, NES_PC_IMP, 0, nes_NOP),
	[0x5a] = OPCODE(2, NES_PC_IMP, 0, nes_NOP),
	[0x7a] = OPCODE(2, NES_PC_IMP, 0, nes_NOP),
	[0xda] = OPCODE(2, NES_PC_IMP, 0, nes_NOP), 
	[0xea] = OPCODE(2, NES_PC_IMP, 0, nes_NOP),
	[0xfa] = OPCODE(2, NES_PC_IMP, 0, nes_NOP),
	[0x80] = OPCODE(2, NES_PC_IMM, 0, nes_NOP),
	[0x82] = OPCODE(2, NES_PC_IMM, 0, nes_NOP),
	[0x89] = OPCODE(2, NES_PC_IMM, 0, nes_NOP),
	[0xc2] = OPCODE(2, NES_PC_IMM, 0, nes_NOP),
	[0xe2] = OPCODE(2, NES_PC_IMM, 0, nes_NOP),
	[0x0c] = OPCODE(4, NES_PC_ABS, 0, nes_NOP, nes_z, nes_abs),
	[0x1c] = OPCODE(4, NES_PC_ABX, 0, nes_NOP, nes_z, nes_abx),
	[0x3c] = OPCODE(4, NES_PC_ABX, 0, nes_NOP, nes_z, nes_abx),
	[0x5c] = OPCODE(4, NES_PC_ABX, 0, nes_NOP, nes_z, nes_abx),
	[0x7c] = OPCODE(4, NES_PC_ABX, 0, nes_NOP, nes_z, nes_abx),
	[0xdc] = OPCODE(4, NES_PC_ABX, 0, nes_NOP, nes_z, nes_abx),
	[0xfc] = OPCODE(4, NES_PC_ABX, 0, nes_NOP, nes_z, nes_abx),
	[0x04] = OPCODE(3, NES_PC_ZP , 0, nes_NOP, nes_z, nes_zp),
	[0x44] = OPCODE(3, NES_PC_ZP , 0, nes_NOP, nes_z, nes_zp),
	[0x64] = OPCODE(3, NES_PC_ZP , 0, nes_NOP, nes_z, nes_zp),
	[0x14] = OPCODE(5, NES_PC_ZPX, 0, nes_NOP, nes_z, nes_zpx),
	[0x34] = OPCODE(5, NES_PC_ZPX, 0, nes_NOP, nes_z, nes_zpx),
	[0x54] = OPCODE(5, NES_PC_ZPX, 0, nes_NOP, nes_z, nes_zpx),
	[0x74] = OPCODE(5, NES_PC_ZPX, 0, nes_NOP, nes_z, nes_zpx),
	[0xd4] = OPCODE(5, NES_PC_ZPX, 0, nes_NOP, nes_z, nes_zpx),
	[0xf4] = OPCODE(5, NES_PC_ZPX, 0, nes_NOP, nes_z, nes_zpx),
	[0x09] = OPCODE(2, NES_PC_IMM, NES_ST_NZ, nes_ORA, nes_a, nes_a, nes_imm),
	[0x05] = OPCODE(3, NES_PC_ZP , NES_ST_NZ, nes_ORA, nes_a, nes_a, nes_zp ),
	[0x15] = OPCODE(4, NES_PC_ZPX, NES_ST_NZ, nes_ORA, nes_a, nes_a, nes_zpx),
	[0x01] = OPCODE(6, NES_PC_IZX, NES_ST_NZ, nes_ORA, nes_a, nes_a, nes_izx),
	[0x11] = OPCODE(5, NES_PC_IZY, NES_ST_NZ, nes_ORA, nes_a, nes_a, nes_izy),
	[0x0d] = OPCODE(4, NES_PC_ABS, NES_ST_NZ, nes_ORA, nes_a, nes_a, nes_abs),
	[0x1d] = OPCODE(4, NES_PC_ABX, NES_ST_NZ, nes_ORA, nes_a, nes_a, nes_abx),
	[0x19] = OPCODE(4, NES_PC_ABY, NES_ST_NZ, nes_ORA, nes_a, nes_a, nes_aby),

	[0x29] = OPCODE(2, NES_PC_IMM, NES_ST_NZ, nes_AND, nes_a, nes_a, nes_imm),
	[0x25] = OPCODE(3, NES_PC_ZP , NES_ST_NZ, nes_AND, nes_a, nes_a, nes_zp ),
	[0x35] = OPCODE(4, NES_PC_ZPX, NES_ST_NZ, nes_AND, nes_a, nes_a, nes_zpx),
	[0x21] = OPCODE(6, NES_PC_IZX, NES_ST_NZ, nes_AND, nes_a, nes_a, nes_izx),
	[0x31] = OPCODE(5, NES_PC_IZY, NES_ST_NZ, nes_AND, nes_a, nes_a, nes_izy),
	[0x2d] = OPCODE(4, NES_PC_ABS, NES_ST_NZ, nes_AND, nes_a, nes_a, nes_abs),
	[0x3d] = OPCODE(4, NES_PC_ABX, NES_ST_NZ, nes_AND, nes_a, nes_a, nes_abx),
	[0x39] = OPCODE(4, NES_PC_ABY, NES_ST_NZ, nes_AND, nes_a, nes_a, nes_aby),

	[0x49] = OPCODE(2, NES_PC_IMM, NES_ST_NZ, nes_XOR, nes_a, nes_a, nes_imm),
	[0x45] = OPCODE(3, NES_PC_ZP , NES_ST_NZ, nes_XOR, nes_a, nes_a, nes_zp ),
	[0x55] = OPCODE(4, NES_PC_ZPX, NES_ST_NZ, nes_XOR, nes_a, nes_a, nes_zpx),
	[0x41] = OPCODE(6, NES_PC_IZX, NES_ST_NZ, nes_XOR, nes_a, nes_a, nes_izx),
	[0x51] = OPCODE(5, NES_PC_IZY, NES_ST_NZ, nes_XOR, nes_a, nes_a, nes_izy),
	[0x4d] = OPCODE(4, NES_PC_ABS, NES_ST_NZ, nes_XOR, nes_a, nes_a, nes_abs),
	[0x5d] = OPCODE(4, NES_PC_ABX, NES_ST_NZ, nes_XOR, nes_a, nes_a, nes_abx),
	[0x59] = OPCODE(4, NES_PC_ABY, NES_ST_NZ, nes_XOR, nes_a, nes_a, nes_aby),

	[0x69] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_ADC, nes_a, nes_a, nes_imm),
	[0x65] = OPCODE(3, NES_PC_ZP , NES_ST_NZC, nes_ADC, nes_a, nes_a, nes_zp ),
	[0x75] = OPCODE(4, NES_PC_ZPX, NES_ST_NZC, nes_ADC, nes_a, nes_a, nes_zpx),
	[0x61] = OPCODE(6, NES_PC_IZX, NES_ST_NZC, nes_ADC, nes_a, nes_a, nes_izx),
	[0x71] = OPCODE(5, NES_PC_IZY, NES_ST_NZC, nes_ADC, nes_a, nes_a, nes_izy),
	[0x6d] = OPCODE(4, NES_PC_ABS, NES_ST_NZC, nes_ADC, nes_a, nes_a, nes_abs),
	[0x7d] = OPCODE(4, NES_PC_ABX, NES_ST_NZC, nes_ADC, nes_a, nes_a, nes_abx),
	[0x79] = OPCODE(4, NES_PC_ABY, NES_ST_NZC, nes_ADC, nes_a, nes_a, nes_aby),

	[0xe9] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_SBC, nes_a, nes_a, nes_imm),
	[0xe5] = OPCODE(3, NES_PC_ZP , NES_ST_NZC, nes_SBC, nes_a, nes_a, nes_zp ),
	[0xf5] = OPCODE(4, NES_PC_ZPX, NES_ST_NZC, nes_SBC, nes_a, nes_a, nes_zpx),
	[0xe1] = OPCODE(6, NES_PC_IZX, NES_ST_NZC, nes_SBC, nes_a, nes_a, nes_izx),
	[0xf1] = OPCODE(5, NES_PC_IZY, NES_ST_NZC, nes_SBC, nes_a, nes_a, nes_izy),
	[0xed] = OPCODE(4, NES_PC_ABS, NES_ST_NZC, nes_SBC, nes_a, nes_a, nes_abs),
	[0xfd] = OPCODE(4, NES_PC_ABX, NES_ST_NZC, nes_SBC, nes_a, nes_a, nes_abx),
	[0xf9] = OPCODE(4, NES_PC_ABY, NES_ST_NZC, nes_SBC, nes_a, nes_a, nes_aby),

	[0xc9] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_CMP, nes_z, nes_a, nes_imm),
	[0xc5] = OPCODE(3, NES_PC_ZP , NES_ST_NZC, nes_CMP, nes_z, nes_a, nes_zp ),
	[0xd5] = OPCODE(4, NES_PC_ZPX, NES_ST_NZC, nes_CMP, nes_z, nes_a, nes_zpx),
	[0xc1] = OPCODE(6, NES_PC_IZX, NES_ST_NZC, nes_CMP, nes_z, nes_a, nes_izx),
	[0xd1] = OPCODE(5, NES_PC_IZY, NES_ST_NZC, nes_CMP, nes_z, nes_a, nes_izy),
	[0xcd] = OPCODE(4, NES_PC_ABS, NES_ST_NZC, nes_CMP, nes_z, nes_a, nes_abs),
	[0xdd] = OPCODE(4, NES_PC_ABX, NES_ST_NZC, nes_CMP, nes_z, nes_a, nes_abx),
	[0xd9] = OPCODE(4, NES_PC_ABY, NES_ST_NZC, nes_CMP, nes_z, nes_a, nes_aby),

	[0xc0] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_CMP, nes_z, nes_y, nes_imm),
	[0xc4] = OPCODE(3, NES_PC_ZP , NES_ST_NZC, nes_CMP, nes_z, nes_y, nes_zp ),
	[0xcc] = OPCODE(4, NES_PC_ABS, NES_ST_NZC, nes_CMP, nes_z, nes_y, nes_abs),

	[0xe0] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_CMP, nes_z, nes_x, nes_imm),
	[0xe4] = OPCODE(3, NES_PC_ZP , NES_ST_NZC, nes_CMP, nes_z, nes_x, nes_zp ),
	[0xec] = OPCODE(4, NES_PC_ABS, NES_ST_NZC, nes_CMP, nes_z, nes_x, nes_abs),

	[0xc6] = OPCODE(5, NES_PC_ZP , NES_ST_NZ, nes_DEC, nes_zp , nes_zp ),
	[0xd6] = OPCODE(6, NES_PC_ZPX, NES_ST_NZ, nes_DEC, nes_zpx, nes_zpx),
	[0xce] = OPCODE(6, NES_PC_ABS, NES_ST_NZ, nes_DEC, nes_abs, nes_abs),
	[0xde] = OPCODE(7, NES_PC_ABX, NES_ST_NZ, nes_DEC, nes_abx, nes_abx),

	[0xca] = OPCODE(2, NES_PC_IMP, NES_ST_NZ, nes_DEC, nes_x , nes_x),
	[0x88] = OPCODE(2, NES_PC_IMP, NES_ST_NZ, nes_DEC, nes_y , nes_y),

	[0xe6] = OPCODE(5, NES_PC_ZP , NES_ST_NZ, nes_INC, nes_zp , nes_zp ),
	[0xf6] = OPCODE(6, NES_PC_ZPX, NES_ST_NZ, nes_INC, nes_zpx, nes_zpx),
	[0xee] = OPCODE(6, NES_PC_ABS, NES_ST_NZ, nes_INC, nes_abs, nes_abs),
	[0xfe] = OPCODE(7, NES_PC_ABX, NES_ST_NZ, nes_INC, nes_abx, nes_abx),

	[0xe8] = OPCODE(2, NES_PC_IMP, NES_ST_NZ, nes_INC, nes_x, nes_x),
	[0xc8] = OPCODE(2, NES_PC_IMP, NES_ST_NZ, nes_INC, nes_y, nes_y),

	[0x0a] = OPCODE(2, NES_PC_IMP, NES_ST_NZC, nes_ASL, nes_a  , nes_a  ),
	[0x06] = OPCODE(5, NES_PC_ZP , NES_ST_NZC, nes_ASL, nes_zp , nes_zp ),
	[0x16] = OPCODE(6, NES_PC_ZPX, NES_ST_NZC, nes_ASL, nes_zpx, nes_zpx),
	[0x0e] = OPCODE(6, NES_PC_ABS, NES_ST_NZC, nes_ASL, nes_abs, nes_abs),
	[0x1e] = OPCODE(7, NES_PC_ABX, NES_ST_NZC, nes_ASL, nes_abx, nes_abx),


	[0x2a] = OPCODE(2, NES_PC_IMP, NES_ST_NZC, nes_ROL, nes_a  , nes_a  ),
	[0x26] = OPCODE(5, NES_PC_ZP , NES_ST_NZC, nes_ROL, nes_zp , nes_zp ),
	[0x36] = OPCODE(6, NES_PC_ZPX, NES_ST_NZC, nes_ROL, nes_zpx, nes_zpx),
	[0x2e] = OPCODE(6, NES_PC_ABS, NES_ST_NZC, nes_ROL, nes_abs, nes_abs),
	[0x3e] = OPCODE(7, NES_PC_ABX, NES_ST_NZC, nes_ROL, nes_abx, nes_abx),

	[0x4a] = OPCODE(2, NES_PC_IMP, NES_ST_NZC, nes_LSR, nes_a  , nes_a  ),
	[0x46] = OPCODE(5, NES_PC_ZP , NES_ST_NZC, nes_LSR, nes_zp , nes_zp ),
	[0x56] = OPCODE(6, NES_PC_ZPX, NES_ST_NZC, nes_LSR, nes_zpx, nes_zpx),
	[0x4e] = OPCODE(6, NES_PC_ABS, NES_ST_NZC, nes_LSR, nes_abs, nes_abs),
	[0x5e] = OPCODE(7, NES_PC_ABX, NES_ST_NZC, nes_LSR, nes_abx, nes_abx),

	[0x6a] = OPCODE(2, NES_PC_IMP, NES_ST_NZC, nes_ROR, nes_a  , nes_a  ),
	[0x66] = OPCODE(5, NES_PC_ZP , NES_ST_NZC, nes_ROR, nes_zp , nes_zp ),
	[0x76] = OPCODE(6, NES_PC_ZPX, NES_ST_NZC, nes_ROR, nes_zpx, nes_zpx),
	[0x6e] = OPCODE(6, NES_PC_ABS, NES_ST_NZC, nes_ROR, nes_abs, nes_abs),
	[0x7e] = OPCODE(7, NES_PC_ABX, NES_ST_NZC, nes_ROR, nes_abx, nes_abx),

	[0xa9] = OPCODE(2, NES_PC_IMM, NES_ST_NZ, nes_LD, nes_a, nes_imm),
	[0xa5] = OPCODE(3, NES_PC_ZP , NES_ST_NZ, nes_LD, nes_a, nes_zp),
	[0xb5] = OPCODE(4, NES_PC_ZPX, NES_ST_NZ, nes_LD, nes_a, nes_zpx),
	[0xa1] = OPCODE(6, NES_PC_IZX, NES_ST_NZ, nes_LD, nes_a, nes_izx),
	[0xb1] = OPCODE(5, NES_PC_IZY, NES_ST_NZ, nes_LD, nes_a, nes_izy),
	[0xad] = OPCODE(4, NES_PC_ABS, NES_ST_NZ, nes_LD, nes_a, nes_abs),
	[0xbd] = OPCODE(4, NES_PC_ABX, NES_ST_NZ, nes_LD, nes_a, nes_abx),
	[0xb9] = OPCODE(4, NES_PC_ABY, NES_ST_NZ, nes_LD, nes_a, nes_aby),

	[0x85] = OPCODE(3, NES_PC_ZP , 0, nes_LD, nes_zp , nes_a),
	[0x95] = OPCODE(4, NES_PC_ZPX, 0, nes_LD, nes_zpx, nes_a),
	[0x81] = OPCODE(6, NES_PC_IZX, 0, nes_LD, nes_izx, nes_a),
	[0x91] = OPCODE(6, NES_PC_IZY, 0, nes_LD, nes_izy, nes_a),
	[0x8d] = OPCODE(4, NES_PC_ABS, 0, nes_LD, nes_abs, nes_a),
	[0x9d] = OPCODE(5, NES_PC_ABX, 0, nes_LD, nes_abx, nes_a),
	[0x99] = OPCODE(5, NES_PC_ABY, 0, nes_LD, nes_aby, nes_a),

	[0xa2] = OPCODE(2, NES_PC_IMM, NES_ST_NZ, nes_LD, nes_x, nes_imm),
	[0xa6] = OPCODE(3, NES_PC_ZP , NES_ST_NZ, nes_LD, nes_x, nes_zp),
	[0xb6] = OPCODE(4, NES_PC_ZPY, NES_ST_NZ, nes_LD, nes_x, nes_zpy),
	[0xae] = OPCODE(4, NES_PC_ABS, NES_ST_NZ, nes_LD, nes_x, nes_abs),
	[0xbe] = OPCODE(4, NES_PC_ABY, NES_ST_NZ, nes_LD, nes_x, nes_aby),

	[0x86] = OPCODE(3, NES_PC_ZP , 0, nes_LD, nes_zp , nes_x),
	[0x96] = OPCODE(4, NES_PC_ZPY, 0, nes_LD, nes_zpy, nes_x),
	[0x8e] = OPCODE(4, NES_PC_ABS, 0, nes_LD, nes_abs, nes_x),

	[0xa0] = OPCODE(2, NES_PC_IMM, NES_ST_NZ, nes_LD, nes_y, nes_imm),
	[0xa4] = OPCODE(3, NES_PC_ZP , NES_ST_NZ, nes_LD, nes_y, nes_zp ),
	[0xb4] = OPCODE(4, NES_PC_ZPX, NES_ST_NZ, nes_LD, nes_y, nes_zpx),
	[0xac] = OPCODE(4, NES_PC_ABS, NES_ST_NZ, nes_LD, nes_y, nes_abs),
	[0xbc] = OPCODE(4, NES_PC_ABX, NES_ST_NZ, nes_LD, nes_y, nes_abx),

	[0x84] = OPCODE(3, NES_PC_ZP , 0, nes_LD, nes_zp , nes_y),
	[0x94] = OPCODE(4, NES_PC_ZPX, 0, nes_LD, nes_zpx, nes_y),
	[0x8c] = OPCODE(4, NES_PC_ABS, 0, nes_LD, nes_abs, nes_y),

	[0xaa] = OPCODE(2, NES_PC_IMP, NES_ST_NZ, nes_LD, nes_x, nes_a),
	[0x8a] = OPCODE(2, NES_PC_IMP, NES_ST_NZ, nes_LD, nes_a, nes_x),
	[0xa8] = OPCODE(2, NES_PC_IMP, NES_ST_NZ, nes_LD, nes_y, nes_a),
	[0x98] = OPCODE(2, NES_PC_IMP, NES_ST_NZ, nes_LD, nes_a, nes_y),
	[0xba] = OPCODE(2, NES_PC_IMP, NES_ST_NZ, nes_LD, nes_x, nes_s),
	[0x9a] = OPCODE(2, NES_PC_IMP,         0, nes_LD, nes_s, nes_x),

	[0x48] = OPCODE(3, NES_PC_IMP, 0, nes_PHA, nes_z, nes_a),
	[0x68] = OPCODE(4, NES_PC_IMP, NES_ST_NZ, nes_PLA, nes_a),

	[0x08] = OPCODE(3, NES_PC_IMP, 0, nes_PHP, nes_z, nes_p),
	[0x28] = OPCODE(4, NES_PC_IMP, 0, nes_PLP, nes_p),

	[0x10] = OPCODE(2, NES_PC_REL, 0, nes_BPL, nes_z, nes_rel),
	[0x30] = OPCODE(2, NES_PC_REL, 0, nes_BMI, nes_z, nes_rel),
	[0x50] = OPCODE(2, NES_PC_REL, 0, nes_BVC, nes_z, nes_rel),
	[0x70] = OPCODE(2, NES_PC_REL, 0, nes_BVS, nes_z, nes_rel),
	[0x90] = OPCODE(2, NES_PC_REL, 0, nes_BCC, nes_z, nes_rel),
	[0xb0] = OPCODE(2, NES_PC_REL, 0, nes_BCS, nes_z, nes_rel),
	[0xd0] = OPCODE(2, NES_PC_REL, 0, nes_BNE, nes_z, nes_rel),
	[0xf0] = OPCODE(2, NES_PC_REL, 0, nes_BEQ, nes_z, nes_rel),

	[0x00] = OPCODE(7, NES_PC_IMP, 0, nes_BRK, nes_z, nes_IRQ),
	[0x40] = OPCODE(6, NES_PC_IMP, 0, nes_RTI),
	[0x20] = OPCODE(6, NES_PC_ABS, 0, nes_JSR, nes_z, nes_abs),
	[0x60] = OPCODE(6, NES_PC_IMP, 0, nes_RTS),

	[0x4c] = OPCODE(3, NES_PC_ABS, 0, nes_JMP, nes_z, nes_abs),
	[0x6c] = OPCODE(5, NES_PC_IND, 0, nes_JMP, nes_z, nes_ind),

	[0x24] = OPCODE(3, NES_PC_ZP , 0, nes_BIT, nes_z, nes_zp),
	[0x2c] = OPCODE(4, NES_PC_ABS, 0, nes_BIT, nes_z, nes_abs),

	[0x18] = OPCODE(2, NES_PC_IMP, 0, nes_CLC),
	[0x38] = OPCODE(2, NES_PC_IMP, 0, nes_SEC),

	[0xd8] = OPCODE(2, NES_PC_IMP, 0, nes_CLD),
	[0xf8] = OPCODE(2, NES_PC_IMP, 0, nes_SED),

	[0x58] = OPCODE(2, NES_PC_IMP, 0, nes_CLI),
	[0x78] = OPCODE(2, NES_PC_IMP, 0, nes_SEI),
	[0xb8] = OPCODE(2, NES_PC_IMP, 0, nes_CLV),

	[0x07] = OPCODE(5, NES_PC_ZP , NES_ST_NZC, nes_SLO, nes_a, nes_zp ),
	[0x17] = OPCODE(6, NES_PC_ZPX, NES_ST_NZC, nes_SLO, nes_a, nes_zpx),
	[0x03] = OPCODE(8, NES_PC_IZX, NES_ST_NZC, nes_SLO, nes_a, nes_izx),
	[0x13] = OPCODE(8, NES_PC_IZY, NES_ST_NZC, nes_SLO, nes_a, nes_izy),
	[0x0f] = OPCODE(6, NES_PC_ABS, NES_ST_NZC, nes_SLO, nes_a, nes_abs),
	[0x1f] = OPCODE(7, NES_PC_ABX, NES_ST_NZC, nes_SLO, nes_a, nes_abx),
	[0x1b] = OPCODE(7, NES_PC_ABY, NES_ST_NZC, nes_SLO, nes_a, nes_aby),

	[0x27] = OPCODE(5, NES_PC_ZP , NES_ST_NZC, nes_RLA, nes_a, nes_zp ),
	[0x37] = OPCODE(6, NES_PC_ZPY, NES_ST_NZC, nes_RLA, nes_a, nes_zpx),
	[0x23] = OPCODE(8, NES_PC_IZX, NES_ST_NZC, nes_RLA, nes_a, nes_izx),
	[0x33] = OPCODE(8, NES_PC_IZY, NES_ST_NZC, nes_RLA, nes_a, nes_izy),
	[0x2f] = OPCODE(6, NES_PC_ABS, NES_ST_NZC, nes_RLA, nes_a, nes_abs),
	[0x3f] = OPCODE(7, NES_PC_ABX, NES_ST_NZC, nes_RLA, nes_a, nes_abx),
	[0x3b] = OPCODE(7, NES_PC_ABY, NES_ST_NZC, nes_RLA, nes_a, nes_aby),

	[0x47] = OPCODE(5, NES_PC_ZP , NES_ST_NZC, nes_SRE, nes_a, nes_zp ),
	[0x57] = OPCODE(6, NES_PC_ZPY, NES_ST_NZC, nes_SRE, nes_a, nes_zpx),
	[0x43] = OPCODE(8, NES_PC_IZX, NES_ST_NZC, nes_SRE, nes_a, nes_izx),
	[0x53] = OPCODE(8, NES_PC_IZY, NES_ST_NZC, nes_SRE, nes_a, nes_izy),
	[0x4f] = OPCODE(6, NES_PC_ABS, NES_ST_NZC, nes_SRE, nes_a, nes_abs),
	[0x5f] = OPCODE(7, NES_PC_ABX, NES_ST_NZC, nes_SRE, nes_a, nes_abx),
	[0x5b] = OPCODE(7, NES_PC_ABY, NES_ST_NZC, nes_SRE, nes_a, nes_aby),

	[0x67] = OPCODE(5, NES_PC_ZP , NES_ST_NZC, nes_RRA, nes_a, nes_zp ),
	[0x77] = OPCODE(6, NES_PC_ZPX, NES_ST_NZC, nes_RRA, nes_a, nes_zpx),
	[0x63] = OPCODE(8, NES_PC_IZX, NES_ST_NZC, nes_RRA, nes_a, nes_izx),
	[0x73] = OPCODE(8, NES_PC_IZY, NES_ST_NZC, nes_RRA, nes_a, nes_izy),
	[0x6f] = OPCODE(6, NES_PC_ABS, NES_ST_NZC, nes_RRA, nes_a, nes_abs),
	[0x7f] = OPCODE(7, NES_PC_ABX, NES_ST_NZC, nes_RRA, nes_a, nes_abx),
	[0x7b] = OPCODE(7, NES_PC_ABY, NES_ST_NZC, nes_RRA, nes_a, nes_aby),

	[0x87] = OPCODE(3, NES_PC_ZP , 0, nes_AND, nes_zp , nes_a, nes_x),
	[0x97] = OPCODE(4, NES_PC_ZPY, 0, nes_AND, nes_zpy, nes_a, nes_x),
	[0x83] = OPCODE(6, NES_PC_IZX, 0, nes_AND, nes_izx, nes_a, nes_x),
	[0x8f] = OPCODE(4, NES_PC_ABS, 0, nes_AND, nes_abs, nes_a, nes_x),

	[0xa7] = OPCODE(3, NES_PC_ZP , NES_ST_NZ, nes_LAX, nes_a, nes_x, nes_zp),
	[0xb7] = OPCODE(4, NES_PC_ZPY, NES_ST_NZ, nes_LAX, nes_a, nes_x, nes_zpy),
	[0xa3] = OPCODE(6, NES_PC_IZX, NES_ST_NZ, nes_LAX, nes_a, nes_x, nes_izx),
	[0xb3] = OPCODE(5, NES_PC_IZY, NES_ST_NZ, nes_LAX, nes_a, nes_x, nes_izy),
	[0xaf] = OPCODE(4, NES_PC_ABS, NES_ST_NZ, nes_LAX, nes_a, nes_x, nes_abs),
	[0xbf] = OPCODE(4, NES_PC_ABY, NES_ST_NZ, nes_LAX, nes_a, nes_x, nes_aby),

	[0xc7] = OPCODE(5, NES_PC_ZP , NES_ST_NZC, nes_DCP, nes_z, nes_zp ),
	[0xd7] = OPCODE(6, NES_PC_ZPX, NES_ST_NZC, nes_DCP, nes_z, nes_zpx),
	[0xc3] = OPCODE(8, NES_PC_IZX, NES_ST_NZC, nes_DCP, nes_z, nes_izx),
	[0xd3] = OPCODE(8, NES_PC_IZY, NES_ST_NZC, nes_DCP, nes_z, nes_izy),
	[0xcf] = OPCODE(6, NES_PC_ABS, NES_ST_NZC, nes_DCP, nes_z, nes_abs),
	[0xdf] = OPCODE(7, NES_PC_ABX, NES_ST_NZC, nes_DCP, nes_z, nes_abx),
	[0xdb] = OPCODE(7, NES_PC_ABY, NES_ST_NZC, nes_DCP, nes_z, nes_aby),

	[0xe7] = OPCODE(5, NES_PC_ZP , NES_ST_NZC, nes_ISC, nes_a, nes_zp ),
	[0xf7] = OPCODE(6, NES_PC_ZPX, NES_ST_NZC, nes_ISC, nes_a, nes_zpx),
	[0xe3] = OPCODE(8, NES_PC_IZX, NES_ST_NZC, nes_ISC, nes_a, nes_izx),
	[0xf3] = OPCODE(8, NES_PC_IZY, NES_ST_NZC, nes_ISC, nes_a, nes_izy),
	[0xef] = OPCODE(6, NES_PC_ABS, NES_ST_NZC, nes_ISC, nes_a, nes_abs),
	[0xff] = OPCODE(7, NES_PC_ABX, NES_ST_NZC, nes_ISC, nes_a, nes_abx),
	[0xfb] = OPCODE(7, NES_PC_ABY, NES_ST_NZC, nes_ISC, nes_a, nes_aby),

	[0x0b] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_AND, nes_a, nes_a, nes_imm),
	[0x2b] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_AND, nes_a, nes_a, nes_imm),
	[0x4b] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_ALR, nes_a, nes_a, nes_imm),
	[0x6b] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_ALR, nes_a, nes_a, nes_imm),
	[0x8b] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_AND, nes_a, nes_x, nes_imm),
	[0xab] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_LAX, nes_a, nes_x, nes_imm),
	[0xcb] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_AXS, nes_x, nes_x, nes_imm),
	[0xeb] = OPCODE(2, NES_PC_IMM, NES_ST_NZC, nes_SBC, nes_a, nes_a, nes_imm),

	[0x93] = OPCODE(6, NES_PC_IZY,			0, nes_AHX, nes_izy, nes_a, nes_x),
	[0x9f] = OPCODE(5, NES_PC_ABY,			0, nes_AHX, nes_aby, nes_a, nes_x),
	[0x9c] = OPCODE(5, NES_PC_ABX,			0, nes_SHXY, nes_abx, nes_y),
	[0x9e] = OPCODE(5, NES_PC_ABY,			0, nes_SHXY, nes_aby, nes_x),
	[0x9b] = OPCODE(5, NES_PC_ABY,			0, nes_TAS, nes_aby),
	[0xbb] = OPCODE(4, NES_PC_ABY, NES_ST_NZ , nes_LAS, nes_a, nes_aby),
};

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

static uint8_t do_opcode(union opfunc op, uint16_t pc, uint8_t mask,
	nes_addressing adr1, nes_addressing adr2, nes_addressing adr3)
{
	uint16_t v;
	union NES_ST P;

	v = op.fnc(pc, adr1, adr2, adr3);
	P.C = !!(v & 0x100);
	P.Z = !(v & 0xff);
	P.N = !!(v & 0x80);
	cpu.P.val &= ~mask;
	cpu.P.val |= (P.val & mask);

	return v & 0xff;
}

void nes_cpu_nmi(void)
{
	nes_INT(cpu.PC, nes_NMI);
}

int nes_cpu_eval(void)
{
	uint8_t op;
	uint16_t pc = cpu.PC;
	__typeof__(opcodes[0]) *opc;

	op = nes_get_val8(pc);
	opc= opcodes + op;

	//printf("%04X  %02X A:%02X X:%02X Y:%02X S:%02X"
	//	//" %c%c%c%c%c%c%c",
	//	"\n",
	//	pc, op, cpu.A, cpu.X, cpu.Y, cpu.S
	//	//cpu.P.N?'N':' ',
	//	//cpu.P.V?'V':' ',
	//	//cpu.P.B?'B':' ',
	//	//cpu.P.D?'D':' ',
	//	//cpu.P.I?'I':' ',
	//	//cpu.P.Z?'Z':' ',
	//	//cpu.P.C?'C':' '
	//	);


	cpu.PC += opc->pc;
	nes_put_val8(opc->adrr(pc), do_opcode(opc->op, pc, opc->mask, opc->adr1, opc->adr2, opc->adr3));

	return opc->cycle;
}
