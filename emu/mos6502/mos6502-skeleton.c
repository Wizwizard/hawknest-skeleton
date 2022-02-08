#include <rc.h>
#include <base.h>
#include <membus.h>
#include <timekeeper.h>
#include <mos6502/vmcall.h>
#include <mos6502/mos6502.h>

#include <string.h>

static const uint8_t instr_cycles[256] = {
	7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	6, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
	2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
	2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
	2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
	2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
	2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
	2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
};

// below added by BigWhite

// opcode arraies

static const uint8_t LDAs[] = {
	0xA9, 0xA5, 0xB5, 0xAD, 0xBD, 0xB9, 0xA1, 0xB1, 0xFF
};
static const uint8_t LDXs[] = {
	0xA2, 0xA6, 0xB6, 0xAE, 0xBE, 0xFF
};
static const uint8_t LDYs[] = {
	0xA0, 0xA4, 0xB4, 0xAC, 0xBC, 0xFF
};
static const uint8_t STAs[] = {
	0x85, 0x95, 0x8D, 0x9D, 0x99, 0x81, 0x91, 0xFF
};
static const uint8_t STXs[] = {
	0x86, 0x96, 0x8E, 0xFF
};
static const uint8_t STYs[] = {
	0x84, 0x94, 0x8C, 0xFF
};
static const uint8_t ADCs[] = {
	0x69, 0x65, 0x75, 0x6D, 0x7D, 0x79, 0x61, 0x71, 0xFF
};
static const uint8_t SBCs[] = {
	0xE9, 0xE5, 0xF5, 0xED, 0xFD, 0xF9, 0xE1, 0xF1, 0xFF
};
static const uint8_t ANDs[] = {
	0x29, 0x25, 0x35, 0x2D, 0x3D, 0x39, 0x21, 0x31, 0xFF
};
static const uint8_t ORAs[] = {
	0x09, 0x05, 0x15, 0x0D, 0x1D, 0x19, 0x01, 0x11, 0xFF
};
static const uint8_t ASLs[] = {
	0x0A, 0x06, 0x16, 0x0E, 0x1E, 0xFF
};
static const uint8_t LSRs[] = {
	0x4A, 0x46, 0x56, 0x4E, 0x5E, 0xFF
};
static const uint8_t ROLs[] = {
	0x2A, 0x26, 0x36, 0x2E, 0x3E, 0xFF
};
static const uint8_t RORs[] = {
	0x6A, 0x66, 0x76, 0x6E, 0x7E, 0xFF
};
static const uint8_t FLAGs[] = {
	0x18, 0xD8, 0x58, 0xB8, 0x38, 0xF8, 0x78, 0xFF
};
static const uint8_t INCs[] = {
	0xE6, 0xF6, 0xEE, 0xFE, 0xE8, 0xC8 , 0xFF
};
static const uint8_t DECs[] = {
	0xC6, 0xD6, 0xCE, 0xDE, 0xCA, 0x88, 0xFF
};
static const uint8_t Trans[] = {
	0xAA, 0xA8, 0xBA, 0x8A, 0x9A, 0x98, 0xFF
};
static const uint8_t Ps[] = {
	0x48, 0x08, 0x68, 0x28, 0xFF
};
static const uint8_t JTs[] = {
	0x4C, 0x6C, 0x20, 0x60, 0xFF
};
static const uint8_t CMPs[] = {
	0xC9, 0xC5, 0xD5, 0xCD, 0xDD, 0xD9, 0xC1, 0xD1, 0xFF
};
static const uint8_t CPXs[] = {
	0xE0, 0xE4, 0xEC, 0xFF
};
static const uint8_t CPYs[] = {
	0xC0, 0xC4, 0xCC, 0xFF
};
static const uint8_t BRANCHs[] = {
	0x90, 0xB0, 0xF0, 0x30, 0xD0, 0x10, 0x50, 0x70, 0xFF
};
static const uint8_t BITs[] = {
	0x24, 0x2C, 0xFF
};
static const uint8_t EORs[] = {
	0x49, 0x45, 0x55, 0x4D, 0x5D, 0x59, 0x41, 0x51, 0xFF
};

// above added by BigWhite

static inline uint8_t
read8 (mos6502_t * cpu, uint16_t addr)
{
	return membus_read(cpu->bus, addr);
}

static inline void
write8 (mos6502_t * cpu, uint16_t addr, uint8_t val)
{
	membus_write(cpu->bus, addr, val);
}

static inline uint16_t
read16 (mos6502_t * cpu, uint16_t addr)
{
	uint16_t lo = (uint16_t)read8(cpu, addr);
	uint16_t hi = (uint16_t)read8(cpu, addr + 1);
	uint16_t val = lo | (uint16_t)(hi << 8);
	return val;
}

static inline uint16_t
buggy_read16 (mos6502_t * cpu, uint16_t addr)
{
	uint16_t first = addr;
    uint16_t msb = addr & 0xff00;
    uint16_t lsb = ((addr & 0xff) == 0xff) ? 0 : ((addr & 0xff) + 1);
	uint16_t secnd = msb | lsb;
	uint16_t lo = (uint16_t)read8(cpu, first);
	uint16_t hi = (uint16_t)read8(cpu, secnd);
	uint16_t val = (uint16_t)(hi << 8) | lo;
	return val;
}

size_t
mos6502_instr_repr (mos6502_t * cpu, uint16_t addr, char * buffer, size_t buflen)
{
	// FILL ME IN

	// Delete this line when you're done
	buffer[0] = 0;
	return 0;
}

// added below

// static funcs

static bool is_opcode_in(uint8_t opcode, const uint8_t * opcode_array)
{
	int i=0;
	
	while(opcode_array[i] != 0xff)
	{
		if(opcode == opcode_array[i])
		{
			return true;
		}
		i ++;
	}

	return false;
}

// address funcs

uint8_t immediate(mos6502_t * cpu)
{
	
	uint8_t v = read8(cpu, cpu->pc);
	cpu->pc += 1;

	return v;
}

uint16_t zeropage_address(mos6502_t * cpu)
{
	uint8_t page0_addr;
	uint16_t addr;

	page0_addr = read8(cpu, cpu->pc);
	addr = 0x00<<8 | page0_addr;
	cpu->pc += 1;

	return addr;
}

uint8_t zeropage(mos6502_t * cpu)
{
	return read8(cpu, zeropage_address(cpu));
}

uint16_t zeropage_x_address(mos6502_t * cpu)
{
	uint8_t page0_addr;
	uint16_t addr;

	page0_addr = read8(cpu, cpu->pc);
	addr = (0x00<<8|(uint8_t)(page0_addr + cpu->x));
	cpu->pc += 1;

	return addr;
}

uint8_t zeropage_x(mos6502_t * cpu)
{
	return read8(cpu, zeropage_x_address(cpu));
}

uint16_t zeropage_y_address(mos6502_t * cpu)
{
	uint8_t page0_addr;
	uint16_t addr;

	page0_addr = read8(cpu, cpu->pc);
	addr = (0x00<<8|(uint8_t)(page0_addr + cpu->y));
	cpu->pc += 1;

	return addr;
}

uint8_t zeropage_y(mos6502_t * cpu)
{
	return read8(cpu, zeropage_y_address(cpu));
}

uint16_t absolute_address(mos6502_t * cpu)
{
	uint16_t addr;

	addr = read16(cpu, cpu->pc);
	cpu->pc += 2;

	return addr;
}

uint8_t absolute(mos6502_t * cpu)
{
	return read8(cpu, absolute_address(cpu));
}

uint16_t absolute_x_address(mos6502_t * cpu)
{
	uint16_t addr;

	addr = read16(cpu, cpu->pc);
	addr = addr + cpu->x;
	cpu->pc += 2;

	return addr;
}

uint8_t absolute_x(mos6502_t * cpu)
{
	return read8(cpu, absolute_x_address(cpu));
}

uint16_t absolute_y_address(mos6502_t * cpu)
{
	uint16_t addr;

	addr = read16(cpu, cpu->pc);
	addr = addr + cpu->y;
	cpu->pc += 2;

	return addr;
}

uint8_t absolute_y(mos6502_t * cpu)
{
	return read8(cpu, absolute_y_address(cpu));
}

uint16_t indirectX_address(mos6502_t * cpu)
{
	uint8_t page0_addr;
	uint16_t addr;

	page0_addr = read8(cpu, cpu->pc);
	addr = read16(cpu, 0x00<<8|(page0_addr+cpu->x));
	cpu->pc += 1;

	return addr;
}

uint8_t indirectX(mos6502_t * cpu)
{
	return read8(cpu, indirectX_address(cpu));
}

uint16_t indirect_y_address(mos6502_t * cpu)
{
	uint8_t page0_addr;
	uint16_t addr;

	page0_addr = read8(cpu, cpu->pc);
	addr = read16(cpu, 0x00<<8|page0_addr);
	addr = addr + cpu->y;
	cpu->pc += 1;

	return addr;
}

uint8_t indirect_y(mos6502_t * cpu)
{
	return read8(cpu, indirect_y_address(cpu));
}

// flags set

void N_flag_check_set(mos6502_t * cpu, uint8_t v)
{
	// check highest bit if it is 1
	// set N = 1 if highest bit is 1 (nagative)
	if((v & 0x80) == 0x80)
	{
		cpu->p.val |= 0x80;
	} else 
	{
		cpu->p.val &= 0x7f;
	}
}

void Z_flag_check_set(mos6502_t * cpu, uint8_t v)
{
	// check if v == 0
	// set Z = 1 if v == 0
	if(v == 0x00)
	{
		cpu->p.val |= 0x02;
	} else
	{
		cpu->p.val &= 0xfd;
	}
}


void LD_flag_set(mos6502_t * cpu, uint8_t v)
{
	N_flag_check_set(cpu, v);
	Z_flag_check_set(cpu, v);
}


// opcode funcs

// LDA
mos6502_step_result_t
LDA_handler(uint8_t opcode, mos6502_t * cpu)
{

	uint8_t v;

	switch (opcode)
	{
	case 0xA9:
		// immediate
		// LDA #oper
		v = immediate(cpu);
		break;

	case 0xA5:
		// zeropage 
		// LDA oper
		v = zeropage(cpu);
		break;

	case 0xB5:
		// zerpage, X
		// LDA oper, X
		v = zeropage_x(cpu);
		break;

	case 0xAD:
		// absolute
		// LDA oper
		v = absolute(cpu);
		break;

	case 0xBD:
		// absolute, X
		// LDA oper, X
		v = absolute_x(cpu);
		break;

	case 0xB9:
		// absolute, Y
		// LDA oper, Y
		v = absolute_y(cpu);
		break;

	case 0xA1:
		// (indirect, X)
		// LDA (oper, X)
		v = indirectX(cpu);
		break;
	
	case 0xB1:
		// (indirect), Y
		// LDA (oper), Y
		v = indirect_y(cpu);
		break;

	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	LD_flag_set(cpu, v);
	cpu->a = v;

	return MOS6502_STEP_RESULT_SUCCESS;
}

// LDX
mos6502_step_result_t
LDX_handler(uint8_t opcode, mos6502_t * cpu)
{

	uint8_t v;

	switch (opcode)
	{
	case 0xA2:
		// immediate
		// LDX #oper
		v = immediate(cpu);
		break;

	case 0xA6:
		// zeropage 
		// LDX oper
		v = zeropage(cpu);
		break;

	case 0xB6:
		// zerpage, y
		// LDX oper, y
		v = zeropage_y(cpu);
		break;

	case 0xAE:
		// absolute
		// LDX oper
		v = absolute(cpu);
		break;

	case 0xBE:
		// absolute, Y
		// LDX oper, Y
		v = absolute_y(cpu);
		break;

	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	LD_flag_set(cpu, v);
	cpu->x = v;

	return MOS6502_STEP_RESULT_SUCCESS;
}

// LDY
mos6502_step_result_t
LDY_handler(uint8_t opcode, mos6502_t * cpu)
{

	uint8_t v;

	switch (opcode)
	{
	case 0xA0:
		// immediate
		v = immediate(cpu);
		break;

	case 0xA4:
		// zeropage 
		v = zeropage(cpu);
		break;

	case 0xB4:
		// zerpage, x
		v = zeropage_x(cpu);
		break;

	case 0xAC:
		// absolute
		v = absolute(cpu);
		break;

	case 0xBC:
		// absolute, X
		v = absolute_x(cpu);
		break;

	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	LD_flag_set(cpu, v);
	cpu->y = v;

	return MOS6502_STEP_RESULT_SUCCESS;
}

// STA
mos6502_step_result_t
STA_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;
	uint16_t addr;

	switch (opcode)
	{
	case 0x85:
		// zeropage
		v = read8(cpu, cpu->pc);
		addr = 0x00<<8|v;
		cpu->pc += 1;
		break;
	case 0x95:
		// zeropage, X
		v = read8(cpu, cpu->pc);
		addr = ((0x00<<8|v) + cpu->x);
		cpu->pc += 1;
		break;
	case 0x8D:
		// absolute
		addr = read16(cpu, cpu->pc);
		cpu->pc += 2;
		break;
	case 0x9D:
		// absolute, X
		addr = read16(cpu, cpu->pc);
		addr += cpu->x;
		cpu->pc += 2;
		break;
	case 0x99:
		// absolute, Y
		addr = read16(cpu, cpu->pc);
		addr += cpu->y;
		cpu->pc += 2;
		break;
	case 0x81:
		// (indirect, X)
		v = read8(cpu, cpu->pc);
		addr = read16(cpu, (0x00<<8|v)+cpu->x);
		cpu->pc += 1;
		break;
	case 0x91:
		// (indirect), Y
		v = read8(cpu, cpu->pc);
		addr = read16(cpu, (0x00<<8|v));
		addr += cpu->y;
		cpu->pc += 1;
		break;	
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	write8(cpu, addr, cpu->a);
	return MOS6502_STEP_RESULT_SUCCESS;
}

// STX
mos6502_step_result_t
STX_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint16_t addr;

	switch (opcode)
	{
	case 0x86:
		// zeropage
		addr = zeropage_address(cpu);
		break;
	case 0x96:
		// zeropage, Y
		addr = zeropage_y_address(cpu);
		break;
	case 0x8E:
		// absolute
		addr = absolute_address(cpu);
		break;	
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	write8(cpu, addr, cpu->x);
	return MOS6502_STEP_RESULT_SUCCESS;
}

// STY
mos6502_step_result_t
STY_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;
	uint16_t addr;

	switch (opcode)
	{
	case 0x84:
		// zeropage
		v = read8(cpu, cpu->pc);
		addr = 0x00<<8|v;
		cpu->pc += 1;
		break;
	case 0x94:
		// zeropage, X
		v = read8(cpu, cpu->pc);
		addr = ((0x00<<8|v) + cpu->x);
		cpu->pc += 1;
		break;
	case 0x8C:
		// absolute
		addr = read16(cpu, cpu->pc);
		cpu->pc += 2;
		break;	
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	write8(cpu, addr, cpu->y);
	return MOS6502_STEP_RESULT_SUCCESS;
}

uint8_t ADC_flags_set(mos6502_t * cpu, uint8_t v)
{
	uint8_t c = cpu->p.c == 1 ? 0x01:0x00;
	uint8_t s = cpu->a + v + c;

	N_flag_check_set(cpu, s);
	Z_flag_check_set(cpu, s);

	// Carry flag
	// C check
	if((0xff - v) < (cpu->a + c))
	{
		cpu->p.c = 1;
	} else 
	{
		cpu->p.c = 0;
	}

	// Overflow flag
	// V check
	// all nagative
	if ((v&0x80)==0x80 && (cpu->a&0x80)==0x80 && s<0x80)
	{
		cpu->p.v = 1;
	}
	// all positive 
	else if ((v&0x80)==0x00 && (cpu->a&0x80)==0x00 && s>0x7f)
	{
		cpu->p.v = 1;
	} else 
	{
		cpu->p.v = 0;
	}

	return s;
}

// ADC
mos6502_step_result_t
ADC_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;

	switch (opcode)
	{
	case 0x69:
		// immediate
		v = immediate(cpu);
		break;
	case 0x65:
		// zeropage
		v = zeropage(cpu);
		break;
	case 0x75:
		// zeropage, X
		v = zeropage_x(cpu);
		break;
	case 0x6D:
		// absolute
		v = absolute(cpu);
		break;
	case 0x7D:
		// absolute, X
		v = absolute_x(cpu);
		break;
	case 0x79:
		// absolute, Y
		v = absolute_y(cpu);
		break;
	case 0x61:
		// (indirect, X)
		v = indirectX(cpu);
		break;
	case 0x71:
		// (indirect), Y
		v = indirect_y(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	cpu->a = ADC_flags_set(cpu, v);

	return MOS6502_STEP_RESULT_SUCCESS;
}

uint8_t SBC_flags_set(mos6502_t * cpu, uint8_t v)
{
	uint8_t b = cpu->p.c == 1 ? 0x00:0x01;
	uint8_t s = cpu->a - v - b;

	N_flag_check_set(cpu, s);
	Z_flag_check_set(cpu, s);

	// Carry flag
	// C check
	if((v + b) > cpu->a)
	{
		cpu->p.c = 0;
	} else 
	{
		cpu->p.c = 1;
	}

	// Overflow flag
	// V check
	// a negative v positive
	if ((cpu->a&0x80) == 0x80 && (v&0x80) == 0x00 && s < 0x80)
	{
		cpu->p.v = 1;
	}
	// a positive v negative 
	else if ((cpu->a&0x80) == 0x00 && (v&0x80) == 0x80 && s > 0x7f)
	{
		cpu->p.v = 1;
	} else
	{
		cpu->p.v = 0;
	}

	return s;
}

// SBC
mos6502_step_result_t
SBC_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;

	switch (opcode)
	{
	case 0xE9:
		// immediate
		v = immediate(cpu);
		break;
	case 0xE5:
		// zeropage
		v = zeropage(cpu);
		break;
	case 0xF5:
		// zeropage, X
		v = zeropage_x(cpu);
		break;
	case 0xED:
		// absolute
		v = absolute(cpu);
		break;
	case 0xFD:
		// absolute, X
		v = absolute_x(cpu);
		break;
	case 0xF9:
		// absolute, Y
		v = absolute_y(cpu);
		break;
	case 0xE1:
		// (indirect, X)
		v = indirectX(cpu);
		break;
	case 0xF1:
		// (indirect), Y
		v = indirect_y(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	cpu->a = SBC_flags_set(cpu, v);

	return MOS6502_STEP_RESULT_SUCCESS;
}

// AND
mos6502_step_result_t
AND_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;

	switch (opcode)
	{
	case 0x29:
		// immediate
		v = immediate(cpu);
		break;
	case 0x25:
		// zeropage
		v = zeropage(cpu);
		break;
	case 0x35:
		// zeropage, X
		v = zeropage_x(cpu);
		break;
	case 0x2D:
		// absolute
		v = absolute(cpu);
		break;
	case 0x3D:
		// absolute, X
		v = absolute_x(cpu);
		break;
	case 0x39:
		// absolute, Y
		v = absolute_y(cpu);
		break;
	case 0x21:
		// (indirect, X)
		v = indirectX(cpu);
		break;
	case 0x31:
		// (indrect), Y
		v = indirect_y(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	uint8_t s = cpu->a & v;
	N_flag_check_set(cpu, s);
	Z_flag_check_set(cpu, s);
	cpu->a = s;

	return MOS6502_STEP_RESULT_SUCCESS;
}

// ASL
mos6502_step_result_t
ASL_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint16_t addr;
	uint8_t v;
	
	switch (opcode)
	{
	case 0x0A:
		// accumulator
		break;
	case 0x06:
		// zeropage
		addr = zeropage_address(cpu);
		break;
	case 0x16:
		// zeropage, X
		addr = zeropage_x_address(cpu);
		break;
	case 0x0e:
		// absolute
		addr = absolute_address(cpu);
		break;
	case 0x1E:
		// absolute, X
		addr = absolute_x_address(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	if (opcode == 0x0A)
		v = cpu->a;
	else
		v = read8(cpu, addr);

	// carry check
	cpu->p.c = ((v&0x80)==0x80) ? 1:0;

	v = v << 1;
	N_flag_check_set(cpu, v);
	Z_flag_check_set(cpu, v);

	// serveral checks, any way to optimize?
	if (opcode == 0x0A)
		cpu->a = v;
	else
		write8(cpu, addr, v);

	return MOS6502_STEP_RESULT_SUCCESS;
}

// LSR
mos6502_step_result_t
LSR_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint16_t addr;
	uint8_t v;
	
	switch (opcode)
	{
	case 0x4A:
		// accumulator
		break;
	case 0x46:
		// zeropage
		addr = zeropage_address(cpu);
		break;
	case 0x56:
		// zeropage, X
		addr = zeropage_x_address(cpu);
		break;
	case 0x4E:
		// absolute
		addr = absolute_address(cpu);
		break;
	case 0x5E:
		// absolute, X
		addr = absolute_x_address(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	if (opcode == 0x4A)
		v = cpu->a;
	else
		v = read8(cpu, addr);

	// carry check
	cpu->p.c = ((v&0x01)==0x01) ? 1:0;

	v = v >> 1;
	cpu->p.n = 0;
	Z_flag_check_set(cpu, v);

	// serveral checks, any way to optimize?
	if (opcode == 0x4A)
		cpu->a = v;
	else
		write8(cpu, addr, v);

	return MOS6502_STEP_RESULT_SUCCESS;
}

// ROL
mos6502_step_result_t
ROL_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint16_t addr;
	uint8_t v;
	
	switch (opcode)
	{
	case 0x2A:
		// accumulator
		break;
	case 0x26:
		// zeropage
		addr = zeropage_address(cpu);
		break;
	case 0x36:
		// zeropage, X
		addr = zeropage_x_address(cpu);
		break;
	case 0x2E:
		// absolute
		addr = absolute_address(cpu);
		break;
	case 0x3E:
		// absolute, X
		addr = absolute_x_address(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	if (opcode == 0x2A)
		v = cpu->a;
	else
		v = read8(cpu, addr);

	uint8_t c = cpu->p.c;

	// carry check
	cpu->p.c = ((v&0x80)==0x80) ? 1:0;
	
	v = v << 1;
	if(c == 1)
		v |= 0x01;
	N_flag_check_set(cpu, v);
	Z_flag_check_set(cpu, v);

	// serveral checks, any way to optimize?
	if (opcode == 0x2A)
		cpu->a = v;
	else
		write8(cpu, addr, v);

	return MOS6502_STEP_RESULT_SUCCESS;
}

// ROR
mos6502_step_result_t
ROR_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint16_t addr;
	uint8_t v;
	
	switch (opcode)
	{
	case 0x6A:
		// accumulator
		break;
	case 0x66:
		// zeropage
		addr = zeropage_address(cpu);
		break;
	case 0x76:
		// zeropage, X
		addr = zeropage_x_address(cpu);
		break;
	case 0x6E:
		// absolute
		addr = absolute_address(cpu);
		break;
	case 0x7E:
		// absolute, X
		addr = absolute_x_address(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	if (opcode == 0x6A)
		v = cpu->a;
	else
		v = read8(cpu, addr);

	uint8_t c = cpu->p.c;
	// carry check
	cpu->p.c = ((v&0x01)==0x01) ? 1:0;

	v = v >> 1;
	if(c == 1)
		v |= 0x80;
	N_flag_check_set(cpu, v);
	Z_flag_check_set(cpu, v);

	// serveral checks, any way to optimize?
	if (opcode == 0x6A)
		cpu->a = v;
	else
		write8(cpu, addr, v);

	return MOS6502_STEP_RESULT_SUCCESS;
}

mos6502_step_result_t
FLAG_handler(uint8_t opcode, mos6502_t * cpu)
{
	switch (opcode)
	{
	case 0x18:
		// CLC
		cpu->p.c = 0;
		break;
	case 0xD8:
		// CLD
		cpu->p.d = 0;
		break;
	case 0x58:
		// CLI
		cpu->p.i = 0;
		break;
	case 0xB8:
		// CLV
		cpu->p.v = 0;
		break;
	case 0x38:
		// SEC
		cpu->p.c = 1;
		break;
	case 0xF8:
		// SED
		cpu->p.d = 1;
		break;
	case 0x78:
		// SEI
		cpu->p.i = 1;
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	return MOS6502_STEP_RESULT_SUCCESS;
}

mos6502_step_result_t
ORA_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;

	switch (opcode)
	{
	case 0x09:
		// immediate
		v = immediate(cpu);
		break;
	case 0x05:
		// zeropage
		v = zeropage(cpu);
		break;
	case 0x15:
		// zeropage, X
		v = zeropage_x(cpu);
		break;
	case 0x0D:
		// absolute
		v = absolute(cpu);
		break;
	case 0x1D:
		// absolute, X
		v = absolute_x(cpu);
		break;
	case 0x19:
		// absolute, Y
		v = absolute_y(cpu);
		break;
	case 0x01:
		// (indirect, X)
		v = indirectX(cpu);
		break;
	case 0x11:
		// (indrect), Y
		v = indirect_y(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	uint8_t s = cpu->a | v;
	N_flag_check_set(cpu, s);
	Z_flag_check_set(cpu, s);
	cpu->a = s;

	return MOS6502_STEP_RESULT_SUCCESS;
}

mos6502_step_result_t
INC_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;
	uint16_t addr;

	switch (opcode)
	{
	case 0xE6:
		addr = zeropage_address(cpu);
		break;
	case 0xF6:
		addr = zeropage_x_address(cpu);
		break;
	case 0xEE:
		addr = absolute_address(cpu);
		break;
	case 0xFE:
		addr = absolute_x_address(cpu);
		break;
	case 0xE8:
		v = cpu->x;
		break;
	case 0xC8:
		v = cpu->y;
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	if(opcode != 0xE8 && opcode != 0xC8)
		v = read8(cpu, addr);
	v += 1;
	N_flag_check_set(cpu, v);
	Z_flag_check_set(cpu, v);

	if(opcode == 0xE8)
		cpu->x = v;
	else if(opcode == 0xC8)
		cpu->y = v;
	else
		write8(cpu, addr, v);

	return MOS6502_STEP_RESULT_SUCCESS;
}

mos6502_step_result_t
DEC_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;
	uint16_t addr;

	switch (opcode)
	{
	case 0xC6:
		addr = zeropage_address(cpu);
		break;
	case 0xD6:
		addr = zeropage_x_address(cpu);
		break;
	case 0xCE:
		addr = absolute_address(cpu);
		break;
	case 0xDE:
		addr = absolute_x_address(cpu);
		break;
	case 0xCA:
		v = cpu->x;
		break;
	case 0x88:
		v = cpu->y;
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	if(opcode != 0xCA && opcode != 0x88)
		v = read8(cpu, addr);
	v -= 1;
	N_flag_check_set(cpu, v);
	Z_flag_check_set(cpu, v);

	if(opcode == 0xCA)
		cpu->x = v;
	else if(opcode == 0x88)
		cpu->y = v;
	else
		write8(cpu, addr, v);

	return MOS6502_STEP_RESULT_SUCCESS;
}

// Transfer
mos6502_step_result_t
Trans_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;

	switch (opcode)
	{
	case 0xAA:
		// TAX
		v = cpu->a;
		cpu->x = v;
		break;
	case 0xA8:
		// TAY
		v = cpu->a;
		cpu->y = v;
		break;
	case 0xBA:
		// TSX
		v = cpu->sp;
		cpu->x = v;
		break;
	case 0x8A:
		// TXA
		v = cpu->x;
		cpu->a = v;
		break;
	case 0x9A:
		// TXS
		v = cpu->x;
		cpu->sp = v;
		break;
	case 0x98:
		// TYA
		v = cpu->y;
		cpu->a = v;
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	N_flag_check_set(cpu, v);
	Z_flag_check_set(cpu, v);

	return MOS6502_STEP_RESULT_SUCCESS;
}

mos6502_step_result_t
P_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;

	switch (opcode)
	{
	case 0x48:
		// PHA
		write8(cpu, 0x01<<8|cpu->sp, cpu->a);
		cpu->sp -= 1;
		break;
	case 0x08:
		// PHP
		// pushed with the break flag and bit 5 set to 1.
		v = cpu->p.val;
		v |= 0x30;
		write8(cpu, 0x01<<8|cpu->sp, v);
		cpu->sp -= 1;
		break;
	case 0x68:
		// PLA
		cpu->sp += 1;
		cpu->a = read8(cpu, 0x01<<8|cpu->sp);
		break;
	case 0x28:
		// PLA
		// pulled with the break flag and bit 5 ignored.
		cpu->sp += 1;
		v = read8(cpu, 0x01<<8|cpu->sp);
		v &= 0xcf;
		cpu->p.val = v;
		break;	
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	return MOS6502_STEP_RESULT_SUCCESS;
}

mos6502_step_result_t
JT_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint16_t addr;
	uint8_t pl;
	uint8_t ph;

	switch (opcode)
	{
	case 0x4C:
		// JMP oper
		addr = absolute_address(cpu);
		cpu->pc = addr;
		break;
	case 0x6C:
		// JMP (oper)
		addr = absolute_address(cpu);
		// addr = read16(cpu, addr);
		// buggy version
		addr = buggy_read16(cpu, addr);
		cpu->pc = addr;
		break;
	case 0x20:
		// JSR oper
		addr = absolute_address(cpu);
		// WARNING: why JSR only push PC+2???
		cpu->pc -= 1;
		pl = cpu->pc;
		ph = cpu->pc>>8;
		write8(cpu, (0x01<<8|cpu->sp), ph);
		cpu->sp -= 1;
		write8(cpu, (0x01<<8|cpu->sp), pl);
		cpu->sp -= 1;
		cpu->pc = addr;
		break;
	case 0x60:
		// RTS
		cpu->sp += 1;
		pl = read8(cpu, (0x01<<8)|cpu->sp);
		cpu->sp += 1;
		ph = read8(cpu, (0x01<<8)|cpu->sp);
		addr = ph<<8|pl;
		cpu->pc = addr + 0x0001;
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	return MOS6502_STEP_RESULT_SUCCESS;
}

mos6502_step_result_t
CMP_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;

	switch (opcode)
	{
		case 0xC9:
			v = immediate(cpu);
			break;
		case 0xC5:
			v = zeropage(cpu);
			break;
		case 0xD5:
			v = zeropage_x(cpu);
			break;
		case 0xCD:
			v = absolute(cpu);
			break;
		case 0xDD:
			v = absolute_x(cpu);
			break;
		case 0xD9:
			v = absolute_y(cpu);
			break;
		case 0xC1:
			v = indirectX(cpu);
			break;
		case 0xD1:
			v = indirect_y(cpu);
			break;
		default:
			return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	v = cpu->a - v;
	N_flag_check_set(cpu, v);
	Z_flag_check_set(cpu, v);
	cpu->p.c = v>cpu->a?0:1;

	return MOS6502_STEP_RESULT_SUCCESS;
}

// CPX
mos6502_step_result_t
CPX_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;

	switch (opcode)
	{
	case 0xE0:
		v = immediate(cpu);
		break;
	case 0xE4:
		v = zeropage(cpu);
		break;
	case 0xEC:
		v = absolute(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	v = cpu->x - v;
	N_flag_check_set(cpu, v);
	Z_flag_check_set(cpu, v);
	cpu->p.c = v>cpu->x?0:1;

	return MOS6502_STEP_RESULT_SUCCESS;
}

// Branch
mos6502_step_result_t
BRANCH_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t offset = immediate(cpu);
	uint16_t addr = offset>0x7f?(cpu->pc+offset-0xff-0x01):(cpu->pc+offset);
	switch (opcode)
	{
	case 0x90:
		if(cpu->p.c == 0)
			cpu->pc = addr;
		break;
	case 0xB0:
		if(cpu->p.c == 1)
			cpu->pc = addr;
		break;
	case 0xF0:
		if(cpu->p.z == 1)
			cpu->pc = addr;
		break;
	case 0x30:
		if(cpu->p.n == 1)
			cpu->pc = addr;
		break;
	case 0xD0:
		if(cpu->p.z == 0)
			cpu->pc = addr;
		break;
	case 0x10:
		if(cpu->p.n == 0)
			cpu->pc = addr;
		break;
	case 0x50:
		if(cpu->p.v == 0)
			cpu->pc = addr;
		break;
	case 0x70:
		if(cpu->p.v == 1)
			cpu->pc = addr;
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}
	return MOS6502_STEP_RESULT_SUCCESS;
}

// CPY
mos6502_step_result_t
CPY_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;

	switch (opcode)
	{
	case 0xC0:
		v = immediate(cpu);
		break;
	case 0xC4:
		v = zeropage(cpu);
		break;
	case 0xCC:
		v = absolute(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	v = cpu->y - v;
	N_flag_check_set(cpu, v);
	Z_flag_check_set(cpu, v);
	cpu->p.c = v>cpu->y?0:1;

	return MOS6502_STEP_RESULT_SUCCESS;
}

// BIT
mos6502_step_result_t
BIT_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;
	switch (opcode)
	{
	case 0x24:
		v = zeropage(cpu);
		break;
	case 0x2C:
		v = absolute(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}
	v = cpu->a & v;
	cpu->p.n = (v&0x80)==0x80?1:0;
	cpu->p.z = (v&0x01)==0x01?1:0;

	return MOS6502_STEP_RESULT_SUCCESS;
}

// BRK
mos6502_step_result_t
BRK_handler(mos6502_t * cpu)
{
	// brk only occupy 1 byte but next byte is for break mark
	cpu->pc += 1;

	uint8_t pl = (cpu->pc & 0x00ff);
	uint8_t ph = (cpu->pc >> 8);

	write8(cpu, (0x01<<8|cpu->sp), ph);
	cpu->sp -= 1;
	write8(cpu, (0x01<<8|cpu->sp), pl);
	cpu->sp -= 1;

	cpu->p.b = 1;
	write8(cpu, (0x01<<8|cpu->sp), cpu->p.val);
	cpu->sp -= 1;

	uint16_t irq_addr = read16(cpu, 0xfffe);
	cpu->pc = irq_addr;
	
	// what's this meaning?
	mos6502_raise_irq(cpu);

	return MOS6502_STEP_RESULT_SUCCESS;
}

// RTI
mos6502_step_result_t
RTI_handler(mos6502_t * cpu)
{
	cpu->sp += 1;
	cpu->p.val = read8(cpu, (0x01<<8|cpu->sp));
	cpu->sp += 1;
	uint8_t pl = read8(cpu, (0x01<<8|cpu->sp));
	cpu->sp += 1;
	uint8_t ph = read8(cpu, (0x01<<8|cpu->sp));
	cpu->pc = (ph<<8|pl);

	return MOS6502_STEP_RESULT_SUCCESS;
}

// EOR
mos6502_step_result_t
EOR_handler(uint8_t opcode, mos6502_t * cpu)
{
	uint8_t v;

	switch (opcode)
	{
	case 0x49:
		v = immediate(cpu);
		break;
	case 0x45:
		v = zeropage(cpu);
		break;
	case 0x55:
		v = zeropage_x(cpu);
		break;
	case 0x4D:
		v = absolute(cpu);
		break;
	case 0x5D:
		v = absolute_x(cpu);
		break;
	case 0x59:
		v = absolute_y(cpu);
		break;
	case 0x41:
		v = indirectX(cpu);
		break;
	case 0x51:
		v = indirect_y(cpu);
		break;
	default:
		return MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	cpu->a = (cpu->a^v);
	N_flag_check_set(cpu, cpu->a);
	Z_flag_check_set(cpu, cpu->a);

	return MOS6502_STEP_RESULT_SUCCESS;
}

// added above

mos6502_step_result_t
mos6502_step (mos6502_t * cpu)
{
	uint8_t opcode = read8(cpu, cpu->pc);
	// FILL ME IN

	// read opcode
	cpu->pc ++;
	
	mos6502_step_result_t step_result;	

	// how to optimize the is_opcode_in?
	// LDs
	if(is_opcode_in(opcode, LDAs))
	{
		step_result = LDA_handler(opcode, cpu);		
	} 
	else if(is_opcode_in(opcode, LDXs))
	{
		step_result = LDX_handler(opcode, cpu);
	} else if (is_opcode_in(opcode, LDYs))
	{
		step_result = LDY_handler(opcode, cpu);
	}
	// STs
	else if (is_opcode_in(opcode, STAs))
	{
		step_result = STA_handler(opcode, cpu);
	}
	else if (is_opcode_in(opcode, STXs))
	{
		step_result = STX_handler(opcode, cpu);
	} else if (is_opcode_in(opcode, STYs))
	{
		step_result = STY_handler(opcode, cpu);
	}
	// ADC
	else if (is_opcode_in(opcode, ADCs))
	{
		step_result = ADC_handler(opcode, cpu);
	}
	// SBC 
	else if (is_opcode_in(opcode, SBCs))
	{
		step_result = SBC_handler(opcode, cpu);
	}
	// AND
	else if (is_opcode_in(opcode, ANDs))
	{
		step_result = AND_handler(opcode, cpu);
	}

	// Shift Command
	// ASL
	else if (is_opcode_in(opcode, ASLs))
	{
		step_result = ASL_handler(opcode, cpu);
	}
	// LSR
	else if (is_opcode_in(opcode, LSRs))
	{
		step_result = LSR_handler(opcode, cpu);
	}
	// ROL
	else if (is_opcode_in(opcode, ROLs))
	{
		step_result = ROL_handler(opcode, cpu);
	}
	// ROR
	else if (is_opcode_in(opcode, RORs))
	{
		step_result = ROR_handler(opcode, cpu);
	}

	// set flags
	else if (is_opcode_in(opcode, FLAGs))
	{
		step_result = FLAG_handler(opcode, cpu);
	} 
	// NOP
	else if (opcode == 0xEA)
	{
		step_result = MOS6502_STEP_RESULT_SUCCESS;
	}
	// ORA
	else if (is_opcode_in(opcode, ORAs))
	{
		step_result = ORA_handler(opcode, cpu);
	}
	// INC
	else if (is_opcode_in(opcode, INCs))
	{
		step_result = INC_handler(opcode, cpu);
	}
	// DEC
	else if (is_opcode_in(opcode, DECs))
	{
		step_result = DEC_handler(opcode, cpu);
	}
	// Trans
	else if (is_opcode_in(opcode, Trans))
	{
		step_result = Trans_handler(opcode, cpu);
	}
	// Ps
	else if (is_opcode_in(opcode, Ps))
	{
		step_result = P_handler(opcode, cpu);
	}

	// JT
	else if (is_opcode_in(opcode, JTs))
	{
		step_result = JT_handler(opcode, cpu);
	}

	// CPs
	// CMP
	else if (is_opcode_in(opcode, CMPs))
	{
		step_result = CMP_handler(opcode, cpu);	
	}
	// CPX
	else if (is_opcode_in(opcode, CPXs))
	{
		step_result = CPX_handler(opcode, cpu);
	}
	// CPY
	else if (is_opcode_in(opcode, CPYs))
	{
		step_result = CPY_handler(opcode, cpu);
	}

	// Branchs
	else if (is_opcode_in(opcode, BRANCHs))
	{
		step_result = BRANCH_handler(opcode, cpu);
	}

	// BIT
	else if (is_opcode_in(opcode, BITs))
	{
		step_result = BIT_handler(opcode, cpu);
	}


	// RTI
	else if (opcode == 0x40)
	{
		step_result = RTI_handler(cpu);
	}

	// BRK
	else if (opcode == 0x00)
	{
		step_result = BRK_handler(cpu);
	}

	// EOR
	else if (is_opcode_in(opcode, EORs))
	{
		step_result = EOR_handler(opcode, cpu);
	}

	// exit
	else if (opcode == 0x80)
	{
		cpu->pc += 1;
		step_result = MOS6502_STEP_RESULT_VMBREAK;
		handle_vmcall(cpu, VMCALL_DUMP);
		handle_vmcall(cpu, VMCALL_EXIT);
	}

	else 
	{
		step_result = MOS6502_STEP_RESULT_ILLEGAL_INSTRUCTION;
	}

	mos6502_advance_clk(cpu, instr_cycles[opcode]);

	return step_result;
}
