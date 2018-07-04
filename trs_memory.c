/*
 * Copyright (C) 1992 Clarendon Hill Software.
 *
 * Permission is granted to any individual or institution to use, copy,
 * or redistribute this software, provided this copyright notice is retained. 
 *
 * This software is provided "as is" without any expressed or implied
 * warranty.  If this software brings on any sort of damage -- physical,
 * monetary, emotional, or brain -- too bad.  You've got no one to blame
 * but yourself. 
 *
 * The software may be modified for your own purposes, but modified versions
 * must retain this notice.
 */

/*
   Modified by Timothy Mann, 1996 and later
   $Id$
*/

/*
 * trs_memory.c -- memory emulation functions for the TRS-80 emulator
 *
 * Routines in this file perform operations such as mem_read and mem_write,
 * and are the top level emulation points for memory-mapped devices such
 * as the screen and keyboard.
 */

#include "z80.h"
#include "trs.h"
#include <stdlib.h>
#include "trs_disk.h"
#include "trs_hard.h"

#define MAX_ROM_SIZE	(0x3800)
#define MAX_VIDEO_SIZE	(0x0800)

/* Locations for Model I, Model III, and Model 4 map 0 */
#define ROM_START	(0x0000)
#define IO_START	(0x3000)
#define KEYBOARD_START	(0x3800)
#define MORE_IO_START	(0x3c00)
#define VIDEO_START	(0x3c00)
#define RAM_START	(0x4000)
#define CASSETTE_SELECT	(0x37E4)
#define PRINTER_ADDRESS	(0x37E8)

/* Interrupt latch register in EI (Model 1) */
#define TRS_INTLATCH(addr) (((addr)&~3) == 0x37e0)

/* We allow for 2MB of banked memory via port 0x94. That is the extreme limit
   of the port mods rather than anything normal (512K might be more 'normal' */
Uchar memory[0x200001]; /* +1 so strings from mem_pointer are NUL-terminated */
Uchar *rom;
int trs_rom_size;
Uchar *video;
int trs_video_size;

int memory_map = 0;
int bank_offset[2];
#define VIDEO_PAGE_0 0
#define VIDEO_PAGE_1 1024
int video_offset = (-VIDEO_START + VIDEO_PAGE_0);
int romin = 0; /* Model 4p */
unsigned short trs_changecount = 0;
unsigned int bank_base = 0x10000;
unsigned char mem_command = 0;
int huffman_ram = 0;
int supermem = 0;
int selector = 0;
int selector_reg = 0;

/*SUPPRESS 53*/
/*SUPPRESS 112*/

void mem_video_page(int which)
{
    video_offset = -VIDEO_START + (which ? VIDEO_PAGE_1 : VIDEO_PAGE_0);
}

void mem_bank(int command)
{
    switch (command) {
      case 0:
	bank_offset[0] = 0 << 15;
	bank_offset[1] = 0 << 15;
	break;
      case 2:
	bank_offset[0] = 0 << 15;
	bank_offset[1] = 1 << 15;
	break;
      case 3:
	bank_offset[0] = 0 << 15;
	bank_offset[1] = (0 << 15) + bank_base;
	break;
      case 6:
	bank_offset[0] = (0 << 15) + bank_base;
	bank_offset[1] = 0 << 15;
	break;
      case 7:
	bank_offset[0] = (1 << 15) + bank_base;
	bank_offset[1] = 0 << 15;
	break;
      default:
	error("unknown mem_bank command %d", command);
	break;
    }
    mem_command = command;
}

/*
 *	Dave Huffman (and some other) memory expansions. These decode
 *	port 0x94 off U50 as follows
 *
 *	7: only used with Z180 board (not emulated - would need Z180 emulation!)
 *	6: write protect - not emulated
 *	5: sometimes used for > 4MHz turbo mod
 *	4-0: Bits A20-A16 of the alt bank
 *
 *	Starts as 1 on a reset so that you get the 'classic' memory map
 *
 *	This port is read-write and the drivers depend upon it
 *	(See RAMDV364.ASM)
 */

void mem_bank_base(int bits)
{
	if (huffman_ram) {
		bits &= 0x1F;
		bank_base = bits << 16;
		mem_bank(mem_command);
	}
	/* For the model 1 with the Alpha products SuperMem the top 32K
	   switches between 32K banks. We keep them at 64K+ in the array */
	if (trs_model < 4 && supermem) {
		/* Emulate a 512Kb system which is a fairly typical board.
		   256/512/768/1024K were available for the Model I and
		   more for the model III */
		bits &= 0x0F; /* 15 bits of address + 4bits logical */
		bank_base = bits << 15;
	}
}

int mem_read_bank_base(void)
{
	if (huffman_ram)
		return (bank_base >> 16) & 0x1F;
	if (trs_model < 4 && supermem)
		return bank_base >> 15;
	return 0xFF;
}

void selector_out(unsigned char value)
{
	/* Not all bits are necessarily really present but hey what
	   you can't read back you can't tell */
	selector_reg = value;
	/* Always Model 1 */
	memory_map = (trs_model << 4) + (selector_reg & 7);
	/* 0x10 is already the default tandy map we add 11-17 in the style
	   of the model 4 approach */
	if (selector_reg & 0x8) {
		/* External RAM enabled */
		/* Effectively the selector bits << 15 */
		/* Low 64K is the base memory */
		bank_base = 32768 + ((selector_reg & 0x30) << 11);
		/* Now are we mapping it high or low */
		if (selector_reg & 1) /* Low */
			bank_base += 32768;
	} else
		bank_base = 0;
}

/* Check for changes in all floppy, hard, and stringy drives. */
void
trs_change_all()
{
  trs_disk_change_all();
  trs_hard_change_all();
  stringy_change_all();
  trs_changecount++;
}

/* Handle reset button if poweron=0;
   handle hard reset or initial poweron if poweron=1 */
void trs_reset(int poweron)
{
    /* Reset devices (Model I SYSRES, Model III/4 RESET) */
    trs_cassette_reset();
    trs_timer_speed(0);
    trs_disk_reset();
    trs_hard_reset();
    stringy_reset();
    trs_change_all();

    if (trs_model == 5) {
        /* Switch in boot ROM */
	z80_out(0x9C, 1);
    }
    if (trs_model >= 4) {
        /* Turn off various memory map and video mode bits */
	z80_out(0x84, 0);
	if (huffman_ram)
		z80_out(0x94, 0);
    }
    if (trs_model >= 3) {
	grafyx_write_mode(0);
	trs_interrupt_mask_write(0);
	trs_nmi_mask_write(0);
    }
    if (trs_model == 3) {
        grafyx_m3_reset();
    }
    if (trs_model == 1) {
	hrg_onoff(0);		/* Switch off HRG1B hi-res graphics. */
	bank_base = 0;
	selector_reg = 0;
    }
    trs_kb_reset();  /* Part of keyboard stretch kludge */

    trs_cancel_event();
    trs_timer_interrupt(0);
    if (poweron || trs_model >= 4) {
        /* Reset processor */
	z80_reset();
    } else {
	/* Signal a nonmaskable interrupt. */
	trs_reset_button_interrupt(1);
	trs_schedule_event(trs_reset_button_interrupt, 0, 2000);
    }

    /*
     * Need to skip kbwait twice.  The first gets us out of the kbwait
     * loop if we were reset from inside it.  The second works around
     * the fact that z80 reset does not reset the stack pointer, so if
     * we are reset from inside a kbwait and ROM code polls the
     * keyboard before reloading the stack pointer, we will see a
     * stale keyboard wait signature on the stack and incorrectly wait
     * again.
     */
    trs_skip_next_kbwait();
    trs_skip_next_kbwait();
}

void mem_map(int which)
{
    memory_map = which + (trs_model << 4) + (romin << 2);
}

void mem_romin(int state)
{
    romin = (state & 1);
    memory_map = (memory_map & ~4) + (romin << 2);
}

void mem_init()
{
    if (trs_model <= 3) {
	rom = &memory[ROM_START];
	video = &memory[VIDEO_START];
	trs_video_size = 1024;
    } else {
	/* +1 so strings from mem_pointer are NUL-terminated */
	rom = (Uchar *) calloc(MAX_ROM_SIZE+1, 1);
	video = (Uchar *) calloc(MAX_VIDEO_SIZE+1, 1);
	trs_video_size = MAX_VIDEO_SIZE;
    }
    mem_map(0);
    mem_bank(0);
    mem_video_page(0);
    if (trs_model == 5) {
	z80_out(0x9C, 1);
    }
}

/*
 * hack to let us initialize the ROM memory
 */
void mem_write_rom(int address, int value)
{
    address &= 0xffff;

    rom[address] = value;
}

/* Called by load_hex */
void hex_data(int address, int value)
{
    mem_write_rom(address, value);
}

/* Called by load_hex */
void hex_transfer_address(int address)
{
    /* Ignore */
}

static int trs80_model1_ram(int address)
{
  int bank = 0x8000;
  int offset = address;

  /* Model 4 doesn't have banking in Model 1 mode */
  if (trs_model != 1)
    return memory[address];
  /* Selector mode 6 remaps RAM from 0000-3FFF to C000-FFFF while keeping
     the ROMs visible */
  /* selector_reg is always 0 if selector disabled so we don't need an
     extra check */
  if ((selector_reg & 7) == 6) {
    if (address >= 0xC000) {
      /* Use the low 16K, and then bank it. I'm not 100% sure how the
         PAL orders the two */
      offset &= 0x3FFF;
    }
  }
  /* Bank low on odd modes */
  if ((selector_reg & 1) == 1)
    bank = 0;
  /* Deal with 32K banking from selector or supermem */
  if ((address & 0x8000) == bank)
    offset += bank_base;
  /* A model 1 has no RAM at 0-3FFF to demux. An LNW80 can do. If we do
     LNW80 we'll need to store the ROM in its own block and change these */
  if (offset >= 0x4000)
    return memory[offset];
  else
    return 0xFF;
}

static int trs80_model1_mmio(int address)
{
  if (address >= VIDEO_START) return memory[address];
  if (address < trs_rom_size) return memory[address];
  if (address == TRSDISK_DATA) return trs_disk_data_read();
  if (TRS_INTLATCH(address)) return trs_interrupt_latch_read();
  if (address == TRSDISK_STATUS) return trs_disk_status_read();
  if (address == PRINTER_ADDRESS) return trs_printer_read();
  if (address == TRSDISK_TRACK) return trs_disk_track_read();
  if (address == TRSDISK_SECTOR) return trs_disk_sector_read();
  /* With a selector 768 bytes poke through the hole */
  if (address >= 0x3900 && selector)
    return trs80_model1_ram(address);
  if (address >= KEYBOARD_START) return trs_kb_mem_read(address);
  return 0xff;
}

int mem_read(int address)
{
    address &= 0xffff; /* allow callers to be sloppy */

    switch (memory_map) {
      case 0x10: /* Model I */
        if (address < 0x4000)
	  return trs80_model1_mmio(address & 0x3FFF);
	else
	  return trs80_model1_ram(address);
      case 0x11: /* Model 1: selector mode 1 (all RAM except I/O high */
        if (address >= 0xF7E0 && address <= 0xF7FF)
          return trs80_model1_mmio(address & 0x3FFF);
	return trs80_model1_ram(address);
      case 0x12: /* Model 1 selector mode 2 (ROM disabled) */
        if (address < 0x37E0)
          return trs80_model1_ram(address);
	if (address < 0x4000)
	  return trs80_model1_mmio(address);
	return trs80_model1_ram(address);
      case 0x13: /* Model 1: selector mode 3 (CP/M mode) */
        if (address >= 0xF7E0)
          return trs80_model1_mmio(address & 0x3FFF);
	/* Fall through */
      case 0x14: /* Model 1: All RAM banking high */
      case 0x15: /* Model 1: All RAM banking low */
	return trs80_model1_ram(address);
      case 0x16: /* Model 1: Low 16K in top 16K */
	if (address < 0x4000)
	  return trs80_model1_mmio(address);
	return trs80_model1_ram(address);
      case 0x17: /* Model 1: Described in the selector doc as 'not useful' */
        return 0xFF;	/* Not clear what really happens */

      case 0x30: /* Model III */
         /* A Model 4 using a model 1 map doesn't do this shift with
            a Supermem card */
        if (trs_model == 3 && address >= 32768)
	  return memory[address + bank_base];
	if (address >= RAM_START) return memory[address];
	if (address == PRINTER_ADDRESS)	return trs_printer_read();
	if (address < trs_rom_size) return memory[address];
	if (address >= VIDEO_START) {
	  return grafyx_m3_read_byte(address - VIDEO_START);
	}
	if (address >= KEYBOARD_START) return trs_kb_mem_read(address);
	return 0xff;

      case 0x40: /* Model 4 map 0 */
	if (address >= RAM_START) {
	    return memory[address + bank_offset[address>>15]];
	}
	if (address == PRINTER_ADDRESS) return trs_printer_read();
	if (address < trs_rom_size) return rom[address];
	if (address >= VIDEO_START) {
	    return video[address + video_offset];
	}
	if (address >= KEYBOARD_START) return trs_kb_mem_read(address);
	return 0xff;

      case 0x54: /* Model 4P map 0, boot ROM in */
      case 0x55: /* Model 4P map 1, boot ROM in */
	if (address < trs_rom_size) return rom[address];
	/* else fall thru */
      case 0x41: /* Model 4 map 1 */
      case 0x50: /* Model 4P map 0, boot ROM out */
      case 0x51: /* Model 4P map 1, boot ROM out */
	if (address >= RAM_START || address < KEYBOARD_START) {
	    return memory[address + bank_offset[address>>15]];
	}
	if (address >= VIDEO_START) {
	    return video[address + video_offset];
	}
	if (address >= KEYBOARD_START) return trs_kb_mem_read(address);
	return 0xff;

      case 0x42: /* Model 4 map 2 */
      case 0x52: /* Model 4P map 2, boot ROM out */
      case 0x56: /* Model 4P map 2, boot ROM in */
	if (address < 0xf400) {
	    return memory[address + bank_offset[address>>15]];
	}
	if (address >= 0xf800) return video[address-0xf800];
	return trs_kb_mem_read(address);

      case 0x43: /* Model 4 map 3 */
      case 0x53: /* Model 4P map 3, boot ROM out */
      case 0x57: /* Model 4P map 3, boot ROM in */
	return memory[address + bank_offset[address>>15]];

    }
    /* not reached */
    return 0xff;
}

void trs80_model1_write_mem(int address, int value)
{
  int bank = 0x8000;
  int offset = address;

  /* Model 4 doesn't have banking in Model 1 mode */
  if (trs_model != 1) {
    memory[address] = value;
    return;
  }
  /* Selector mode 6 remaps RAM from 0000-3FFF to C000-FFFF while keeping
     the ROMs visible */
  /* selector_reg is always 0 if selector disabled so we don't need an
     extra check */
  if ((selector_reg & 7) == 6) {
    if (address >= 0xC000) {
      /* We have no low 16K of RAM. This is for the LNW80 really */
      if (!(selector_reg & 8))
	return;
      /* Use the low 16K, and then bank it. I'm not 100% sure how the
         PAL orders the two */
      offset &= 0x3FFF;
    }
  }
  /* Bank low on odd modes */
  if ((selector_reg & 1) == 1)
    bank = 0;
  /* Deal with 32K banking from selector or supermem */
  if ((address & 0x8000) == bank)
    offset += bank_base;
  if (offset >= 0x4000)
    memory[offset] = value;
}

void trs80_model1_write_mmio(int address, int value)
{
  if (address >= VIDEO_START) {
    int vaddr = address + video_offset;
    /* FIXME: this should be a command line option */
#if UPPERCASE
    /*
     * Video write.  Hack here to make up for the missing bit 6
     * video ram, emulating the gate in Z30.
     */
    if (trs_model == 1) {
      if(value & 0xa0)
        value &= 0xbf;
      else
        value |= 0x40;
    }
#endif
    if (video[vaddr] != value) {
      video[vaddr] = value;
      trs_screen_write_char(vaddr, value);
    }
  } else if (address == PRINTER_ADDRESS) {
    trs_printer_write(value);
  } else if (address == CASSETTE_SELECT) {
    trs_cassette_select(value);
  } else if (address == TRSDISK_DATA) {
    trs_disk_data_write(value);
  } else if (address == TRSDISK_STATUS) {
    trs_disk_command_write(value);
  } else if (address == TRSDISK_TRACK) {
    trs_disk_track_write(value);
  } else if (address == TRSDISK_SECTOR) {
    trs_disk_sector_write(value);
  } else if (TRSDISK_SELECT(address)) {
    trs_disk_select_write(value);
  } else if (address >= 0x3900)
    trs80_model1_write_mem(address, value);
}


void mem_write(int address, int value)
{
    address &= 0xffff;

    switch (memory_map) {
      case 0x10: /* Model I */
        /* A Model 4 using a model 1 map doesn't do this shift with
           a Supermem card */
        if (address >= RAM_START)
          trs80_model1_write_mem(address, value);
	else
	  trs80_model1_write_mmio(address, value);
	break;
      case 0x11: /* Model 1: selector mode 1 (all RAM except I/O high */
        if (address >= 0xF7E0 && address <= 0xF7FF)
          trs80_model1_write_mmio(address & 0x3FFF, value);
	else
	  trs80_model1_write_mem(address, value);
	break;
      case 0x12: /* Model 1 selector mode 2 (ROM disabled) */
        if (address < 0x37E0)
          trs80_model1_write_mem(address, value);
	else if (address < 0x4000)
	  trs80_model1_write_mmio(address, value);
	else
	  trs80_model1_write_mem(address, value);
	break;
      case 0x13: /* Model 1: selector mode 3 (CP/M mode) */
        if (address >= 0xF7E0)
          trs80_model1_write_mmio(address & 0x3FFF, value);
	/* Fall through */
      case 0x14: /* Model 1: All RAM banking high */
      case 0x15: /* Model 1: All RAM banking low */
	trs80_model1_write_mem(address, value);
	break;
      case 0x16: /* Model 1: Low 16K in top 16K */
	if (address < 0x4000)
	  trs80_model1_write_mmio(address, value);
	else
	  trs80_model1_ram(address);
      case 0x17: /* Model 1: Described in the selector doc as 'not useful' */
        break;	/* Not clear what really happens */

      case 0x30: /* Model III */
        /* A Model 4 using a model 1 map doesn't do this shift with
           a Supermem card */
        if (trs_model == 3 && address >= 32768)
            memory[address + bank_base] = value;
	else if (address >= RAM_START) {
	    memory[address] = value;
	} else if (address >= VIDEO_START) {
	    int vaddr = address + video_offset;
	    if (grafyx_m3_write_byte(vaddr, value)) return;
	    if (video[vaddr] != value) {
	      video[vaddr] = value;
	      trs_screen_write_char(vaddr, value);
	    }
	} else if (address == PRINTER_ADDRESS) {
	    trs_printer_write(value);
	}
	break;

      case 0x40: /* Model 4 map 0 */
      case 0x50: /* Model 4P map 0, boot ROM out */
      case 0x54: /* Model 4P map 0, boot ROM in */
	if (address >= RAM_START) {
	    memory[address + bank_offset[address>>15]] = value;
	} else if (address >= VIDEO_START) {
	    int vaddr = address+ video_offset;
	    if (video[vaddr] != value) {
		video[vaddr] = value;
		trs_screen_write_char(vaddr, value);
	    }
	} else if (address == PRINTER_ADDRESS) {
	    trs_printer_write(value);
	}
	break;

      case 0x41: /* Model 4 map 1 */
      case 0x51: /* Model 4P map 1, boot ROM out */
      case 0x55: /* Model 4P map 1, boot ROM in */
	if (address >= RAM_START || address < KEYBOARD_START) {
	    memory[address + bank_offset[address>>15]] = value;
	} else if (address >= VIDEO_START) {
	    int vaddr = address + video_offset;
	    if (video[vaddr] != value) {
		video[vaddr] = value;
		trs_screen_write_char(vaddr, value);
	    }
	}
	break;

      case 0x42: /* Model 4 map 2 */
      case 0x52: /* Model 4P map 2, boot ROM out */
      case 0x56: /* Model 4P map 2, boot ROM in */
	if (address < 0xf400) {
	    memory[address + bank_offset[address>>15]] = value;
	} else if (address >= 0xf800) {
	    int vaddr = address - 0xf800;
	    if (video[vaddr] != value) {
		video[vaddr] = value;
		trs_screen_write_char(vaddr, value);
	    }
	}
	break;

      case 0x43: /* Model 4 map 3 */
      case 0x53: /* Model 4P map 3, boot ROM out */
      case 0x57: /* Model 4P map 3, boot ROM in */
	memory[address + bank_offset[address>>15]] = value;
	break;
    }
}

/*
 * Words are stored with the low-order byte in the lower address.
 */
int mem_read_word(int address)
{
    int rval;

    rval = mem_read(address++);
    rval |= mem_read(address & 0xffff) << 8;
    return rval;
}

void mem_write_word(int address, int value)
{
    mem_write(address++, value & 0xff);
    mem_write(address, value >> 8);
}

static Uchar *trs80_model1_ram_addr(int address)
{
  int bank = 0x8000;
  int offset = address;

  /* Model 4 doesn't have banking in Model 1 mode */
  if (trs_model != 1)
    return memory + address;
  /* Selector mode 6 remaps RAM from 0000-3FFF to C000-FFFF while keeping
     the ROMs visible */
  /* selector_reg is always 0 if selector disabled so we don't need an
     extra check */
  if ((selector_reg & 7) == 6) {
    if (address >= 0xC000) {
      /* We have no low 16K of RAM. This is for the LNW80 really */
      if (!(selector_reg & 8))
	return NULL;
      /* Use the low 16K, and then bank it. I'm not 100% sure how the
         PAL orders the two */
      offset &= 0x3FFF;
    }
  }
  /* Bank low on odd modes */
  if ((selector_reg & 1) == 1)
    bank = 0;
  /* Deal with 32K banking from selector or supermem */
  if ((address & 0x8000) == bank)
    offset += bank_base;
  if (offset < 0x4000)
    return NULL;
  return memory + offset;
}

static Uchar *trs80_model1_mmio_addr(int address, int writing)
{
  if (address >= VIDEO_START) return memory + address;
  if (address < trs_rom_size && !writing) return memory + address;
  /* With a selector 768 bytes poke through the hole */
  if (address >= 0x3900 && selector)
    return trs80_model1_ram_addr(address);
  return NULL;
}

/*
 * Get a pointer to the given address.  Note that there is no checking
 * whether the next virtual address is physically contiguous.  The
 * caller is responsible for making sure his strings don't span 
 * memory map boundaries.
 *
 * Needs to die...
 */
Uchar *mem_pointer(int address, int writing)
{
    address &= 0xffff;

    switch (memory_map + (writing << 3)) {
      case 0x10: /* Model I */
      case 0x18:
        if (address < 0x4000)
	  return trs80_model1_mmio_addr(address & 0x3FFF, writing);
	else
	  return trs80_model1_ram_addr(address);
      case 0x11: /* Model 1: selector mode 1 (all RAM except I/O high */
      case 0x19:
	return trs80_model1_ram_addr(address);
      case 0x12: /* Model 1 selector mode 2 (ROM disabled) */
        if (address < 0x37E0)
          return trs80_model1_ram_addr(address);
	if (address < 0x4000)
	  return trs80_model1_mmio_addr(address, writing);
	return trs80_model1_ram_addr(address);
      case 0x13: /* Model 1: selector mode 3 (CP/M mode) */
        if (address >= 0xF7E0)
          return trs80_model1_mmio_addr(address & 0x3FFF, writing);
	/* Fall through */
      case 0x14: /* Model 1: All RAM banking high */
      case 0x15: /* Model 1: All RAM banking low */
	return trs80_model1_ram_addr(address);
      case 0x16: /* Model 1: Low 16K in top 16K */
	if (address < 0x4000)
	  return trs80_model1_mmio_addr(address, writing);
	return trs80_model1_ram_addr(address);
      case 0x17: /* Model 1: Described in the selector doc as 'not useful' */        return NULL;	/* Not clear what really happens */
        return NULL;
      case 0x30: /* Model III reading */
        if (trs_model < 4 && address >= 32768)
	    return &memory[address + bank_base];
	if (address >= VIDEO_START) return &memory[address];
	if (address < trs_rom_size) return &rom[address];
	return NULL;

      case 0x38: /* Model III writing */
	if (address >= VIDEO_START) return &memory[address];
	return NULL;

      case 0x40: /* Model 4 map 0 reading */
	if (address >= RAM_START) {
	    return &memory[address + bank_offset[address>>15]];
	}
	if (address < trs_rom_size) return &rom[address];
	if (address >= VIDEO_START) {
	    return &video[address + video_offset];
	}
	return NULL;

      case 0x48: /* Model 4 map 0 writing */
      case 0x58: /* Model 4P map 0, boot ROM out, writing */
      case 0x5c: /* Model 4P map 0, boot ROM in, writing */
	if (address >= RAM_START) {
	    return &memory[address + bank_offset[address>>15]];
	}
	if (address >= VIDEO_START) {
	    return &video[address + video_offset];
	}
	return NULL;

      case 0x54: /* Model 4P map 0, boot ROM in, reading */
      case 0x55: /* Model 4P map 1, boot ROM in, reading */
	if (address < trs_rom_size) return &rom[address];
	/* else fall thru */
      case 0x41: /* Model 4 map 1 reading */
      case 0x49: /* Model 4 map 1 writing */
      case 0x50: /* Model 4P map 0, boot ROM out, reading */
      case 0x51: /* Model 4P map 1, boot ROM out, reading */
      case 0x59: /* Model 4P map 1, boot ROM out, writing */
      case 0x5d: /* Model 4P map 1, boot ROM in, writing */
	if (address >= RAM_START || address < KEYBOARD_START) {
	    return &memory[address + bank_offset[address>>15]];
	}
	if (address >= VIDEO_START) {
	    return &video[address + video_offset];
	}
	return NULL;

      case 0x42: /* Model 4 map 1, reading */
      case 0x4a: /* Model 4 map 1, writing */
      case 0x52: /* Model 4P map 2, boot ROM out, reading */
      case 0x5a: /* Model 4P map 2, boot ROM out, writing */
      case 0x56: /* Model 4P map 2, boot ROM in, reading */
      case 0x5e: /* Model 4P map 2, boot ROM in, writing */
	if (address < 0xf400) {
	    return &memory[address + bank_offset[address>>15]];
	}
	if (address >= 0xf800) return &video[address-0xf800];
	return NULL;

      case 0x43: /* Model 4 map 3, reading */
      case 0x4b: /* Model 4 map 3, writing */
      case 0x53: /* Model 4P map 3, boot ROM out, reading */
      case 0x5b: /* Model 4P map 3, boot ROM out, writing */
      case 0x57: /* Model 4P map 3, boot ROM in, reading */
      case 0x5f: /* Model 4P map 3, boot ROM in, writing */
	return &memory[address + bank_offset[address>>15]];
    }
    /* not reached */
    return NULL;
}

/*
 * Block move instructions, for LDIR and LDDR instructions.
 *
 * Direction is either +1 or -1.  
 *
 * Note that a count of zero => move 64K bytes.
 *
 * These can be special cased to do fun stuff like fast
 * video scrolling.
 */
int
mem_block_transfer(Ushort dest, Ushort source, int direction, Ushort count)
{
    int ret;
    /* special case for screen scroll */
    if(0 && (trs_model <= 3 || (memory_map & 3) < 2) &&
       (dest == VIDEO_START) && (source == VIDEO_START + 0x40) &&
       (count == 0x3c0) && (direction > 0) && !grafyx_m3_active())
    {
	/* scroll screen one line */
        unsigned char *p = video, *q = video + 0x40;
	trs_screen_scroll();
	do { *p++ = ret = *q++; } while (count--);
    }
    else
    {
	if(direction > 0)
	{
	    do
	    {
		mem_write(dest++, ret = mem_read(source++));
		count--;
	    }
	    while(count);
	}
	else
	{
	    do
	    {
		mem_write(dest--, ret = mem_read(source--));
		count--;
	    }
	    while(count);
	}

    }
    return ret;
}
