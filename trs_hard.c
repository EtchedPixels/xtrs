/* Copyright (c) 2000, Timothy Mann */

/* This software may be copied, modified, and used for any purpose
 * without fee, provided that (1) the above copyright notice is
 * retained, and (2) modified versions are clearly marked as having
 * been modified, with the modifier's name and the date included.  */

/* Last modified on Sun May 14 21:30:57 PDT 2000 by mann */

/*
 * Emulation of the Radio Shack TRS-80 Model I/III/4/4P
 * hard disk controller.  This is a Western Digital WD1000/WD1010
 * mapped at ports 0xc8-0xcf, plus control registers at 0xc0-0xc1.
 */

#include <errno.h>
#include "trs.h"
#include "trs_hard.h"
#include "reed.h"

/* Private types and data */

/* Structure describing one drive */
typedef struct {
  FILE* file;
  /* Values decoded from rhh */
  int writeprot;
  int cyls;  /* cyls per drive */
  int heads; /* tracks per cyl */
  int secs;  /* secs per track */
} Drive;

/* Structure describing controller state */
typedef struct {
  /* Controller register images */
  Uchar control;
  Uchar data;
  Uchar error;
  Uchar seccnt;
  Uchar secnum;
  Ushort cyl;
  Uchar drive;
  Uchar head;
  Uchar status;
  Uchar command;

  /* Number of bytes already done in current read/write */
  int bytesdone;

  /* Drive geometries and files */
  Drive d[TRS_HARD_MAXDRIVES];
} State;

static State state;

/* Forward */
static int hard_data_in();
static void hard_data_out(int value);
static void hard_restore(int cmd);
static void hard_read(int cmd);
static void hard_write(int cmd);
static void hard_verify(int cmd);
static void hard_format(int cmd);
static void hard_init(int cmd);
static void hard_seek(int cmd);
static int open_drive(int drive);
static int find_sector();
static int open_drive(int n);
static void set_dir_cyl(int cyl);

/* Powerup or reset button */
void trs_hard_init(int reset_button)
{
  int i;
  state.control = 0;
  state.data = 0;
  state.error = 0;
  state.seccnt = 0;
  state.secnum = 0;
  state.cyl = 0;
  state.drive = 0;
  state.head = 0;
  state.status = TRS_HARD_READY;
  state.command = 0;
  for (i=0; i<TRS_HARD_MAXDRIVES; i++) {
    if (state.d[i].file) {
      fclose(state.d[i].file);
      state.d[i].file = NULL;
    }
    state.d[i].writeprot = 0;
    state.d[i].cyls = 0;
    state.d[i].heads = 0;
    state.d[i].secs = 0;
  }
}

/* Read from an I/O port mapped to the controller */
int trs_hard_in(int port)
{
  switch (port) {
  case TRS_HARD_WP: {
    int i, v;
    v = 0;
    for (i=0; i<TRS_HARD_MAXDRIVES; i++) {
      open_drive(i);
      if (state.d[i].writeprot) {
	v |= TRS_HARD_WPBIT(i);
      }
    }
    return v; }
  case TRS_HARD_CONTROL:
    return state.control;
  case TRS_HARD_DATA:
    return hard_data_in();
  case TRS_HARD_ERROR:
    return state.error;
  case TRS_HARD_SECCNT:
    return state.seccnt;
  case TRS_HARD_SECNUM:
    return state.secnum;
  case TRS_HARD_CYLLO:
    return state.cyl & 0xff;
  case TRS_HARD_CYLHI:
    return (state.cyl >> 8) & 0xff;
  case TRS_HARD_SDH:
    return (state.drive << 3) | state.head;
  case TRS_HARD_STATUS:
    return state.status;
  default:
    return 0xff;
  }
}

/* Write to an I/O port mapped to the controller */
void trs_hard_out(int port, int value)
{
  switch (port) {
  case TRS_HARD_WP:
    break;
  case TRS_HARD_CONTROL:
    if (value & TRS_HARD_SOFTWARE_RESET) {
      trs_hard_init(1);
    }
    state.control = value;
    break;
  case TRS_HARD_DATA:
    hard_data_out(value);
    break;
  case TRS_HARD_PRECOMP:
    break;
  case TRS_HARD_SECCNT:
    state.seccnt = value;
    break;
  case TRS_HARD_SECNUM:
    state.secnum = value;
    break;
  case TRS_HARD_CYLLO:
    state.cyl = (state.cyl & 0xff00) | (value & 0x00ff);
    break;
  case TRS_HARD_CYLHI:
    state.cyl = (state.cyl & 0x00ff) | ((value << 8) & 0xff00);
    break;
  case TRS_HARD_SDH:
    if (value & TRS_HARD_SIZEMASK) {
      error("trs_hard: size bits set to nonzero value (%d)",
	    value & TRS_HARD_SIZEMASK);
    }
    state.drive = (value & TRS_HARD_DRIVEMASK) >> TRS_HARD_DRIVESHIFT;
    state.head = (value & TRS_HARD_HEADMASK) >> TRS_HARD_HEADSHIFT;
    break;

  case TRS_HARD_COMMAND:
    state.bytesdone = 0;
    state.command = value;
    switch (value & TRS_HARD_CMDMASK) {
    default:
      error("trs_hard: unknown command 0x%02x", value);
      break;
    case TRS_HARD_RESTORE:
      hard_restore(value);
      break;
    case TRS_HARD_READ:
      hard_read(value);
      break;
    case TRS_HARD_WRITE:
      hard_write(value);
      break;
    case TRS_HARD_VERIFY:
      hard_verify(value);
      break;
    case TRS_HARD_FORMAT:
      hard_format(value);
      break;
    case TRS_HARD_INIT:
      hard_init(value);
      break;
    case TRS_HARD_SEEK:
      hard_seek(value);
      break;
    }
    break;

  default:
    break;
  }
}

void hard_restore(int cmd)
{
  state.cyl = 0;
  /*!! should anything else be zeroed? */
  state.status = TRS_HARD_READY;
}

void hard_read(int cmd)
{
  if (cmd & TRS_HARD_DMA) {
    error("trs_hard: dma read not supported (0x%02x)", cmd);
    state.status = TRS_HARD_ERR;
    state.error = TRS_HARD_ABRTERR;
    return;
  }
  if (find_sector()) {
    state.status = TRS_HARD_READY | TRS_HARD_DRQ;
  }
}

void hard_write(int cmd)
{
  if (cmd & TRS_HARD_DMA) {
    error("trs_hard: dma write not supported (0x%02x)", cmd);
    state.status = TRS_HARD_ERR;
    state.error = TRS_HARD_ABRTERR;
    return;
  }
  if (find_sector()) {
    state.status = TRS_HARD_READY | TRS_HARD_DRQ;
  }
}

void hard_verify(int cmd)
{
  if (find_sector()) {
    state.status = TRS_HARD_READY;
  }
}

void hard_format(int cmd)
{
  if (state.seccnt != TRS_HARD_SEC_PER_TRK) {
    error("trs_hard: can only do %d sectors/track, not %d",
	  TRS_HARD_SEC_PER_TRK, state.seccnt);
  }
  if (state.secnum != TRS_HARD_SECSIZE_CODE) {
    error("trs_hard: can only do %d bytes/sectors (code %d), not code %d",
	  TRS_HARD_SECSIZE, TRS_HARD_SECSIZE_CODE, state.secnum);
  }
  /* !!should probably set up to read skew table here */
  state.status = TRS_HARD_READY;
}

void hard_init(int cmd)
{
  /* I don't know what this command does */
  error("trs_hard: init command (0x%02x) not implemented", cmd);
  state.status = TRS_HARD_READY;
}

void hard_seek(int cmd)
{
  if (find_sector()) {
    state.status = TRS_HARD_READY;
  }
}


/* 
 * 1) Make sure the file for the current drive is open.  If the file
 * cannot be opened, return 0 and set the controller error status.
 *
 * 2) If newly opening the file, establish the hardware write protect
 * status and geometry in the Drive structure.
 *
 * 3) Return 1 if all OK.
 */
int open_drive(int drive)
{
  Drive *d = &state.d[state.drive];
  char diskname[1024];
  ReedHardHeader rhh;
  size_t res;

  if (d->file != NULL) return 1;

  /* Compute the filename */
  if (trs_model == 5) {
    sprintf(diskname, "%s/hard4p-%d", trs_disk_dir, state.drive);
  } else {
    sprintf(diskname, "%s/hard%d-%d", trs_disk_dir, trs_model, state.drive);
  }

  /* First try opening for reading and writing */
  d->file = fopen(diskname, "r+");
  if (d->file == NULL) {
    /* No luck, try for reading only */
    d->file = fopen(diskname, "r");
    if (d->file == NULL) {
      goto fail;
    }
    d->writeprot = 1;
  } else {
    d->writeprot = 0;
  }

  /* Read in the Reed header and check some basic magic numbers (not all) */
  res = fread(&rhh, sizeof(rhh), 1, d->file);
  if (res != 1 || rhh.id1 != 0x56 || rhh.id2 != 0xcb || rhh.ver != 0x10) {
    error("trs_hard: unrecognized hard drive image %s", diskname);
    goto fail;
  }
  if (rhh.flag1 & 0x80) d->writeprot = 1;

  /* Use the number of cylinders specified in the header */
  d->cyls = rhh.cyl ? rhh.cyl : 256;

  /* Use the number of secs/track that RSHARD requires */
  d->secs = TRS_HARD_SEC_PER_TRK;

  /* Header gives only secs/cyl.  Compute number of heads from 
     this and the assumed number of secs/track. */
  d->heads = (rhh.sec ? rhh.sec : 256) / d->secs;

  if ((rhh.sec % d->secs) != 0 ||
      d->heads <= 0 || d->heads > TRS_HARD_MAXHEADS) {
    error("trs_hard: unusable geometry in image %s", diskname);
    goto fail;
  }

  return 1;

 fail:
  if (d->file) fclose(d->file);
  d->file = NULL;
  state.status = TRS_HARD_ERR;
  state.error = TRS_HARD_NFERR;
  return 0;
}    

/*
 * Check whether the current position is in bounds for the geometry.
 * If not, return 0 and set the controller error status.  If so, fseek
 * the file to the start of the current sector and return 1. 
 */
int find_sector()
{
  Drive *d = &state.d[state.drive];
  if (open_drive(state.drive) == 0) return 0;
  if (/**state.cyl >= d->cyls ||**/ /* ignore this limit */
      state.head >= d->heads ||
      state.secnum >= d->secs) {
    state.status = TRS_HARD_ERR;
    state.error = TRS_HARD_NFERR;
    return 0;
  }
  fseek(d->file,
	sizeof(ReedHardHeader) +
	TRS_HARD_SECSIZE * (state.cyl * d->heads * d->secs +
			    state.head * d->secs +
			    state.secnum),
	0);
  return 1;
}

int hard_data_in()
{
  Drive *d = &state.d[state.drive];
  if ((state.command & TRS_HARD_CMDMASK) == TRS_HARD_READ &&
      (state.status & TRS_HARD_ERR) == 0) {
    if (state.bytesdone < TRS_HARD_SECSIZE) {
      state.data = getc(d->file);
      state.bytesdone++;
    }
  }
  return state.data;
}

void hard_data_out(int value)
{
  Drive *d = &state.d[state.drive];
  int res = 0;
  state.data = value;
  if ((state.command & TRS_HARD_CMDMASK) == TRS_HARD_WRITE &&
      (state.status & TRS_HARD_ERR) == 0) {
    if (state.bytesdone < TRS_HARD_SECSIZE) {
      if (state.cyl == 0 && state.head == 0 &&
	  state.secnum == 0 && state.bytesdone == 2) {
	set_dir_cyl(value);
      }
      res = putc(state.data, d->file);
      state.bytesdone++;
      if (res != EOF && state.bytesdone == TRS_HARD_SECSIZE) {
	res = fflush(d->file);
      }
    }
  }
  if (res == EOF) {
    error("trs_hard: errno %d while writing drive %d", errno, state.drive);
    state.status = TRS_HARD_ERR;
    state.error = TRS_HARD_DATAERR; /* arbitrary choice */
  }
}

/* Sleazy trick to update the "directory cylinder" byte in the Reed
   header.  This value is only needed by the Reed emulator itself, and
   we would like xtrs to set it automatically so that the user doesn't
   have to know about it. */
void set_dir_cyl(int cyl)
{
  Drive *d = &state.d[state.drive];
  long where = ftell(d->file);
  fseek(d->file, 31, 0);
  putc(cyl, d->file);
  fseek(d->file, where, 0);
}