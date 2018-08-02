#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include "usbcfg.h"

#define CCGPIO_CRL      GPIOA->CRL
#define CCGPIO_IDR      GPIOA->IDR
#define CCGPIO_ODR      GPIOA->ODR
#define CCGPIO_CRL_DIN  0x88888334
#define CCGPIO_CRL_DOUT 0x88888333
#define CCGPIO_DATA     (1<<0)
#define CCGPIO_CLOCK    (1<<1)
#define CCGPIO_RESET    (1<<2)

#define NOP __asm("mov r0, r0")
inline void wait_85ns(void) { NOP; NOP; NOP; NOP; NOP; NOP;}

inline void set_data_out(void) { CCGPIO_CRL = CCGPIO_CRL_DOUT; }
inline void set_data_in(void) { CCGPIO_CRL = CCGPIO_CRL_DIN; }

inline void set_d0_c0_r0(void) { CCGPIO_ODR = 0; }
inline void set_d1_c0_r0(void) { CCGPIO_ODR = CCGPIO_DATA; }
inline void set_d0_c1_r0(void) { CCGPIO_ODR = CCGPIO_CLOCK; }
inline void set_d1_c1_r0(void) { CCGPIO_ODR = CCGPIO_DATA | CCGPIO_CLOCK; }
inline void set_d0_c0_r1(void) { CCGPIO_ODR = CCGPIO_RESET; }
inline void set_d1_c0_r1(void) { CCGPIO_ODR = CCGPIO_DATA | CCGPIO_RESET; }
inline void set_d0_c1_r1(void) { CCGPIO_ODR = CCGPIO_CLOCK | CCGPIO_RESET; }
inline void set_d1_c1_r1(void) { CCGPIO_ODR = CCGPIO_DATA | CCGPIO_CLOCK | CCGPIO_RESET; }

inline uint32_t  get_data(void) { return CCGPIO_IDR & CCGPIO_DATA; }

/* put CC2xxx into debug mode
 *
 * I had to increase the pauses a *lot*. They should have been
 * long enough before - according to the datasheet. Success, however,
 * only occurred after raising the pauses to the values used here.
 */
void debug_init(void) {
  chSysLock();
  set_d0_c0_r1();
  set_d0_c0_r0();
  for(int i=20; i>0; i--) wait_85ns();
  set_d0_c1_r0();
  wait_85ns();
  set_d0_c0_r0();
  wait_85ns();
  set_d0_c1_r0();
  wait_85ns();
  set_d0_c0_r0();
  for(int i=10; i>0; i--) wait_85ns();
  set_d0_c0_r1();
  chSysUnlock();
  for(int i=30; i>0; i--) wait_85ns();
}

/* reset CC2xxx into "normal" mode
 */
void debug_reset(void) {
  chSysLock();
  set_d0_c0_r0();
  for(int i=20; i>0; i--) wait_85ns();
  set_d0_c0_r1();
  for(int i=30; i>0; i--) wait_85ns();
  chSysUnlock();
}

/* clock one byte out to the CC2xxx
 *
 * note that the caller should enter syslock state - this
 * is not done here.
 */
void write_debug_byte(const uint8_t b) {
  for(int i=7; i>=0; i--) {
    wait_85ns();
    if(b & (1<<i)) {
      set_d1_c1_r1();
      wait_85ns();
      set_d1_c0_r1();
    } else {
      set_d0_c1_r1();
      wait_85ns();
      set_d0_c0_r1();
    }
  }
  set_d0_c0_r1();
}

/* clock one byte in from the CC2xxx
 *
 * again, syslock should be taken by the caller.
 */
uint8_t read_debug_byte(void) {
  uint8_t b=0;
  for(int i=7; i>=0; i--) {
    b<<=1;
    set_d0_c1_r1();
    wait_85ns();
    b |= get_data();
    set_d0_c0_r1();
    wait_85ns();
  }
  return b;
}

/* wait for the CC2xxx to pull down the data line to signal that it's ready
 *
 * will clock "in" (dismiss) one byte if this does not happen.
 * returns 1 if CC2xxx signals readyness
 * returns 0 if CC2xxx does not do this after 16 retries
 */
unsigned char wait_dup_ready(void) {
  int timeout = 16;
  wait_85ns();
  int d = get_data();
  while(d && timeout--) {
    read_debug_byte(); // dummy read
    d = get_data();
  }
  if(d == 0) return 1;
  return 0;
}

/*********** (slightly adapted) TI code *********/

#include "tiflash.c"

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/* read one byte in hex notation from *line
 *
 * *length is expected to contain the number of bytes left in buffer *line
 * *line will be incremented by 2, *length will be decremented by 2
 */
int hex_byte(BaseSequentialStream *out, char **line, unsigned int *length) {
  if(*length < 2) goto error;
  int ret = 0;
  for(int i=0; i<2; i++) {
    const uint8_t c = (*line)[i];
    ret <<= 4;
    if(c >= '0' && c <= '9') {
      ret |= c - '0';
    } else if(c >= 'A' && c <= 'F') {
      ret |= c - 'A' + 10;
    } else if(c >= 'a' && c <= 'f') {
      ret |= c - 'a' + 10;
    } else {
      goto error;
    }
  }
  *line = *line + 2;
  *length = *length - 2;
  return ret;

error:
  chprintf(out, "no valid hex byte readable\r\n");
  return -1;
}

enum hexfile_state { UNINITIALIZED, INITIALIZED, ABORTED };

/* handle input lines that are considered in Intel Hexfile format, i.e.
 * firmware data
 */
void cmd_handle_hexfile(BaseSequentialStream *out, char *line, unsigned int length) {
  static enum hexfile_state state = UNINITIALIZED;
  static int lcount = 0;
  static unsigned int segm = 0;

  /* we will reuse the buffer space offered by the line buffer when we're decoding */
  unsigned char *scratch = (unsigned char*) line;
  /* stupid line counter */
  lcount++;

  /* read line length as seen by the hex file */
  int count = hex_byte(out, &line, &length);
  if(count < 0) goto error;
  const int payload_length = 4 + 2 + 2*count + 2;
  if(payload_length > length) {
    chprintf(out, "line should be %d bytes long, but I only got %d. ", payload_length+2+1, length+1);
    goto error;
  }

  /* read address 16 bit word */
  int address_high = hex_byte(out, &line, &length);
  if(address_high < 0) goto error;
  int address_low = hex_byte(out, &line, &length);
  if(address_low < 0) goto error;
  unsigned int address = (address_high << 8) | address_low;

  /* read record type byte */
  int record_type = hex_byte(out, &line, &length);
  if(record_type < 0) goto error;

  /* we calculate a checksum for the line, to be matched with the checksum we get */
  int checksum_calculated = count + address_high + address_low + record_type;

  /* read payload data */
  for(int i=0; i<count; i++) {
    int payload = hex_byte(out, &line, &length);
    if(payload < 0) {
      chprintf(out, "short/invalid (expected %d, err in %d) payload: ", count, i);
      goto error;
    }
    /* reuse line buffer as scratch buffer for parsed bytes */
    scratch[i] = payload;
    /* it's a quite stupid checksum mechanism. */
    checksum_calculated += payload;
  }

  /* read checksum byte */
  int checksum = hex_byte(out, &line, &length);
  if(checksum < 0) goto error;

  /* compare this with our own calculation */
  if(((checksum + checksum_calculated) & 0xFF) != 0) {
    chprintf(out, "bad checksum, aborting.\r\n");
    goto error;
  }

  if(record_type == 4) {
    /* upper 16 bit of address for the following lines */
    if(count != 2) {
      chprintf(out, "unrecognized format for record type 4: ");
      goto error;
    }
    segm = (scratch[0] << 8) | scratch[1];
    chprintf(out, "flashing segment starting at %x0000\r\n", segm);

  } else if(record_type == 0) {
    /* flash payload */

    if(address % 4) {
      /* the CC2xxx will only flash to 32bit word boundaries */
      int fill_bytes = 4 - (address % 4);
      chprintf(out, "warning: address not a multiple of 4, filling/prefixing!\r\n");
      for(int i=count-1; i>=0; i--) scratch[i+fill_bytes] = scratch[i];
      for(int i=0; i<fill_bytes; i++) scratch[i] = 0xFF;
      count += fill_bytes;
      address -= fill_bytes;
    }
    if(count % 4) {
      /* the CC2xxx will only flash full 32bit words */
      int fill_bytes = 4 - (count % 4);
      chprintf(out, "warning: number of bytes not a multiple of 4, filling up\r\n");
      for(int i=0; i<fill_bytes; i++) scratch[count+i] = 0xFF;
      count += fill_bytes;
    }

    if(state == UNINITIALIZED) {
      /* when starting, we need to make the device ready first.
       * This means:
       * - put it into debug mode
       * - check if we accept the device type
       * - switch CC2xxx to use external crystal as system clock source
       * - erase full flash memory
       * - enable the DMA controller while we're still in debug mode
       */
      debug_init();
      uint8_t id = read_chip_id();
      if(id != 0xB5) {
        chprintf(out, "unknown chip ID %x, aborting.\r\n", id);
        goto error;
      }
      /* switch to external crystal */
      write_xdata_memory(DUP_CLKCONCMD, 0x80);
      while (read_xdata_memory(DUP_CLKCONSTA) != 0x80);

      chprintf(out, "erasing flash...\r\n");
      chip_erase();
      chprintf(out, "done, ready to be flashed again.\r\n");

      /* Enable DMA (Disable DMA_PAUSE bit in debug configuration) */
      unsigned char debug_config = 0x22;
      debug_command(CMD_WR_CONFIG, &debug_config, 1);

      state = INITIALIZED;
    }
    if(state == INITIALIZED) {
      /* flashing data */

      /* verbose debug output */
      chprintf(out, "flashing block of %d bytes to address %x...\r\n", count, (segm << 16) | address);
      for(int i=0; i<count; i++) chprintf(out, "%02x ", scratch[i]);
      for(int i=0; i<count; i++) {
        if(scratch[i] >= 0x20 && scratch[i] < 0x7E) {
          streamPut(out, scratch[i]);
        } else {
          streamPut(out, '.');
        }
      }
      streamWrite(out, (unsigned char*)"\r\n", 2);

      write_flash_memory_block(scratch, (segm << 16) | address, count); // src, address, count
    }

  } else if(record_type == 1) {
    chprintf(out, "done, end of file, resetting!\r\n");
    goto reset_nocause;
  }
  return;

error:
  state = ABORTED;
  return;

reset_nocause:
  debug_reset();
  state = UNINITIALIZED;
  lcount = 0;
  segm = 0;
}

/* dump full flash (WIP)
 *
 * this still fails after about 0x2000 bytes. I'm a bit at loss as for why this is.
 */
void dump_flash(BaseSequentialStream *out) {
  uint8_t rbuf[16];
  for(int b=0; b<8; b++) {
    for(int a=0; a<0x8000; a+=16) {
      read_flash_memory_block(b, a, 16, rbuf);
      chprintf(out, "%x %04x  ", b, a);
      for(int i=0; i<16; i++) chprintf(out, "%02x ", rbuf[i]);
      for(int i=0; i<16; i++) {
        if(rbuf[i] >= 0x20 && rbuf[i] < 0x7E) {
          streamPut(out, rbuf[i]);
        } else {
          streamPut(out, '.');
        }
      }
      chprintf(out, "\r\n");
    }
  }
}

/* maximum input line length
 *
 * overflow will wrap
 */
#define MAX_LINE 128

/* main command parser */
void cmd_handle_line(BaseSequentialStream *out, char *line, unsigned int length) {
  if(line[0] == ':') {
    cmd_handle_hexfile(out, &line[1], length - 1);
  } else if(strncmp(line, "init", 4) == 0) {
    debug_init();
  } else if(strncmp(line, "status", 6) == 0) {
    chprintf(out, "status: %x\r\n", debug_command(CMD_READ_STATUS, NULL, 0));
  } else if(strncmp(line, "id", 2) == 0) {
    chprintf(out, "chip id: %x\r\n", read_chip_id());
  } else if(strncmp(line, "dump", 4) == 0) {
    dump_flash(out);
  } else if(strncmp(line, "reset", 5) == 0) {
    debug_reset();
  } else {
    chprintf(out, "Unknown command: %s\r\n", line);
  }
}

int main(void) {
  /* setup hal, gpios, chibios RT */
  halInit();
  set_d0_c0_r1();
  set_data_out();
  chSysInit();

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /*
   * Normal main() thread activity, spawning shells.
   */
  uint8_t linebuf[MAX_LINE];
  unsigned int p=0;
  while (true) {
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      /* toggle LED for each byte read */
      if(p&1) {
        GPIOC->ODR = 1 << GPIOC_LED;
      } else {
        GPIOC->ODR = 0;
      }
      /* read a byte */
      uint8_t c = streamGet(&SDU1);
      if(c == '\n' || c == '\r') {
        /* EOL */
        if(p > 0) {
          streamWrite(&SDU1, (const unsigned char*) "\r\n", 2);
          linebuf[p] = '\0';
          cmd_handle_line((BaseSequentialStream*) &SDU1, (char *)linebuf, p);
          p = 0;
        }
      } else {
        /* echo input */
        streamPut(&SDU1, c);
        /* add byte to line buffer */
        linebuf[p] = c;
        p = (p + 1) % MAX_LINE;
      }
    } else {
      /* disconnected */
      p = 0;
      chThdSleepMilliseconds(100);
    }
  }
}
