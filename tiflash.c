//*****************************************************************************
//! @file       main.c
//! @brief      Example programming a CC2530/31/33/40/41/43/44/45 (DUP) using a
//!             CC2530 (programmer). DUP is used as an abbreviation for Device
//!             Under Programming.
//!
//!             Programmer: SmartRF05EB + CC2530EM
//!             DUP       : SmartRF05EB + CC2530/31/33EM,
//!                                     + CC2540/41/43/44/45EM
//!
//!             Project is built using IAR EW8051 8.11.2.
//!
//!             Note that the primary focus of this example is to demonstrate
//!             the flash programming algorithm. Thus, the code is not
//!             optimized for speed.
//!
//!             Programmer I/O used in this example:
//!             P0[0] as Debug Data (DD)
//!             P0[1] as Debug Clock (DC)
//!             P0[7] as RESET_N
//!
//!             P0[7:1] are always output. P0[0] changes direction. P0[6:2]
//!             are configured as logic 0.
//!
//! Revised     $Date: 2012-09-04 10:31:41 +0200 (ti, 04 sep 2012) $
//! Revision    $Revision: 23431 $
//
//  Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/


/******************************************************************************
* INCLUDES
*/
//#include <ioCC2530.h>
#define FCTL            0x6270


/******************************************************************************
* DEFINES
*/
// Programmer data line bitmasks (programmer I/O port 0)
//#define DD                          0x01 // P0.0
//#define DC                          0x02 // P0.1
//#define RESET_N                     0x80 // P0.7

// Start addresses on DUP (Increased buffer size improves performance)
#define ADDR_BUF0                   0x0000 // Buffer (512 bytes)
#define ADDR_DMA_DESC_0             0x0200 // DMA descriptors (8 bytes)
#define ADDR_DMA_DESC_1             (ADDR_DMA_DESC_0 + 8)

// DMA channels used on DUP
#define CH_DBG_TO_BUF0              0x01   // Channel 0
#define CH_BUF0_TO_FLASH            0x02   // Channel 1

// Debug commands
#define CMD_CHIP_ERASE              0x10
#define CMD_WR_CONFIG               0x19
#define CMD_RD_CONFIG               0x24
#define CMD_READ_STATUS             0x30
#define CMD_RESUME                  0x4C
#define CMD_DEBUG_INSTR_1B          (0x54|1)
#define CMD_DEBUG_INSTR_2B          (0x54|2)
#define CMD_DEBUG_INSTR_3B          (0x54|3)
#define CMD_BURST_WRITE             0x80
#define CMD_GET_CHIP_ID             0x68

// Debug status bitmasks
#define STATUS_CHIP_ERASE_BUSY_BM   0x80 // New debug interface
#define STATUS_PCON_IDLE_BM         0x40
#define STATUS_CPU_HALTED_BM        0x20
#define STATUS_PM_ACTIVE_BM         0x10
#define STATUS_HALT_STATUS_BM       0x08
#define STATUS_DEBUG_LOCKED_BM      0x04
#define STATUS_OSC_STABLE_BM        0x02
#define STATUS_STACK_OVERFLOW_BM    0x01

// DUP registers (XDATA space address)
#define DUP_DBGDATA                 0x6260  // Debug interface data buffer
#define DUP_FCTL                    0x6270  // Flash controller
#define DUP_FADDRL                  0x6271  // Flash controller addr
#define DUP_FADDRH                  0x6272  // Flash controller addr
#define DUP_FWDATA                  0x6273  // Clash controller data buffer
#define DUP_CLKCONSTA               0x709E  // Sys clock status
#define DUP_CLKCONCMD               0x70C6  // Sys clock configuration
#define DUP_MEMCTR                  0x70C7  // Flash bank xdata mapping
#define DUP_DMA1CFGL                0x70D2  // Low byte, DMA config ch. 1
#define DUP_DMA1CFGH                0x70D3  // Hi byte , DMA config ch. 1
#define DUP_DMA0CFGL                0x70D4  // Low byte, DMA config ch. 0
#define DUP_DMA0CFGH                0x70D5  // Low byte, DMA config ch. 0
#define DUP_DMAARM                  0x70D6  // DMA arming register

// Utility macros
//! Set programmer DD line as input
#define SET_DD_INPUT()      set_data_in()
//! Set programmer DD line as output
#define SET_DD_OUTPUT()     set_data_out()
//! Low nibble of 16bit variable
#define LOBYTE(w)           ((unsigned char)(w))
//! High nibble of 16bit variable
#define HIBYTE(w)           ((unsigned char)(((unsigned short)(w) >> 8) & 0xFF))


/******************************************************************************
* VARIABLES
*/
//! DUP DMA descriptor
const unsigned char dma_desc_0[8] =
{
    // Debug Interface -> Buffer
    HIBYTE(DUP_DBGDATA),            // src[15:8]
    LOBYTE(DUP_DBGDATA),            // src[7:0]
    HIBYTE(ADDR_BUF0),              // dest[15:8]
    LOBYTE(ADDR_BUF0),              // dest[7:0]
    0,                              // len[12:8] - filled in later
    0,                              // len[7:0]
    31,                             // trigger: DBG_BW
    0x11                            // increment destination
};
//! DUP DMA descriptor
const unsigned char dma_desc_1[8] =
{
    // Buffer -> Flash controller
    HIBYTE(ADDR_BUF0),              // src[15:8]
    LOBYTE(ADDR_BUF0),              // src[7:0]
    HIBYTE(DUP_FWDATA),             // dest[15:8]
    LOBYTE(DUP_FWDATA),             // dest[7:0]
    0,                              // len[12:8] - filled in later
    0,                              // len[7:0]
    18,                             // trigger: FLASH
    0x42,                           // increment source
};
#if 0
static unsigned char write_data[4] = {0x55, 0xAA, 0x55, 0xAA};
static unsigned char read_data[4];


/******************************************************************************
* FUNCTIONS
*/
/**************************************************************************//**
* @brief    Initializes the programmer by switching to 32 MHz XOSC and
*           configuring I/O.
*
* @param    data    Byte to write
*
* @return   None.
******************************************************************************/
/*
void programmer_init(void)
{
    // Switch programmer (a CC2530) to 32 MHz XOSC for max performance
    CLKCONCMD = 0x80;
    while (CLKCONSTA != 0x80);

    // Set P0[6:0] as output low, P0[7] (RESET_N) as output high
    P0 = RESET_N;
    P0DIR = 0xFF;
}
*/

/**************************************************************************//**
* @brief    Writes a byte on the debug interface. Requires DD to be
*           output when function is called.
*
* @param    data    Byte to write
*
* @return   None.
******************************************************************************/
#pragma inline
void write_debug_byte(unsigned char data)
{
    unsigned char i;
    for (i = 0; i < 8; i++)
    {
        // Set clock high and put data on DD line
        P0 = (data & 0x80) ? (RESET_N|DC|DD) : (RESET_N|DC);
        data <<= 1;
        P0 &= ~DC; // set clock low (DUP capture flank)
    }
}

/**************************************************************************//**
* @brief    Reads a byte from the debug interface. Requires DD to be
*           input when function is called.
*
* @return   Returns the byte read.
******************************************************************************/
#pragma inline
unsigned char read_debug_byte(void)
{
    unsigned char i;
    unsigned char data;
    for (i = 0; i < 8; i++)
    {
        P0 = (RESET_N|DC);  // DC high
        data <<= 1;
        data |= (P0 & DD);  // Read DD line
        P0 = (RESET_N);     // DC low
    }
    return data;
}

/**************************************************************************//**
* @brief    Function waits for DUP to indicate that it is ready. The DUP will
*           pulls DD line low when it is ready. Requires DD to be input when
*           function is called.
*
* @return   Returns 0 if function timed out waiting for DD line to go low
* @return   Returns 1 when DUP has indicated it is ready.
******************************************************************************/
#pragma inline
unsigned char wait_dup_ready(void)
{
    // DUP pulls DD low when ready
    unsigned char count = 0;
    while (P0 & DD && count < 16)
    {
        // Clock out 8 bits before checking if DD is low again
        read_debug_byte();
        count++;
    }
    return (count == 16) ? 0 : 1;
}
#endif

/**************************************************************************//**
* @brief    Issues a command on the debug interface. Only commands that return
*           one output byte are supported.
*
* @param    cmd             Command byte
* @param    cmd_bytes       Pointer to the array of data bytes following the
*                           command byte [0-3]
* @param    num_cmd_bytes   The number of data bytes (input to DUP) [0-3]
*
* @return   Data returned by command
******************************************************************************/
unsigned char debug_command(unsigned char cmd, unsigned char *cmd_bytes,
                            unsigned short num_cmd_bytes)
{
    unsigned short i;
    unsigned char output = 0;

    chSysLock();
    // Make sure DD is output
    SET_DD_OUTPUT();

    // Send command
    write_debug_byte(cmd);

    // Send bytes
    for (i = 0; i < num_cmd_bytes; i++)
    {
        write_debug_byte(cmd_bytes[i]);
    }

    // Set DD as input
    SET_DD_INPUT();

    // Wait for data to be ready
    wait_dup_ready();

    // Read returned byte
    output = read_debug_byte();

    // Set DD as output
    SET_DD_OUTPUT();
    chSysUnlock();

    return output;
}


/**************************************************************************//**
* @brief    Resets the DUP into debug mode. Function assumes that
*           the programmer I/O has already been configured using e.g.
*           programmer_init().
*
* @return   None.
******************************************************************************/
#if 0
void debug_init(void)
{
    volatile unsigned char i;

    // Send two flanks on DC while keeping RESET_N low
    P0 = 0x00;                  // All low (incl. RESET_N)
    for (i = 0; i < 10; i++);   // Wait
    P0 = DC;                    // DC high
    P0 = 0x00;                  // DC low
    P0 = DC;                    // DC high
    P0 = 0x00;                  // DC low
    for (i = 0; i < 10; i++);   // Wait
    P0 = RESET_N;               // Release RESET_N
}
#endif


/**************************************************************************//**
* @brief    Reads the chip ID over the debug interface using the
*           GET_CHIP_ID command.
*
* @return   Returns the chip id returned by the DUP
******************************************************************************/
unsigned char read_chip_id(void)
{
    unsigned char id = 0;

    chSysLock();
    // Make sure DD is output
    SET_DD_OUTPUT();

    // Send command
    write_debug_byte(CMD_GET_CHIP_ID);

    // Set DD as input
    SET_DD_INPUT();

    // Wait for data to be ready
    wait_dup_ready();

    // Read ID and revision
    id = read_debug_byte(); // ID
    read_debug_byte();      // Revision (discard)

    // Set DD as output
    SET_DD_OUTPUT();
    chSysUnlock();

    return id;
}

/**************************************************************************//**
* @brief    Sends a block of data over the debug interface using the
*           BURST_WRITE command.
*
* @param    src         Pointer to the array of input bytes
* @param    num_bytes   The number of input bytes
*
* @return   None.
******************************************************************************/
void burst_write_block(unsigned char *src, unsigned short num_bytes)
{
    unsigned short i;

    chSysLock();
    // Make sure DD is output
    SET_DD_OUTPUT();

    write_debug_byte(CMD_BURST_WRITE | HIBYTE(num_bytes));
    write_debug_byte(LOBYTE(num_bytes));
    for (i = 0; i < num_bytes; i++)
    {
        write_debug_byte(src[i]);
    }

    // Set DD as input
    SET_DD_INPUT();

    // Wait for DUP to be ready
    wait_dup_ready();

    read_debug_byte(); // ignore output

    // Set DD as output
    SET_DD_OUTPUT();
    chSysUnlock();
}


/**************************************************************************//**
* @brief    Issues a CHIP_ERASE command on the debug interface and waits for it
*           to complete.
*
* @return   None.
******************************************************************************/
void chip_erase(void)
{
    volatile unsigned char status;
    // Send command
    debug_command(CMD_CHIP_ERASE, 0, 0);

    // Wait for status bit 7 to go low
    do {
        status = debug_command(CMD_READ_STATUS, 0, 0);
    } while((status & STATUS_CHIP_ERASE_BUSY_BM));
}


/**************************************************************************//**
* @brief    Writes a block of data to the DUP's XDATA space.
*
* @param    address     XDATA start address
* @param    values      Pointer to the array of bytes to write
* @param    num_bytes   Number of bytes to write
*
* @return   None.
******************************************************************************/
void write_xdata_memory_block(unsigned short address,
                              const unsigned char *values,
                              unsigned short num_bytes)
{
    unsigned char instr[3];
    unsigned short i;

    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    for (i = 0; i < num_bytes; i++)
    {
        // MOV A, values[i]
        instr[0] = 0x74;
        instr[1] = values[i];
        debug_command(CMD_DEBUG_INSTR_2B, instr, 2);

        // MOV @DPTR, A
        instr[0] = 0xF0;
        debug_command(CMD_DEBUG_INSTR_1B, instr, 1);

        // INC DPTR
        instr[0] = 0xA3;
        debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
    }
}


/**************************************************************************//**
* @brief    Writes a byte to a specific address in the DUP's XDATA space.
*
* @param    address     XDATA address
* @param    value       Value to write
*
* @return   None.
******************************************************************************/
void write_xdata_memory(unsigned short address, unsigned char value)
{
    unsigned char instr[3];

    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    // MOV A, values[i]
    instr[0] = 0x74;
    instr[1] = value;
    debug_command(CMD_DEBUG_INSTR_2B, instr, 2);

    // MOV @DPTR, A
    instr[0] = 0xF0;
    debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
}


/**************************************************************************//**
* @brief    Read a byte from a specific address in the DUP's XDATA space.
*
* @param    address     XDATA address
*
* @return   Value read from XDATA
******************************************************************************/
unsigned char read_xdata_memory(unsigned short address)
{
    unsigned char instr[3];

    // MOV DPTR, address
    instr[0] = 0x90;
    instr[1] = HIBYTE(address);
    instr[2] = LOBYTE(address);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    // MOVX A, @DPTR
    instr[0] = 0xE0;
    return debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
}


/**************************************************************************//**
* @brief    Reads 1-32767 bytes from DUP's flash to a given buffer on the
*           programmer.
*
* @param    bank        Flash bank to read from [0-7]
* @param    address     Flash memory start address [0x0000 - 0x7FFF]
* @param    values      Pointer to destination buffer.
*
* @return   None.
******************************************************************************/
void read_flash_memory_block(unsigned char bank,unsigned short flash_addr,
                             unsigned short num_bytes, unsigned char *values)
{
    unsigned char instr[3];
    unsigned short i;
    unsigned short xdata_addr = (0x8000 + flash_addr);

    // 1. Map flash memory bank to XDATA address 0x8000-0xFFFF
    write_xdata_memory(DUP_MEMCTR, bank);

    // 2. Move data pointer to XDATA address (MOV DPTR, xdata_addr)
    instr[0] = 0x90;
    instr[1] = HIBYTE(xdata_addr);
    instr[2] = LOBYTE(xdata_addr);
    debug_command(CMD_DEBUG_INSTR_3B, instr, 3);

    for (i = 0; i < num_bytes; i++)
    {
        // 3. Move value pointed to by DPTR to accumulator (MOVX A, @DPTR)
        instr[0] = 0xE0;
        values[i] = debug_command(CMD_DEBUG_INSTR_1B, instr, 1);

        // 4. Increment data pointer (INC DPTR)
        instr[0] = 0xA3;
        debug_command(CMD_DEBUG_INSTR_1B, instr, 1);
    }
}


/**************************************************************************//**
* @brief    Writes 4-2048 bytes to DUP's flash memory. Parameter \c num_bytes
*           must be a multiple of 4.
*
* @param    src         Pointer to programmer's source buffer (in XDATA space)
* @param    start_addr  FLASH memory start address [0x0000 - 0x7FFF]
* @param    num_bytes   Number of bytes to transfer [4-1024]
*
* @return   None.
******************************************************************************/
void write_flash_memory_block(unsigned char *src, unsigned long start_addr,
                              unsigned short num_bytes)
{
    // 1. Write the 2 DMA descriptors to RAM
    write_xdata_memory_block(ADDR_DMA_DESC_0, dma_desc_0, 8);
    write_xdata_memory_block(ADDR_DMA_DESC_1, dma_desc_1, 8);

    // 2. Update LEN value in DUP's DMA descriptors
    unsigned char len[2] = {HIBYTE(num_bytes), LOBYTE(num_bytes)};
    write_xdata_memory_block((ADDR_DMA_DESC_0+4), len, 2);  // LEN, DBG => ram
    write_xdata_memory_block((ADDR_DMA_DESC_1+4), len, 2);  // LEN, ram => flash

    // 3. Set DMA controller pointer to the DMA descriptors
    write_xdata_memory(DUP_DMA0CFGH, HIBYTE(ADDR_DMA_DESC_0));
    write_xdata_memory(DUP_DMA0CFGL, LOBYTE(ADDR_DMA_DESC_0));
    write_xdata_memory(DUP_DMA1CFGH, HIBYTE(ADDR_DMA_DESC_1));
    write_xdata_memory(DUP_DMA1CFGL, LOBYTE(ADDR_DMA_DESC_1));

    // 4. Set Flash controller start address (wants 16MSb of 18 bit address)
    write_xdata_memory(DUP_FADDRH, HIBYTE( (start_addr>>2) ));
    write_xdata_memory(DUP_FADDRL, LOBYTE( (start_addr>>2) ));

    // 5. Arm DBG=>buffer DMA channel and start burst write
    write_xdata_memory(DUP_DMAARM, CH_DBG_TO_BUF0);
    burst_write_block(src, num_bytes);

    // 6. Start programming: buffer to flash
    write_xdata_memory(DUP_DMAARM, CH_BUF0_TO_FLASH);
    write_xdata_memory(DUP_FCTL, 0x06);

    // 7. Wait until flash controller is done
    while (read_xdata_memory(FCTL) & 0x80);
}


/**************************************************************************//**
* @brief    Main function.
*
* @return   None.
******************************************************************************/
#if 0
void main(void)
{
    unsigned char chip_id = 0;
    unsigned char debug_config = 0;

    /****************************************
    * Initialize programmer
    *****************************************/
    programmer_init();


    /****************************************
    * Initialize debug interface
    *****************************************/
    // Put the DUP in debug mode
    debug_init();


    /****************************************
    * Read chip ID
    *****************************************/
    chip_id = read_chip_id();
    if(chip_id == 0) {
        while(1); // No chip detected.
    }


    /****************************************
    * Initialize DUP
    *****************************************/
    // Switch DUP to external crystal osc. (XOSC) and wait for it to be stable.
    // This is recommended if XOSC is available during programming. If
    // XOSC is not available, comment out these two lines.
    write_xdata_memory(DUP_CLKCONCMD, 0x80);
    while (read_xdata_memory(DUP_CLKCONSTA) != 0x80);


    /****************************************
    * Erase entire chip memory
    *****************************************/
    chip_erase();


    /****************************************
    * Write FLASH memory
    *****************************************/
    // Enable DMA (Disable DMA_PAUSE bit in debug configuration)
    debug_config = 0x22;
    debug_command(CMD_WR_CONFIG, &debug_config, 1);

    // Program data (start address must be word aligned [32 bit])
    write_flash_memory_block(write_data,  0x0100, 4); // src, address, count


    /****************************************
    * Read FLASH memory
    *****************************************/
    // Read 4 bytes starting at flash address 0x0100 (flash bank 0)
    read_flash_memory_block(0, 0x0100, 4, read_data); // Bank, address, count, dest.


    /****************************************
    * Verification
    *****************************************/
    // Bytwise check of read and written data
    for(unsigned char i = 0; i < sizeof(read_data); i++) {
        if(read_data[i] != write_data[i]) {
            // Fail
            while(1);
        }
    }

    // Infinite loop
    while(1);
}
#endif
