Flashing tool for CC253x devices using the STM32F1xx as debugger

Implements the debugging protocol. Presents an USB serial interface
onto which you can pipe Intel Hex encoded firmware files for the
CC253x.

Also has some additional commands.

Still very much WIP, but it successfully flashed by CC2531 USB stick.
Bought that for a few bucks in China and was to eager to play with it
so I didn't want to wait until the programmer arrives from there, too.

Re-uses a bit of TI's demo code for the debugging interface.


Building:
---------

Make sure you have git and an arm-none-eabi toolchain ready.

git submodule init
git submodule update
make

Flash the binary build/ch.bin to your STM32F1xx. Depending on if you
do this using the ST-Link or the bootloader plus UART, it's

st-flash write build/ch.bin 0x8000000

or 

stm32flash -w build/ch.bin -v /dev/tty...


Usage:
------

You need to connect the CC2531 debug port to the STM32F1xx as follows:

      1 +------+------+
 GND -- | GND  |      |
        +------+------+
 PA1 -- | DCLK |   DD | -- PA0
        +------+------+ 
        |      |      |
        +------+------+ 
 PA2 -- | RST  |      |
        +------+------+ 
        |      |      |
        +------+------+ 10
         CC2531 dbg hdr

So - all the signals will be run out of the STM32's PA0,1,2 outputs.
Just power the CC2531 stick via USB. Do *not* connect any VCC lines.

Plug in your flashed STM32F1xx.

Use e.g. flash.sh in order to flash a CC2531. Syntax:

  flash.sh /dev/ttyACMx firmware.hex > flash.log

(it will output a lot of debugging info, so make sure you redirect the output)
It will need about 20s for 256kByte of effective firmware data.

You can also use a terminal program to connect to the USB serial TTY device.
The one of the STM32F1xx, not the CC2531 one (depending on firmware type).
Sorry, commands are not yet documented.
