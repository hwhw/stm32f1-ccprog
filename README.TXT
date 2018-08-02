Flashing tool for CC253x devices using the STM32F1xx as debugger

Implements the debugging protocol. Presents an USB serial interface
onto which you can pipe Intel Hex encoded firmware files for the
CC253x.

Also has some additional commands.

Still very much WIP, but it successfully flashed by CC2531 USB stick.
Bought that for a few bucks in China and was to eager to play with it
so I didn't want to wait until the programmer arrives from there, too.

Re-uses a bit of TI's demo code for the debugging interface.
