i2c-ir
======
This is a program for the NXP LPC810 ARM microcontroller that makes it act as an IR remote
control code sender.

It implements a basic controlling interface using I2C, where the master device can send
packets of data to set the timing, length, repeat count and delay between repeats for
a single IR remote control code.

The IR signals generated consist of variable duration pulses of a 38kHz square wave,
interleaved with variable duration gaps.
