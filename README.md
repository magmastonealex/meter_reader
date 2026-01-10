meter_reader
====

This is an open-hardware design for a magnetic gas / water meter "reader" or "snooper". It contains a magnetometer and microcontroller, and can measure the rotations of a water meter or oscillations of the bellows in a gas meter to fairly accurately track consumption of both natural gas and water.

The device communicates externally using CAN, accepting configuration and reporting sensor values.

Schematics and PCB design are in the root of this repository. I have built a few devices, but would not consider the hardware to be "fully qualified" yet - I still need to do some transient testing and measure power supplies and signal integrity. I'd also still like to find a more available magnetometer - MLX3094RLD is _barely_ available and a bit annoying to solder.

Firmware in `firmware/` is Rust code targetting the MSPM0G3107 on the board. I _think_ this is the first working example of MSPM0 CAN access in Rust on the internet :) It requires modifications to the mspm0-data crate which I have in a branch here: https://github.com/magmastonealex/mspm0-data - there is an open PR upstream where smarter people than I are figuring out a better way to do this.

The firmware right now is just bringing up various peripherals to prove they work - the code is very, very ugly, requiring a lot of cleanup, and doesn't really "do" anything yet.

More to come, including a blog post detailing assembly, development, and setup
