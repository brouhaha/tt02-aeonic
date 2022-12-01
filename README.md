![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Aeonic: Vertically microcoded processor using SPI FRAM memory

This project is a vertically microcoded processor, intended to be
fabricated in [Tiny Tapeout 2](https://tinytapeout.com/). The
microarchitecture is a variant of
[Glacial](https://github.com/brouhaha/glacial), but whereas Glacial
executed all microinstructions in four clock cycles, using FPGA
blockram for both microcode and data storage, Aeonic takes a variable
number of clock cycles to execute each microinstruction, due to the
use of external serial (SPI) memory.  SPI FRAM is used because it is
nonvolatile, like flash memory, but has not write latency, like
SRAM. It is expected that SPI MRAM would also work.

The microinstruction set is identical to that of Glacial, with the
exception of the I/O microinstructions. See the comments in aeonic.v
for details. The Glacial repository wiki has a PDF file documenting
the microinstruction set.
