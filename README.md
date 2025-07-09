## INA228 Driver (code drop)

Bare-bones C++ driver for TI’s **INA228** precision power monitor.

* Written for an internal nRF52 project; depends on a thin `neoI2C` wrapper.  
* Builds in SEGGER Embedded Studio; porting notes are in the header comments.  
* Endian-correct for all 16 / 24 / 40-bit registers (MSB-first per datasheet).

_No guarantees. Pull requests welcome but merges not promised._

