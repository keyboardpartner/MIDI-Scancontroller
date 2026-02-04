rem Merge Bootloader and firmware to one file, rename to firmware.hex for ISP programming
type optiboot_atmega328_20M.hex .\.pio\build\ATmega328P_20MHz_USBasp\firmware.hex > firmware.hex