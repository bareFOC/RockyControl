/* RP2350 (Pico 2 / RP2350A) linker script for the Embassy-only firmware runtime. */
MEMORY {
    /* Pico 2 ships with 4 MiB QSPI flash; 2 MiB is a conservative universal default. */
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
    /* 512 KiB striped SRAM (SRAM0–SRAM7). */
    RAM   : ORIGIN = 0x20000000, LENGTH = 512K
    /* Dedicated core scratch banks. */
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
}

SECTIONS {
    /* RP2350 Boot ROM PICOBIN image definition block (must reside in first 4 KiB of flash). */
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH
} INSERT AFTER .vector_table;

_stext = ALIGN(ADDR(.start_block) + SIZEOF(.start_block), ALIGNOF(.text));

SECTIONS {
    /* C/C++ startup constructors (kept for toolchain compatibility; no runtime use). */
    .init_array : ALIGN(4)
    {
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
        KEEP(*(.init_array))
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    /* Picotool binary info entries emitted by embassy-rp `binary-info`. */
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .init_array;

SECTIONS {
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
        __flash_binary_end = .;
    } > FLASH
} INSERT AFTER .uninit;

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);
