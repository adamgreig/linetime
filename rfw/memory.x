MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 2M
    RAM   : ORIGIN = 0x20020000, LENGTH = 368K
    DTCM  : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
    .dtdm (NOLOAD) : ALIGN(4)
    {
        *(.dtcm .dtcm.*);
        . = ALIGN(4);
    } > DTCM
} INSERT AFTER .bss;
