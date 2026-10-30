SECTIONS
{
  .integration_metadata : ALIGN(4)
  {
    KEEP(*(.integration_metadata));
  } > FLASH
} INSERT AFTER .rodata;
