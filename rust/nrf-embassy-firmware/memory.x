/* Contracted XIAO Sense prototype profile; no bootloader image header.
 * Bootloader 0.6.x source: USER_FLASH_END = 0xf4000 - 10 * 4096 = 0xea000.
 * Observed 0.6.1 CURRENT.UF2 coverage: [0x1000, 0xea000), not full flash.
 * Application [0x27000, 0xea000) excludes MBR/SoftDevice below 0x27000,
 * DFU app data [0xea000, 0xf4000), and boot code [0xf4000, 0x100000).
 * Hardcoded safe prototype default, not installed-binary attestation.
 * Reference: https://github.com/adafruit/Adafruit_nRF52_Bootloader/blob/97fbda552d62f6ba2d518143bac19bf35d8b6d27/src/usb/uf2/ghostfat.c
 * and lib/sdk11/components/libraries/bootloader_dfu/dfu_types.h at that revision.
 */
MEMORY
{
  FLASH : ORIGIN = 0x00027000, LENGTH = 0x000c3000
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
