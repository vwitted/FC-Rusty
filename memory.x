MEMORY
{
  /* STM32H743VI — 2048 KB flash, 1024 KB RAM */
  /*
   * H7 memory map is distributed:
   *   DTCM     128 KB @ 0x20000000 (D1 domain)
   *   ITCM      64 KB @ 0x00000000 (D1 domain)
   *   AXI_SRAM 512 KB @ 0x24000000 (D1 domain)
   *   SRAM1    128 KB @ 0x30000000 (D2 domain - used for DMA)
   *   SRAM2    128 KB @ 0x30020000 (D2 domain)
   *   SRAM3     32 KB @ 0x30040000 (D2 domain)
   *   SRAM4     64 KB @ 0x38000000 (D3 domain)
   */
  /* Last 128 KB sector (bank 2, 0x081E0000) is reserved for the persist
   * config store; FLASH is shrunk to 1920K so the firmware image can't
   * overlap it. DFU programming writes only FLASH, so config survives a
   * reflash. See src/persist/flash.rs (CONFIG_OFFSET). */
  FLASH  (rx) : ORIGIN = 0x08000000, LENGTH = 1920K
  CONFIG (r)  : ORIGIN = 0x081E0000, LENGTH = 128K
  RAM    (rwx): ORIGIN = 0x24000000, LENGTH = 512K
  DTCM   (rwx): ORIGIN = 0x20000000, LENGTH = 128K
  ITCM   (rx) : ORIGIN = 0x00000000, LENGTH = 64K
}
