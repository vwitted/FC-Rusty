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
  /*
   * Flash is split along the H743's two 1 MB banks, and the split is
   * load-bearing rather than tidy.
   *
   * Code lives in BANK 1 only. Bank 2 carries the persist config store
   * and the blackbox log. Two reasons:
   *
   *  1. DFU programming writes only FLASH, so everything in bank 2
   *     survives a reflash. That is what makes the blackbox workflow
   *     work at all: fly a recording build, then flash a dump build and
   *     read back the flight you just did.
   *  2. The H7 cannot read a bank while programming it. Executing from
   *     bank 1 while writing bank 2 avoids that stall entirely, which is
   *     what allows logging during flight rather than only on the bench.
   *
   * Bank 1 is 1024K and the image is ~200K, so there is ample room; if
   * that ever stops being true the fix is to shrink the blackbox, not to
   * spill code into bank 2.
   *
   * See src/persist/flash.rs (CONFIG_OFFSET) and src/blackbox.rs.
   */
  FLASH    (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  BLACKBOX (r)  : ORIGIN = 0x08100000, LENGTH = 896K
  CONFIG   (r)  : ORIGIN = 0x081E0000, LENGTH = 128K
  RAM    (rwx): ORIGIN = 0x24000000, LENGTH = 512K
  DTCM   (rwx): ORIGIN = 0x20000000, LENGTH = 128K
  ITCM   (rx) : ORIGIN = 0x00000000, LENGTH = 64K
}
