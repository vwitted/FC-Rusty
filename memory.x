MEMORY
{
  /* STM32F722RET6 — 512 KB flash, 256 KB RAM, 16 KB ITCM */
  /*
   * Physical RAM on F722 is three contiguous regions starting at 0x20000000:
   *   DTCM   64 KB  @ 0x20000000  (zero-wait data-TCM)
   *   SRAM1 176 KB  @ 0x20010000
   *   SRAM2  16 KB  @ 0x2003C000
   * Total: 256 KB contiguous — treated as a single RAM region here.
   *
   * ITCM (16 KB @ 0x00000000) is exposed as a separate section so hot
   * code (e.g. the MPC solver inner loop) can be pinned there later
   * for zero-wait-state execution. Unused by default.
   *
   * NOTE: this file may be shadowed by the memory.x that
   * `embassy-stm32` generates in OUT_DIR when the `memory-x` feature
   * is enabled — keep it matching reality regardless.
   */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx): ORIGIN = 0x20000000, LENGTH = 255K
  ITCM  (rx) : ORIGIN = 0x00000000, LENGTH = 16K
}
