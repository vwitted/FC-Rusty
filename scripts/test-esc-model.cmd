@echo off

rem test-esc-model.cmd - A simulated ESC decodes the emulated DShot wire.
rem
rem tests/esc_model.py decodes DShot off the BSRR write stream and checks
rem all four motors produce a CRC-valid zero-throttle arming frame. ~100 s.
rem Decodes from write ORDER, never timestamps -- Renode runs the whole DMA
rem transfer in zero virtual time, so intervals here are meaningless.
rem Needs the MOTOR-TEST ELF staged.
rem
rem Tier: slow.  Skipped by --quick.

call "%~dp0renode-check.cmd" esc-model
exit /b %ERRORLEVEL%
