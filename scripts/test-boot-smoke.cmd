@echo off

rem test-boot-smoke.cmd - Flight firmware survives H743 bring-up.
rem
rem Asserts defmt bytes leave USART6 (proving the clock tree came up,
rem not merely that the PC is somewhere in flash) and that the PC is in
rem the flash window. Needs the flight ELF staged: scripts\stage-firmware.cmd
rem
rem Tier: fast.  Included in tests\run-tests.cmd --quick.

call "%~dp0renode-check.cmd" boot-smoke
exit /b %ERRORLEVEL%
