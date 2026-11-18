@echo off

rem test-flight-bringup.cmd - Flight firmware gets through sensor bring-up.
rem
rem Runs the flight ELF on the local overlay in renode\platforms\ (DMA that
rem clears EN on completion, a real SPI1) with fixed-value ICM-42688-P models
rem on SPI1 and SPI4 at level rest. Asserts, from decoded defmt, that both IMUs
rem initialise, the MEKF seeds at roll = pitch = 0, and the control loop
rem starts. Catches main() parked on an await, which boot-smoke cannot.
rem Needs the flight ELF staged (scripts\stage-firmware.cmd) and defmt-print.
rem
rem Tier: fast.  Included in renode\run-tests.cmd --quick.

call "%~dp0renode-check.cmd" flight-bringup
exit /b %ERRORLEVEL%
