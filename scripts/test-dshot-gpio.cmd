@echo off

rem test-dshot-gpio.cmd - DShot programs PA0..PA3 for bidirectional output.
rem
rem Reads GPIOA back after DshotBitbang::new and asserts OUTPUT /
rem PUSH_PULL / LOW_SPEED / PULL_UP. ~3 min: the motor-test firmware runs
rem its 2.5 s REMOVE PROPS countdown first, which is not skippable.
rem Needs the MOTOR-TEST ELF staged: scripts\stage-firmware.cmd
rem
rem Tier: slow.  Skipped by --quick.

call "%~dp0renode-check.cmd" dshot-gpio
exit /b %ERRORLEVEL%
