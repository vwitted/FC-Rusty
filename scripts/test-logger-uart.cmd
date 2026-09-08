@echo off

rem test-logger-uart.cmd - defmt UART matches the clock tree it assumes.
rem
rem logger.rs hard-codes APB2 = 120 MHz, so USART6 BRR is the load-bearing
rem value: if the clock tree changes underneath it the log silently becomes
rem garbage at the wrong baud. Needs the flight ELF staged.
rem
rem Tier: fast.  Included in tests\run-tests.cmd --quick.

call "%~dp0renode-check.cmd" logger-uart
exit /b %ERRORLEVEL%
