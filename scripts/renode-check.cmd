@echo off
setlocal

rem renode-check.cmd - run one check from the Renode harness by name.
rem
rem Shared by the per-check scripts beside it, so the path resolution exists
rem once rather than once per check.
rem
rem   scripts\renode-check.cmd boot-smoke
rem   scripts\renode-check.cmd sim-baseline
rem
rem The harness is renode\ in this repository; see renode\README.md. The
rem ELF-dependent checks need scripts\stage-firmware.cmd to have run.

if "%~1"=="" (
    echo usage: renode-check.cmd ^<check-name^>
    echo   boot-smoke  logger-uart  flight-bringup  sim-baseline  dshot-gpio  esc-model
    exit /b 2
)

set "RUNNER=%~dp0..\renode\run-tests.cmd"

set "CHECK=%~1"
shift

call "%RUNNER%" %CHECK%
exit /b %ERRORLEVEL%
