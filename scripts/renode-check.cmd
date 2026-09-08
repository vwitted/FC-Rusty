@echo off
setlocal

rem renode-check.cmd - run one check from the Renode harness by name.
rem
rem Shared by the per-check scripts beside it, so the path resolution and
rem the "harness is missing" message exist once rather than five times.
rem
rem   scripts\renode-check.cmd boot-smoke
rem   scripts\renode-check.cmd sim-baseline
rem
rem The harness is NOT in this repository. It lives in the Renode
rem workspace that contains this checkout, one level up, and is not
rem version-controlled -- see the root CLAUDE.md. So a standalone clone of
rem FC-Rusty-Code cannot run these, and saying so plainly beats a confused
rem "file not found" from somewhere three scripts deep.

if "%~1"=="" (
    echo usage: renode-check.cmd ^<check-name^>
    echo   boot-smoke  logger-uart  sim-baseline  dshot-gpio  esc-model
    exit /b 2
)

set "WORKSPACE=%~dp0..\.."
set "RUNNER=%WORKSPACE%\tests\run-tests.cmd"

if not exist "%RUNNER%" (
    echo !! Renode harness not found at %RUNNER%
    echo    These checks need the Renode workspace that contains this
    echo    checkout; it is not part of this repository. A standalone
    echo    clone can still run the host tests: scripts\test-host.cmd
    exit /b 2
)

set "CHECK=%~1"
shift

"%RUNNER%" %CHECK%
exit /b %ERRORLEVEL%
