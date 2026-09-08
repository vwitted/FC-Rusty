@echo off
setlocal

rem stage-firmware.cmd - build both firmware variants and put them where
rem the Renode harness reads them.
rem
rem Four of the five Renode checks need a staged ELF and none of them build
rem it, so this was a manual build-and-copy dance repeated every time the
rem firmware changed. It is exactly the step that gets skipped: on
rem 2026-09-08 the staged motor-test ELF was two weeks old, so the DShot
rem checks had been passing against a build that predated the plant-capture
rem work entirely. A stale ELF does not fail, it reports on the wrong
rem firmware -- see the FC_BUILD_STAMP note in motor_test.rs for the same
rem hazard biting on a bench.
rem
rem Order matters: motor-test is built FIRST and flight SECOND, so the
rem repo's own target/ tree is left holding the flight build. Both variants
rem write to the same cargo output path, and leaving motor-test there means
rem the next manual copy silently stages the wrong one.
rem
rem   scripts\stage-firmware.cmd

set "REPO=%~dp0.."
set "WORKSPACE=%REPO%\.."
set "OUT=%REPO%\target\thumbv7em-none-eabihf\release\fc-firmware"
set "DEST=%WORKSPACE%\target\thumbv7em-none-eabihf\release"

if not exist "%DEST%" (
    echo !! Renode harness staging directory not found:
    echo    %DEST%
    echo    These builds are only needed by the Renode checks, which live in
    echo    the workspace containing this checkout and are not part of this
    echo    repository. Nothing to stage.
    exit /b 2
)

pushd "%REPO%" || exit /b 1

echo ==^> building motor-test firmware
cargo build --release --features motor-test
if errorlevel 1 goto :fail
copy /y "%OUT%" "%DEST%\fc-firmware-motortest" >nul
if errorlevel 1 goto :fail

echo ==^> building flight firmware
cargo build --release
if errorlevel 1 goto :fail
copy /y "%OUT%" "%DEST%\fc-firmware" >nul
if errorlevel 1 goto :fail

rem The workspace root keeps a second copy of the flight ELF, and the root
rem CLAUDE.md states the two are byte-identical. Keep that true here rather
rem than leaving it to be noticed later.
copy /y "%OUT%" "%WORKSPACE%\fc-firmware" >nul
if errorlevel 1 goto :fail

popd
echo.
echo staged: fc-firmware ^(flight^), fc-firmware-motortest ^(bench^)
echo run the checks with scripts\test-^<name^>.cmd, or tests\run-tests.cmd
exit /b 0

:fail
popd
echo !! staging failed
exit /b 1
