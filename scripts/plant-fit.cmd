@echo off
setlocal

rem plant-fit.cmd - Windows wrapper around scripts/plant-fit.sh.
rem
rem Locates Git Bash and hands off, so the invocation lives in one place.
rem Same split as test-host.cmd.
rem
rem This is the only one of the plant/blackbox scripts with a .cmd twin:
rem the others flash the board, and that needs dfu-util and lsusb, which
rem are not available on this Windows machine (see the workspace
rem CLAUDE.md). Flashing is a Debian job; fitting a log is not.
rem
rem   scripts\plant-fit.cmd capture.log

set "SCRIPT_DIR=%~dp0"
set "GIT_BASH="

if exist "%ProgramFiles%\Git\bin\bash.exe" set "GIT_BASH=%ProgramFiles%\Git\bin\bash.exe"
if not defined GIT_BASH if exist "%ProgramFiles(x86)%\Git\bin\bash.exe" set "GIT_BASH=%ProgramFiles(x86)%\Git\bin\bash.exe"
if not defined GIT_BASH if exist "%LOCALAPPDATA%\Programs\Git\bin\bash.exe" set "GIT_BASH=%LOCALAPPDATA%\Programs\Git\bin\bash.exe"

if not defined GIT_BASH for /f "delims=" %%G in ('where git 2^>nul') do call :derive "%%G"

if not defined GIT_BASH goto :nobash

"%GIT_BASH%" "%SCRIPT_DIR%plant-fit.sh" %*
exit /b %ERRORLEVEL%

:derive
if defined GIT_BASH goto :eof
set "CANDIDATE=%~dp1..\bin\bash.exe"
for %%F in ("%CANDIDATE%") do set "CANDIDATE=%%~fF"
if exist "%CANDIDATE%" set "GIT_BASH=%CANDIDATE%"
goto :eof

:nobash
echo !! could not find Git Bash ^(bash.exe^).
echo    Install Git for Windows, or run scripts/plant-fit.sh from Git Bash.
exit /b 1
