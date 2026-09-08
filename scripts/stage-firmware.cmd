@echo off
setlocal

rem stage-firmware.cmd - Windows wrapper around scripts/stage-firmware.sh.
rem
rem Locates Git Bash and hands off, so the build-and-copy logic exists in
rem ONE place. What gets duplicated here is only the Git Bash discovery,
rem which is about where Git happens to be installed and does not change;
rem the build steps, which do change, are not duplicated. Same split as
rem test-host.cmd and the workspace's run-tests.cmd.
rem
rem `bash` on PATH under PowerShell is WSL (C:\WINDOWS\system32\bash.exe),
rem a separate Linux environment with no Rust toolchain and no Renode.
rem
rem   scripts\stage-firmware.cmd

set "SCRIPT_DIR=%~dp0"
set "GIT_BASH="

if exist "%ProgramFiles%\Git\bin\bash.exe" set "GIT_BASH=%ProgramFiles%\Git\bin\bash.exe"
if not defined GIT_BASH if exist "%ProgramFiles(x86)%\Git\bin\bash.exe" set "GIT_BASH=%ProgramFiles(x86)%\Git\bin\bash.exe"
if not defined GIT_BASH if exist "%LOCALAPPDATA%\Programs\Git\bin\bash.exe" set "GIT_BASH=%LOCALAPPDATA%\Programs\Git\bin\bash.exe"

if not defined GIT_BASH for /f "delims=" %%G in ('where git 2^>nul') do call :derive "%%G"

if not defined GIT_BASH goto :nobash

"%GIT_BASH%" "%SCRIPT_DIR%stage-firmware.sh" %*
exit /b %ERRORLEVEL%

:derive
if defined GIT_BASH goto :eof
set "CANDIDATE=%~dp1..\bin\bash.exe"
for %%F in ("%CANDIDATE%") do set "CANDIDATE=%%~fF"
if exist "%CANDIDATE%" set "GIT_BASH=%CANDIDATE%"
goto :eof

:nobash
echo !! could not find Git Bash ^(bash.exe^).
echo    Install Git for Windows, or run scripts/stage-firmware.sh from Git Bash.
exit /b 1
