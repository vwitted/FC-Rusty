@echo off
setlocal

rem test-host.cmd - Windows wrapper around scripts/test-host.sh.
rem
rem Locates Git Bash and hands off to the .sh, so there is only one copy of
rem the actual logic. Arguments are forwarded verbatim:
rem
rem   scripts\test-host.cmd                    all host tests
rem   scripts\test-host.cmd persist::record    only tests matching a filter
rem   scripts\test-host.cmd -- --nocapture     flags for the test harness
rem
rem Why this exists: `bash` on PATH in PowerShell is WSL
rem (C:\WINDOWS\system32\bash.exe), a separate Linux environment with its own
rem toolchain, so `bash scripts/test-host.sh` fails with "rustc: command not
rem found". Git Bash is the one that can see the Windows Rust install, and it
rem is usually not on PATH, hence the search below.

set "SCRIPT_DIR=%~dp0"
set "GIT_BASH="

if exist "%ProgramFiles%\Git\bin\bash.exe" set "GIT_BASH=%ProgramFiles%\Git\bin\bash.exe"
if not defined GIT_BASH if exist "%ProgramFiles(x86)%\Git\bin\bash.exe" set "GIT_BASH=%ProgramFiles(x86)%\Git\bin\bash.exe"
if not defined GIT_BASH if exist "%LOCALAPPDATA%\Programs\Git\bin\bash.exe" set "GIT_BASH=%LOCALAPPDATA%\Programs\Git\bin\bash.exe"

rem Last resort: derive it from wherever git.exe lives (...\Git\cmd\git.exe).
if not defined GIT_BASH for /f "delims=" %%G in ('where git 2^>nul') do call :derive "%%G"

if not defined GIT_BASH goto :nobash

"%GIT_BASH%" "%SCRIPT_DIR%test-host.sh" %*
exit /b %ERRORLEVEL%

:derive
if defined GIT_BASH goto :eof
set "CANDIDATE=%~dp1..\bin\bash.exe"
for %%F in ("%CANDIDATE%") do set "CANDIDATE=%%~fF"
if exist "%CANDIDATE%" set "GIT_BASH=%CANDIDATE%"
goto :eof

:nobash
echo !! could not find Git Bash ^(bash.exe^).
echo    Install Git for Windows, or run scripts/test-host.sh from Git Bash directly.
exit /b 1
