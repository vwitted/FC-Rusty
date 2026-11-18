@echo off
setlocal

rem run-tests.cmd - Windows wrapper around renode/run-tests.sh.
rem
rem Locates Git Bash and hands off, so there is one copy of the logic.
rem `bash` on PATH under PowerShell is WSL (C:\WINDOWS\system32\bash.exe), a
rem separate Linux environment with no Renode and no Rust toolchain.
rem
rem   renode\run-tests.cmd          all checks
rem   renode\run-tests.cmd boot     only checks whose name matches

set "SCRIPT_DIR=%~dp0"
set "GIT_BASH="

if exist "%ProgramFiles%\Git\bin\bash.exe" set "GIT_BASH=%ProgramFiles%\Git\bin\bash.exe"
if not defined GIT_BASH if exist "%ProgramFiles(x86)%\Git\bin\bash.exe" set "GIT_BASH=%ProgramFiles(x86)%\Git\bin\bash.exe"
if not defined GIT_BASH if exist "%LOCALAPPDATA%\Programs\Git\bin\bash.exe" set "GIT_BASH=%LOCALAPPDATA%\Programs\Git\bin\bash.exe"

if not defined GIT_BASH for /f "delims=" %%G in ('where git 2^>nul') do call :derive "%%G"

if not defined GIT_BASH goto :nobash

"%GIT_BASH%" "%SCRIPT_DIR%run-tests.sh" %*
exit /b %ERRORLEVEL%

:derive
if defined GIT_BASH goto :eof
set "CANDIDATE=%~dp1..\bin\bash.exe"
for %%F in ("%CANDIDATE%") do set "CANDIDATE=%%~fF"
if exist "%CANDIDATE%" set "GIT_BASH=%CANDIDATE%"
goto :eof

:nobash
echo !! could not find Git Bash ^(bash.exe^).
echo    Install Git for Windows, or run tests/run-tests.sh from Git Bash directly.
exit /b 1
