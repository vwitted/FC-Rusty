@echo off
rem sim-eval.cmd -- headless estimator evaluation; see scripts/sim-eval.sh.
rem Calls the `cargo sim-eval` alias directly, so no Git Bash is needed.
pushd "%~dp0.."
cargo sim-eval %*
set RC=%ERRORLEVEL%
popd
exit /b %RC%
