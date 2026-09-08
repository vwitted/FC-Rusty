@echo off

rem test-sim-baseline.cmd - Host sim still produces the committed numbers.
rem
rem Not Renode: runs sim_sweep --csv and diffs it against
rem tests/sim-baseline.csv. Catches unintended changes to the sim or the
rem controllers -- a 17% change to motor_tau moves 252 values.
rem
rem When a change IS intended, accept it in the same commit with:
rem     BLESS=1 tests\run-tests.cmd sim-baseline
rem
rem Tier: fast.  Included in tests\run-tests.cmd --quick.

call "%~dp0renode-check.cmd" sim-baseline
exit /b %ERRORLEVEL%
