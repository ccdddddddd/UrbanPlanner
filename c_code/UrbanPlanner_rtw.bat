@echo off

set MATLAB=C:\Program\Polyspace\R2020b

cd .

if "%1"=="" ("C:\Program\POLYSP~1\R2020b\bin\win64\gmake"  -f UrbanPlanner_rtw.mk all) else ("C:\Program\POLYSP~1\R2020b\bin\win64\gmake"  -f UrbanPlanner_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit 0

:error_exit
echo The make command returned an error of %errorlevel%
exit 1