@echo off

@if not "%MINGW_ROOT%" == "" (@set "PATH=%PATH%;%MINGW_ROOT%")

cd .

if "%1"=="" ("D:\Program\Matlab\POLYSP~1\R2020b\bin\win64\gmake"  -f UrbanPlanner_rtw.mk all) else ("D:\Program\Matlab\POLYSP~1\R2020b\bin\win64\gmake"  -f UrbanPlanner_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit 0

:error_exit
echo The make command returned an error of %errorlevel%
exit 1