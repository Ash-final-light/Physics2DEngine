@echo off
setlocal
pushd "%~dp0" || exit /b 1
set "BOX2D_PRESET=windows-relwithdebinfo"
if /i "%~1"=="Debug" set "BOX2D_PRESET=windows-debug"
cmake --preset windows
if errorlevel 1 goto failed
cmake --build --preset %BOX2D_PRESET%
if errorlevel 1 goto failed
ctest --preset %BOX2D_PRESET%
if errorlevel 1 goto failed
popd
exit /b 0
:failed
set "BOX2D_EXIT=%errorlevel%"
popd
exit /b %BOX2D_EXIT%
