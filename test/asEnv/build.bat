@echo off
set "PATH=%FLEX_HOME%\bin;%PATH%"

cd ..
call compile.bat asEnv
cd asEnv

call ..\..\ActionScript\buildFlash.bat asEnv.js
