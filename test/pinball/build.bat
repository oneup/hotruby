@echo off
set "PATH=%FLEX_HOME%\bin;%PATH%"

cd ..
call compile.bat pinball
cd pinball

copy /b "%~p0\..\..\src\ASHeader.as" + ImportAS.as + pinball.js + "%~p0\..\..\web\js\HotRuby.js" HotRubyFlash.as
mxmlc --strict=false --warnings=false HotRubyFlash.as
