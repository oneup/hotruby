@echo off
set "PATH=%FLEX_HOME%\bin;%PATH%"

cd ..
call compile.bat Box2DFlashAS3
cd Box2DFlashAS3

copy /b "%~p0\..\..\src\ASHeader.as" + ImportAS.as + Box2DFlashAS3.js + "%~p0\..\..\web\js\HotRuby.js" HotRubyFlash.as
mxmlc -debug=true --strict=false --warnings=false HotRubyFlash.as
