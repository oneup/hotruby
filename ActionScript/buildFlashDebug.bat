@echo off
set "PATH=%FLEX_HOME%\bin;%PATH%"
copy /b "%~p0\..\src\ASHeader.as" + %1 + "%~p0\..\src\RubyVM.js" + "%~p0\..\src\RubyNative.js" HotRubyFlash.as
mxmlc -debug=true --strict=false --warnings=false -default-size 600 400 -default-frame-rate=30 -default-background-color=0xFFFFFF HotRubyFlash.as
