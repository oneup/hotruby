@echo off
set "PATH=%FLEX_HOME%\bin;%PATH%"
copy /b "%~p0\..\src\ASHeader.as" + %1 + "%~p0\..\web\js\HotRuby.js" HotRubyFlash.as
mxmlc --strict=false --warnings=false HotRubyFlash.as
