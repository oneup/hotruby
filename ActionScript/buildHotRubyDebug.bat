@echo off
set "PATH=%FLEX_HOME%\bin;%PATH%"
copy /b HotRubyHeader.as + test.as + ..\web\js\HotRuby.js + ..\web\js\NativeMethods.js HotRubyFlash.as
mxmlc -debug=true --strict=false --warnings=false -default-size 600 400 -default-frame-rate=30 -default-background-color=0xFFFFFF HotRubyFlash.as
pause