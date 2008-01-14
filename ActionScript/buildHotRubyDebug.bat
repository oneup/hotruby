@echo off
set "PATH=C:\Program Files\flex2_sdk_hf1\bin;%PATH%"
copy /b HotRubyHeader.as + test.as + ..\js\HotRuby.js + ..\js\NativeMethods.js HotRubyFlash.as
mxmlc -debug=true --strict=false --warnings=false -default-size 600 400 -default-frame-rate=30 -default-background-color=0xFFFFFF HotRubyFlash.as
pause