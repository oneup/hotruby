@echo off

:check1
if not "%1" == "" goto main
echo Please add argument
goto end

:main
ruby compile.rb %1\%1.rb -o %1\%1.js -p %1\%1_pretty.json

:end
