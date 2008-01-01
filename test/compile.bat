@echo off
ruby compile.rb %1\%1.rb -o %1\%1.json -p %1\%1_pretty.json
