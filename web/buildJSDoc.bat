@echo off
del /S /Q jsdoc
SET "BASE_DIR=C:\Program Files\JavaScript\jsdoc_toolkit-1.4.0"
java "-Djsdoc.dir=%BASE_DIR%" -jar "%BASE_DIR%\app\js.jar" "%BASE_DIR%\app\run.js" --directory=jsdoc --private "--template=%BASE_DIR%\templates\sunny" js\HotRuby.js
