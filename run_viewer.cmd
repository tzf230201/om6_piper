@echo off
call "%~dp0piper.cmd" gui --open %*
exit /b %errorlevel%
