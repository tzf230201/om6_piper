@echo off
if exist "%~dp0piper_readonly\.venv\Scripts\python.exe" (
  "%~dp0piper_readonly\.venv\Scripts\python.exe" "%~dp0piper.py" %*
) else (
  "%~dp0..\piper_readonly\.venv\Scripts\python.exe" "%~dp0piper.py" %*
)
exit /b %errorlevel%
