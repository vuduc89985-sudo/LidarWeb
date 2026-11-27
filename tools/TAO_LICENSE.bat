@echo off
title VLASER - TAO LICENSE BAN QUYEN (NHA CUNG CAP)
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0generate_license.ps1"
echo.
pause
