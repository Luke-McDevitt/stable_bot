@echo off
REM Stable-Bot hard reset + relaunch + verify.
REM Runs reset_stewart_stack.py --launch --verify in a WSL window so
REM you can watch the progress output and the healthy-at-3.4s check.

start "Stable-Bot: hard reset + relaunch" cmd /k wsl -- python3 /home/sorak/ros2_ws/src/stewart_bringup/scripts/reset_stewart_stack.py --launch --verify

REM Give the verify a moment to succeed before auto-opening browser.
timeout /t 8 /nobreak >nul
start "" "http://localhost:8080/"
