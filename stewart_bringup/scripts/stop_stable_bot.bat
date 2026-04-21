@echo off
REM Stable-Bot shutdown — kills every WSL-side Stable-Bot process.
REM Avoids cmd-quoting issues by calling a helper script in WSL.

echo Killing Stable-Bot processes inside WSL...
wsl -- bash /home/sorak/ros2_ws/src/stewart_bringup/scripts/_kill_all.sh

echo.
echo You can now close any leftover WSL windows if they didn't close.
timeout /t 3 >nul
