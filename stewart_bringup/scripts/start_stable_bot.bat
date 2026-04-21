@echo off
REM ============================================================
REM  Stable-Bot Windows launcher (WSL2)
REM
REM  Double-click to start the whole stack and open the GUI.
REM
REM  This .bat deliberately does NOT embed bash commands — nested
REM  Windows-cmd quoting eats escaping. Instead it calls two helper
REM  shell scripts that live inside WSL:
REM
REM    scripts/_launch_gui_server.sh   (brings up can0 + runs gui_server.py)
REM    scripts/_launch_ros_stack.sh    (sources ROS + runs `ros2 launch`)
REM
REM  Prerequisites (one-time):
REM    - WSL2 Ubuntu with ROS 2 Kilted + the workspace built
REM    - Passwordless sudo for `ip link set can0 *` (recommended):
REM        echo 'USERNAME ALL=(ALL) NOPASSWD: /usr/sbin/ip link set can0 *' ^
REM           | sudo tee /etc/sudoers.d/stewart-can
REM      If not configured, the gui_server window will prompt for
REM      the sudo password — type it and the rest proceeds.
REM    - USB-CAN adapter + both MTi-630 IMUs attached via usbipd
REM      (run `usbipd attach --wsl --busid X-Y` from admin PowerShell
REM      once per reboot for each device).
REM
REM  Shutdown: close either WSL window, or run stop_stable_bot.bat.
REM ============================================================

echo.
echo === Starting Stable-Bot ===
echo.

REM Helper scripts. Paths are WSL-side (Linux), not Windows.
set GUI_SCRIPT=/home/sorak/ros2_ws/src/stewart_bringup/scripts/_launch_gui_server.sh
set ROS_SCRIPT=/home/sorak/ros2_ws/src/stewart_bringup/scripts/_launch_ros_stack.sh

REM Step 1: gui_server + can0. Window stays open (cmd /k).
start "Stable-Bot: gui_server + can0" cmd /k wsl -- bash %GUI_SCRIPT%

REM Step 2: ROS 2 launch bundle. Give step 1 a moment so can0 is up
REM before the control node opens it.
timeout /t 3 /nobreak >nul
start "Stable-Bot: ROS 2 launch" cmd /k wsl -- bash %ROS_SCRIPT%

REM Step 3: wait for backend to come up, then open the browser.
echo Waiting 6s for the backend...
timeout /t 6 /nobreak >nul

echo Opening http://localhost:8080/
start "" "http://localhost:8080/"

echo.
echo === Stable-Bot launched ===
echo.
echo Two WSL windows are running the backend. Close them to stop.
echo.
echo If the GUI hangs on "(connected but no /status received...)":
echo    reset_stable_bot.bat
echo.

timeout /t 5 >nul
