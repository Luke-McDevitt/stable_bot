#!/usr/bin/env bash
# Stable-Bot Pi setup script.
#
# Run on a freshly-flashed Ubuntu 24.04 Server Pi AFTER you've:
#   1. Pre-configured WiFi + SSH via Raspberry Pi Imager advanced options
#   2. SSH'd in as your user
#   3. Cloned stewart_bringup into ~/ros2_ws/src/
#
# Idempotent — safe to re-run. Uses sudo for apt + udev + systemd.
#
# Usage:
#   cd ~/ros2_ws/src/stewart_bringup
#   bash scripts/install_on_pi.sh

set -euo pipefail

USER_NAME="${SUDO_USER:-$USER}"
HOME_DIR="$(getent passwd "$USER_NAME" | cut -d: -f6)"
WS_DIR="$HOME_DIR/ros2_ws"
PKG_DIR="$WS_DIR/src/stewart_bringup"

step() { echo -e "\n\033[1;36m=== $1 ===\033[0m"; }

if [[ ! -d "$PKG_DIR" ]]; then
    echo "ERROR: $PKG_DIR not found. Clone the repo into $WS_DIR/src first." >&2
    exit 1
fi

step "installing apt packages (system deps, ROS 2 Kilted, rosbridge, can-utils)"
sudo apt update
sudo apt install -y curl gnupg lsb-release git python3-pip python3-venv \
                    can-utils build-essential

if ! apt list --installed 2>/dev/null | grep -q ros-kilted-ros-base; then
    step "adding ROS 2 apt repo"
    sudo curl -sSL \
        https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    codename="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $codename main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
fi

sudo apt install -y ros-kilted-ros-base ros-kilted-rosbridge-suite \
                    ros-kilted-rclpy python3-colcon-common-extensions \
                    python3-rosdep
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

step "installing Python deps (python-can, pyyaml, numpy)"
pip3 install --break-system-packages python-can pyyaml numpy matplotlib

step "udev rules for stable device symlinks + gs_usb permissions"
sudo tee /etc/udev/rules.d/60-stablebot.rules > /dev/null <<'EOF'
# Xsens MTi-630 IMUs. EDIT the serial numbers below to match yours
# — find them with `udevadm info -a /dev/ttyUSB0 | grep serial`.
SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{serial}=="DBBBRHGX", \
    SYMLINK+="imu_mti630",   MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{serial}=="DBBBRHBS", \
    SYMLINK+="imu_mti630_b", MODE="0666"

# gs_usb CAN adapter permissions (kernel auto-names it can0)
SUBSYSTEM=="net", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", \
    GROUP="dialout", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
if ! groups "$USER_NAME" | grep -q dialout; then
    sudo usermod -aG dialout "$USER_NAME"
    echo "  added $USER_NAME to dialout — log out + back in for it to stick"
fi

step "bringing up can0 on boot (systemd-networkd)"
sudo tee /etc/systemd/network/80-can0.network > /dev/null <<EOF
[Match]
Name=can0

[CAN]
BitRate=1000000
EOF
sudo networkctl reload || true

step "resolve rosdep dependencies + colcon build"
cd "$WS_DIR"
source /opt/ros/kilted/setup.bash
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --symlink-install

if ! grep -q "ros2_ws/install/local_setup.bash" "$HOME_DIR/.bashrc"; then
    echo "source $WS_DIR/install/local_setup.bash" >> "$HOME_DIR/.bashrc"
    echo "source /opt/ros/kilted/setup.bash" >> "$HOME_DIR/.bashrc"
    echo "  appended ROS source lines to ~/.bashrc"
fi

step "installing systemd services (stable_bot, stable_bot_gui)"
# Substitute USER and HOME into the service files so they work for any
# user. The source files use hard-coded 'sorak' by default.
sudo sed -e "s|User=sorak|User=$USER_NAME|g" \
         -e "s|Group=sorak|Group=$USER_NAME|g" \
         -e "s|/home/sorak|$HOME_DIR|g" \
         "$PKG_DIR/scripts/stable_bot.service" \
         > /etc/systemd/system/stable_bot.service.tmp
sudo mv /etc/systemd/system/stable_bot.service.tmp \
        /etc/systemd/system/stable_bot.service

sudo sed -e "s|User=sorak|User=$USER_NAME|g" \
         -e "s|Group=sorak|Group=$USER_NAME|g" \
         -e "s|/home/sorak|$HOME_DIR|g" \
         "$PKG_DIR/scripts/stable_bot_gui.service" \
         > /etc/systemd/system/stable_bot_gui.service.tmp
sudo mv /etc/systemd/system/stable_bot_gui.service.tmp \
        /etc/systemd/system/stable_bot_gui.service

sudo systemctl daemon-reload
sudo systemctl enable stable_bot_gui.service stable_bot.service
echo "  services installed and enabled (will auto-start on boot)"
echo "  start them now with: sudo systemctl start stable_bot_gui stable_bot"

step "done"
cat <<EOF

Next steps:
  1. Reboot (or at minimum log out + back in so dialout group takes effect):
       sudo reboot
  2. Find the Pi's IP: either check the phone's hotspot clients list,
     or run \`hostname -I\` once logged in.
  3. From your laptop browser: http://<pi-ip>:8080/ (or
     http://$(hostname).local:8080/ if mDNS works on your network).
  4. Watch the stack come up:
       journalctl -u stable_bot.service -f
  5. If the Xsens IMUs aren't detected, edit their serial numbers in
     /etc/udev/rules.d/60-stablebot.rules and re-run
     \`sudo udevadm control --reload-rules && sudo udevadm trigger\`.

EOF
