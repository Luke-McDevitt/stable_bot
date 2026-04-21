# Phase 8 — Raspberry Pi Migration

Goal: move the backend (CAN + IMUs + control node + rosbridge + GUI
server) off the laptop and onto a Raspberry Pi mounted on/near the
Stewart platform. Laptop becomes a thin client — a browser pointed at
the Pi's GUI over WiFi. No more usbipd, no more "unplug and replug the
CAN adapter," no more WSL2 DDS quirks.

## Hardware list

- **Raspberry Pi 4B 4GB+ or Pi 5** (Pi 5 preferred — ROS 2 Kilted is
  compile-heavy on armhf, and the Xsens driver benefits from the extra
  cores)
- 32GB+ A1/A2 microSD card
- 5V USB-C PSU (3A min for Pi 4, 5A recommended for Pi 5)
- **Your existing** USB-CAN gs_usb adapter, 2× Xsens MTi-630, 6× ODrive Pro
- Laptop with a browser (connects to the Pi's IP)
- Optional: a 4-port powered USB hub so the Pi can feed the IMUs and
  CAN adapter together

## One-time SD card setup (answers your WiFi-hotspot question)

**Use Raspberry Pi Imager** from any OS. It has a pre-boot configuration
panel that handles headless WiFi, SSH, and hostname in one go — this
is the simplest way to make a Pi auto-connect to a phone hotspot
without attaching a screen.

1. Download **Raspberry Pi Imager** (Windows/Mac/Linux).
2. Choose OS → **Other general-purpose OS → Ubuntu → Ubuntu Server
   24.04 LTS (64-bit)**.
   *(Required for ROS 2 Kilted. Raspberry Pi OS doesn't currently have
   prebuilt Kilted binaries; Ubuntu 24.04 does.)*
3. Choose storage → your microSD card.
4. **Click the gear icon** (or `Ctrl+Shift+X`) → this opens the
   Advanced Options panel:
   - ✅ **Set hostname**: `stablebot` (so you can reach it as
     `stablebot.local` via mDNS)
   - ✅ **Enable SSH** → "Use password authentication" (or keys — your
     choice)
   - ✅ **Set username and password**: `sorak` / (your pick)
   - ✅ **Configure wireless LAN**:
     - **SSID**: name of your phone's hotspot (case-sensitive; must be
       exact)
     - **Password**: your hotspot password
     - **Wireless LAN country**: set to your country (US, GB, DE, etc.)
       — required or WiFi won't come up
   - **Set locale settings** (time zone + keyboard layout)
5. Save → Write. The Imager bakes these into `system-boot/` and
   `writable/` on the SD card so the Pi configures itself on first boot.
6. Eject, insert into Pi, plug in power.
7. **Turn on your phone's hotspot** (same SSID + password you entered).
8. Wait ~60 s. Check your phone's hotspot "connected devices" list —
   you should see the Pi appear. Note its IP (e.g. `192.168.43.42`).

### Connecting multiple networks (phone hotspot + home/lab WiFi)

If you want the Pi to prefer your lab WiFi when available and fall
back to the phone hotspot, after first boot edit
`/etc/netplan/50-cloud-init.yaml` (Ubuntu 24.04 uses netplan):

```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "LabWiFi":
          password: "labpass"
        "PhoneHotspot":
          password: "phonepass"
```
Then `sudo netplan apply`.

Ubuntu will try access points in the order listed, rotating if the
first isn't in range. For priority weighting, use NetworkManager's
`nmcli` instead.

### Finding the Pi without a screen

- **mDNS** (simplest, usually works): `ping stablebot.local` from your
  laptop on the same network. If it resolves, SSH in with
  `ssh sorak@stablebot.local`.
- **Phone hotspot clients list**: shows every device + IP.
- **nmap** of your local subnet: `nmap -sn 192.168.43.0/24` (adjust
  for your hotspot range).
- **Router admin page**: if on lab WiFi, the router usually lists DHCP
  leases.

## First boot on the Pi

Once SSH'd in:

```bash
ssh sorak@stablebot.local

# Update and install system deps
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release git python3-pip python3-venv \
                     can-utils build-essential

# Setup the ROS 2 Kilted apt repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 base + rosbridge
sudo apt update
sudo apt install -y ros-kilted-ros-base ros-kilted-rosbridge-suite \
                     ros-kilted-rclpy python3-colcon-common-extensions \
                     python3-rosdep
sudo rosdep init
rosdep update

# Python deps
pip3 install --break-system-packages python-can pyyaml numpy
```

## Clone + build the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Copy or git-clone:
git clone https://github.com/YOUR-REPO/stewart_bringup.git
git clone https://github.com/YOUR-REPO/jugglebot_interfaces.git
# Xsens driver (required — not in apt):
git clone https://github.com/xsens/xsens_mti_ros2_driver.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/kilted/setup.bash
colcon build --symlink-install
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
```

## udev rules for stable device names

The gs_usb CAN adapter and Xsens IMUs present at `/dev/ttyUSB0`,
`/dev/ttyUSB1`, etc. — but the numbering isn't deterministic. Use udev
rules to pin them to stable paths. The existing laptop setup has these
at `/dev/imu_mti630` and `/dev/imu_mti630_b` — replicate on the Pi:

```bash
sudo tee /etc/udev/rules.d/60-stablebot.rules <<'EOF'
# Xsens MTi-630 — two units, pin by serial number
SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{serial}=="DBBBRHGX", \
    SYMLINK+="imu_mti630",   MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{serial}=="DBBBRHBS", \
    SYMLINK+="imu_mti630_b", MODE="0666"
# gs_usb CAN adapter permissions (no symlink — kernel names it can0)
SUBSYSTEM=="net", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", \
    GROUP="dialout", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout sorak
# log out + back in for group change to take effect
```

## Bring up can0 on boot

Add a systemd network unit (or /etc/network/interfaces.d entry, or
netplan snippet — Ubuntu 24.04 prefers netplan/networkd). Simplest:

```bash
sudo tee /etc/systemd/network/80-can0.network <<EOF
[Match]
Name=can0

[CAN]
BitRate=1000000
EOF
sudo networkctl reload
```

This brings can0 up at 1 Mbps every boot automatically.

## Auto-start services on boot

Two systemd units — one for the ROS 2 launch, one for the GUI HTTP server:

```bash
sudo cp ~/ros2_ws/src/stewart_bringup/scripts/stable_bot.service \
        /etc/systemd/system/
sudo cp ~/ros2_ws/src/stewart_bringup/scripts/stable_bot_gui.service \
        /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now stable_bot_gui.service stable_bot.service
```

Check status any time:
```bash
systemctl status stable_bot.service stable_bot_gui.service
journalctl -u stable_bot.service -f        # live logs
```

## Access from the laptop

Point your laptop's browser at **`http://stablebot.local:8080/`**
(or `http://<pi-ip>:8080/` if mDNS is flaky). That's it — the browser
talks to the Pi's rosbridge at `ws://stablebot.local:9090` and its
GUI server at `:8080`. Zero ROS setup on the laptop.

If connecting from a different subnet, you may need to adjust ROS DDS
discovery — see the "multiple machines" section of the ROS 2 docs.
Within a single WiFi network it just works with default discovery.

## Power-cycle recovery

The gs_usb adapter occasionally needs a physical replug when bus-off
happens with flaky cabling. On the Pi (no usbipd required):

- Physically unplug USB-CAN, wait 3 s, replug → can0 re-appears via the
  systemd-networkd unit above.
- `sudo systemctl restart stable_bot.service` to restart the ROS stack
  — or just use the **Hard reset stack** + **(Re)launch stack** buttons
  in the web GUI, same as the WSL2 workflow.

## What does NOT change from the WSL2 setup

- GUI (`index.html`) — same file served by `gui_server.py`.
- Launch file, control node, all scripts — same code.
- Recording format and analyzer — same.
- `reset_stewart_stack.py` still works for hard resets.

The Pi migration is really just: "run all the same stuff, but on a
different Linux box that happens to have native USB and no WSL2
weirdness."

## Laptop-only mode after migration

You can still run the WSL2 stack alongside the Pi — they don't conflict
because each has its own CAN bus (WSL2 via usbipd, Pi via native USB).
Useful for development: iterate code on the WSL2 side, rsync to the Pi
when happy:

```bash
rsync -av --delete \
  ~/ros2_ws/src/stewart_bringup/ \
  sorak@stablebot.local:/home/sorak/ros2_ws/src/stewart_bringup/
ssh sorak@stablebot.local \
  "cd ~/ros2_ws && colcon build --packages-select stewart_bringup --symlink-install && \
   sudo systemctl restart stable_bot.service"
```
