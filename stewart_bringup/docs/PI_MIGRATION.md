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
  sorak@stablebot.local:/home/sorak/ros2_ws/src/stable_bot/stewart_bringup/
ssh sorak@stablebot.local \
  "cd ~/ros2_ws && colcon build --packages-select stewart_bringup --symlink-install && \
   sudo systemctl restart stable_bot.service"
```

---

## Post-migration lessons learned (2026-04-21 → 22)

Everything below was discovered while actually migrating. The steps above were the plan; the notes below are reality.

### Pi apt sources ship bare

Pi Imager's Ubuntu 24.04 image enables only `noble` and `noble-security` — no
`noble-updates`. You'll see `bzip2 : Depends: libbz2-1.0 (= 1.0.8-5.1) but
1.0.8-5.1build0.1 is to be installed` on the first `apt install`. Fix:

```bash
echo 'deb http://ports.ubuntu.com/ubuntu-ports noble-updates main restricted universe multiverse' | \
  sudo tee /etc/apt/sources.list.d/noble-updates.list
sudo apt update
```

### ROS 2 base vs. dev-tools

`ros-kilted-ros-base` is *runtime only*. To build the workspace you also need
`ros-dev-tools`, `ros-kilted-ament-cmake`, `ros-kilted-ament-cmake-auto`, and
`ros-kilted-rosidl-default-generators`.

### Xsens driver must be rebuilt for arm64

The Xsens driver ships prebuilt amd64 `.a` libs under
`lib/xspublic/{xscontroller,xscommon,xstypes}/`. If you rsync from an x86_64
laptop they come along for the ride and the Pi's linker rejects them with
`skipping incompatible ... libxscontroller.a`. Before the first `colcon build`
on the Pi:

```bash
cd ~/ros2_ws/src/Xsens_MTi_ROS_Driver_and_Ntrip_Client/src/xsens_mti_ros2_driver/lib/xspublic
make clean && make
```

### Repo layout mismatch bit us twice

The Pi clones into `~/ros2_ws/src/stable_bot/stewart_bringup/`, not the
laptop's flat `~/ros2_ws/src/stewart_bringup/`. Two places had hardcoded paths
that assumed the laptop layout and silently misbehaved on the Pi:

- `stewart_control_node.py` — the `LEG_LIMITS_PATH` and sibling config paths.
  Now use `_find_stewart_bringup_dir()` which looks for a real `package.xml`
  before picking a candidate dir.
- `stall_home.py` (outside the ROS package, at
  `~/Getting the robot working/Spin Motor Over CAN Test/`) — wrote homing
  results to a phantom directory the control node never read. Now uses the
  same resolver. **This file lives outside the repo** — after pulling on
  a new machine, copy `stall_home.py` manually if you need homing.

### Systemd on the Pi vs. `/launch` button in the GUI

On the Pi the stack is managed by `stable_bot.service` — systemd keeps it
running and restarts it on failure. The Diagnostics panel's green **(Re)launch
stack** button spawns its own copy via `gui_server.py`'s `/launch` endpoint,
which *duplicates* the stack. Symptoms:

- Two `stewart_control_node` processes — every topic message published twice.
- Two rosbridges — the second retry-loops on `[Errno 98] Address already in
  use` for port 9090, forever, filling the launch log.
- Two stacks fighting over the CAN bus and DDS discovery.

`gui_server.py` now refuses a `/launch` request if anything already owns
port 9090. On the Pi, **don't click that button** — use **Hard reset stack**
(red) instead. It kills everything cleanly; systemd brings the service back
up by itself.

### Reset script needs a longer budget on the Pi

`ros2 daemon start` can easily take >10s cold on a Pi 4. The reset script's
step 4 now waits up to 30s and is non-fatal (the daemon is only a CLI cache —
the stack runs fine without it). The gui_server's `/reset` HTTP timeout was
bumped from 30s → 90s for the same reason.

### Homing-prompt duplicate publish

An early bug where every homing line showed up twice turned out to be the
same duplicate-stack issue above, not a pump bug. Once you ensure there's
only one `stewart_control_node` running, each line appears once.

---

## Connecting your laptop (or phone) to the running Pi

Assumes `stable_bot.service` and `stable_bot_gui.service` are enabled —
the stack starts automatically on boot.

1. Turn on your phone's hotspot.
2. Power the Pi. Wait ~45 s for it to join the hotspot and boot the stack.
3. Open the hotspot's "Connected devices" list — you should see `stablebot`
   and its IP (e.g. `10.31.1.98`).
4. On any device sharing the hotspot:
   - **Browser GUI**: <http://stablebot.local:8080/> (or
     `http://<pi-ip>:8080/` if mDNS doesn't resolve on your OS).
   - **Rosbridge websocket URL** inside the GUI: `ws://stablebot.local:9090`
     (this is the field the GUI's "ws url" input points at — change it on
     first connect if it still says `localhost`).
   - **SSH**: `ssh sorak@stablebot.local`.

### When mDNS doesn't work

Some Windows setups and most WSL2 installs can't resolve `.local` names.
Fall back to the raw IP for both the page URL and the websocket URL.

### Daily startup / shutdown

Nothing to do — `systemctl enable --now stable_bot{,_gui}.service` was done
during migration, so the Pi brings everything up by itself.

```bash
# Check health from any SSH session:
systemctl status stable_bot stable_bot_gui --no-pager
journalctl -u stable_bot -n 50 --no-pager

# Clean shutdown:
sudo shutdown -h now
```

See also `stable_bot_startup.txt` in the home directory for a short cheat
sheet.
