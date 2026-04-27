# Configuring ODrive parameters from the Pi

The Stable-Bot Pi can read and write ODrive Pro parameters over the
existing CAN bus — no USB cable swapping. This doc covers the
workflow, the gotchas, and how to extend it.

Two-script pair:
- [`scripts/dump_odrive_endpoints.py`](../scripts/dump_odrive_endpoints.py)
  — one-time per firmware version, USB-only.
- [`scripts/set_odrive_feedforward_via_can.py`](../scripts/set_odrive_feedforward_via_can.py)
  — runtime, CAN-only.

The parameters they currently target are `vel_integrator_gain`
and `wL_FF_enable` (per-axis), plus `save_configuration` (function);
adding more is a one-line edit to the `TARGETS` list at the top of
each script.

## Why two scripts (and not just the official odrivetool)

`odrivetool` and the modern `odrive` Python library only support USB
and serial transports — `find_sync` has no `path` argument, the
`DeviceManager` has no `add_can_*` hook, and `odrivetool --help`
lists no `can:` syntax. This is a regression from the older 0.5.x
fibre-based library which did support CAN. So using the official
tooling for CAN-side parameter writes isn't possible without
dropping back to a much older library.

The fix: speak ODrive's `Tx_SDO` (cmd 0x04) protocol directly via
python-can, using endpoint IDs that we discover once over USB.
ODrive's CAN protocol exposes every parameter via SDO; the only
piece we need to look up is the integer endpoint ID for each named
parameter.

## One-time endpoint dump (when firmware changes)

Endpoint IDs are baked at firmware compile time. They're identical
across every ODrive of the same firmware version, so doing this for
one drive captures them for all six.

```bash
# Stop the running stack so it isn't fighting for can0/USB:
sudo systemctl stop stable_bot

# Plug ONE USB cable to ANY of the six ODrives. udev rules must be
# in place — see "Pi prerequisites" below if you haven't done this.
python3 scripts/dump_odrive_endpoints.py

# Output is data/odrive_endpoints.json. Commit it to the repo:
git add stewart_bringup/data/odrive_endpoints.json
git commit -m "Endpoint IDs for fw <version>"
git push

# Unplug the USB cable, restart the stack:
sudo systemctl start stable_bot
```

The dump uses defensive reflection (looks for both
`parent._<name>_property._info.endpoint_id` for typed parameters
and `leaf._info.endpoint_id` for function endpoints) so it survives
minor library refactors. If a future firmware/library combo breaks
extraction, the script's verbose diagnostic dump shows the live
attribute layout so the reflection can be patched without another
USB cycle.

## Routine parameter writes (CAN-only)

```bash
sudo systemctl stop stable_bot

# See current values across all 6 nodes (read-only, no changes):
python3 scripts/set_odrive_feedforward_via_can.py --status

# Apply new values:
python3 scripts/set_odrive_feedforward_via_can.py --apply

# Restore previous values from the backup file:
python3 scripts/set_odrive_feedforward_via_can.py --revert

sudo systemctl start stable_bot
```

`--apply` writes the values and triggers `save_configuration` on each
drive (drives reboot one at a time, with a 3 s pause to let them
re-appear on the bus before the next write). The previous values
are recorded in `~/.stable_bot_odrive_ff_backup.json` keyed by CAN
node id, so `--revert` always has somewhere to go back to.

The values to apply are at the top of the script in the `DEFAULT_TARGETS`
list — edit those for tuning iterations.

## Pi prerequisites

**One-time setup of the odrive Python library and udev rules:**

```bash
# library (needed by dump_odrive_endpoints.py only — the CAN script
# uses python-can directly):
pip3 install --user --break-system-packages odrive

# udev rules so non-root user can talk to ODrive USB:
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/91-odrive.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

After installing the udev rules, unplug + replug the USB cable for
them to take effect on the connected device.

## Tx_SDO message layout (for reference)

Implemented in `set_odrive_feedforward_via_can.py:SdoClient`. This
is what's on the wire so the script doesn't depend on the odrive
library:

```
RxSdo (we send to the drive):
  arbitration_id = (node_id << 5) | 0x04
  byte 0    : opcode  (0x00 = read, 0x01 = write)
  bytes 1-2 : endpoint_id (uint16 little-endian)
  byte 3    : reserved (0x00)
  bytes 4-7 : value, packed by endpoint type
              ('<f' for float32, single byte for bool, etc.)

TxSdo (drive responds, only to read requests):
  arbitration_id = (node_id << 5) | 0x05
  byte 0    : opcode (echoed)
  bytes 1-2 : endpoint_id (echoed)
  byte 3    : reserved
  bytes 4-7 : the read value, in the endpoint's native type
```

Function endpoints (e.g. `save_configuration`) are invoked by writing
to them with no value — opcode 0x01, endpoint id, then no payload.

## Adding new parameters

1. Find the parameter's full path (e.g. `axis0.controller.config.pos_gain`).
2. Add it to `TARGETS` in `dump_odrive_endpoints.py` and re-run the
   dump (USB connection — once).
3. Add it to `DEFAULT_TARGETS` in `set_odrive_feedforward_via_can.py`
   with the desired value and Python type cast.
4. Commit both changes plus the regenerated JSON.
