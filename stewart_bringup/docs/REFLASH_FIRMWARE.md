# Reflashing all 6 ODrive Pros to uniform firmware (Path B)

Why this needs to happen: the CAN probe (`probe_odrive_config_via_can.py`)
showed that nodes 2 and 5 are running a different firmware build than
nodes 0/1/3/4 — different endpoint count (200 vs 138 in 0..200),
different metadata at endpoint 0, different SDO response framing.
Until all six are on the same firmware, our CAN-only parameter writes
silently land on wrong endpoints on those two drives. Reflashing
harmonizes them; everything CAN-side starts working uniformly.

This procedure uses USB DFU (one cable, swap between drives, ~30 s
each). Read the **whole document** before starting — there are
points where you can't hit Ctrl-C without consequences.

---

## Pre-flight checklist

Don't begin until all of these are true:

- [ ] You're at the bench with physical access to all 6 ODrives.
- [ ] All 6 ODrives are powered on and connected to the CAN bus
      (you can keep them on; they're powered separately from USB).
- [ ] You have a USB cable that's been working for parameter writes
      (the same one you used for the endpoint dump).
- [ ] The Pi has at least 100 MB free in `/tmp/` for backup JSONs
      (`df -h /tmp/` to verify — should be fine).
- [ ] You can SSH into the Pi from your laptop (so you can run
      `odrivetool` commands while watching the LEDs on the bench).
- [ ] Internet on the Pi (DFU can fetch the firmware from ODrive's CDN).
- [ ] You have ~30 minutes uninterrupted. Don't do this in 5-minute
      bursts — half-flashed drives are recoverable but inconvenient.

---

## Step 0 — Stop the running stack

The control node owns CAN bus and could fight DFU traffic; the GUI
service holds onto port 9090.

```bash
ssh sorak@10.31.1.98
sudo systemctl stop stable_bot stable_bot_gui
sudo systemctl status stable_bot stable_bot_gui --no-pager | head -10
# expect Active: inactive (dead) for both
```

---

## Step 1 — Confirm what firmware is currently on each drive

Plug USB to one ODrive at a time and read the version. **Do all 6
before moving on** — this catalogues what you have.

```bash
# For each drive in turn:
odrivetool   # opens interactive shell, finds the USB-attached drive
# In the shell, paste:
print(f"sn={hex(odrv0.serial_number)}, hw={odrv0.hw_version_major}.{odrv0.hw_version_minor}, fw={odrv0.fw_version_major}.{odrv0.fw_version_minor}.{odrv0.fw_version_revision}, unreleased={odrv0.fw_version_unreleased}, commit={hex(odrv0.commit_hash)}, can_node={odrv0.axis0.config.can.node_id}")
exit()
```

Write down what each drive reports. Specifically:
- Serial number (so we can correlate to backups later)
- `fw_version_major.minor.revision`
- `fw_version_unreleased` (suffix; e.g. "1" → fw 0.6.11-1)
- `commit_hash` (the bit that *really* identifies the build)
- `can_node` (which CAN node ID this drive answers as)

You'll likely see two distinct `commit_hash` values: one shared by 4
drives and another shared by the other 2. **That confirms the firmware
mismatch directly.** If by some chance all 6 share a commit_hash,
the firmware is uniform and the issue was config drift only — in
that case skip to Step 5 (config restore from a known-good backup),
no DFU needed. (Probably you'll see two distinct hashes given the
probe results.)

The CAN node ID is what `set_odrive_feedforward_via_can.py --node-ids`
addresses. So you'll be able to map: "the drive with serial X is at
node Y" — useful when restoring configs in Step 5.

---

## Step 2 — Pick a target firmware version

Choose ONE firmware version to put on all six. Two reasonable picks:

**Option A (recommended): Whatever the 4 "good" drives already run.**
Their behavior under iter-3 gains was characterized in the bag we
already pushed. Keeping their firmware unchanged means that data
remains valid; only nodes 2 and 5 start behaving consistently with
the rest. Use the `fw_version_*` you wrote down for one of nodes 0,
1, 3, or 4.

**Option B: Latest stable.** `odrivetool new-dfu` without a version
arg auto-fetches latest. Cleaner long-term, but invalidates every
sweep we've collected because the *plant* changes (4 drives end up
on different firmware than they were before).

Go with **A** unless you have a specific reason to upgrade. If A,
note the exact version string from your Step 1 readings — you'll
need it for the `--version` flag.

---

## Step 3 — Back up every drive's config (mandatory)

ODrive Pro 0.6.x DFU **erases user configuration**: motor calibration,
encoder offsets, leg-limit-related fields, CAN node IDs, the whole lot.
You need backups before flashing, and you'll need them all in a known
location keyed by serial number so you can restore the right one to
the right drive.

```bash
mkdir -p ~/odrive_backups_$(date +%Y%m%d)
cd ~/odrive_backups_$(date +%Y%m%d)

# For each drive (6 cable swaps — one per drive):
# 1. Plug USB to a drive.
# 2. Run:
odrivetool   # shell opens
sn=hex(odrv0.serial_number)
node=odrv0.axis0.config.can.node_id
print(f"backing up sn={sn}, node={node}")
exit()

# 3. Outside the shell, save with a self-describing filename:
odrivetool backup-config odrv_node<NODE>_sn<SN>.json
# e.g. odrv_node0_sn0x347d35583233.json
# (Replace <NODE> with the can_node you saw, <SN> with the serial.)

# 4. Verify the file exists and has plausible size (~10-50 KB):
ls -la odrv_node*_sn*.json
```

After all 6 are backed up, **verify** with:

```bash
ls -la ~/odrive_backups_$(date +%Y%m%d)/
# expect 6 files, all keyed by node + serial
```

If you only see 5 files, you missed one — go find which drive didn't
get backed up before continuing. **Don't proceed to Step 4 with
incomplete backups.**

---

## Step 4 — Reflash each drive (the actual DFU)

This is the irreversible step. Per drive: ~30 s of flashing + ~10 s
of post-flash boot + cable swap. So ~6 minutes total on this step.

For each ODrive:

```bash
# 1. Plug USB to the drive.
# 2. Make sure it's IDLE (not armed). If you stopped stable_bot in
#    Step 0, all drives should be idle.
# 3. Run DFU. If you chose Option A in Step 2:
odrivetool new-dfu --version 0.6.11    # replace with your exact version
# If Option B (latest):
odrivetool new-dfu

# Output will scroll a lot. Watch for:
#   "Erasing flash..."           (fine, takes ~10 s)
#   "Writing flash..."           (fine, takes ~10 s)
#   "Verifying flash..."         (fine, takes ~5 s)
#   "DFU finished."              (success — drive reboots)
# OR:
#   "Error: ..."                 (failure — see "If something goes wrong")

# 4. Confirm the new firmware:
odrivetool   # shell opens, reconnects to the now-rebooted drive
print(f"fw={odrv0.fw_version_major}.{odrv0.fw_version_minor}.{odrv0.fw_version_revision}, commit={hex(odrv0.commit_hash)}")
# Verify version + commit_hash match your Step 2 target.
exit()

# 5. Unplug USB. Plug USB to the next drive. Repeat from step 1.
```

**Don't skip the version verification.** If `commit_hash` doesn't
match what you expected after DFU, the new firmware didn't take —
re-run `odrivetool new-dfu` on that same drive before moving on.

---

## Step 5 — Restore configurations

Each drive needs ITS OWN backup restored (motor calibration is
per-drive, not transferable). The filename convention from Step 3
makes this straightforward.

```bash
cd ~/odrive_backups_<date>/

# Per drive (6 cable swaps again):
# 1. Plug USB to a drive.
# 2. Get its serial:
odrivetool
sn=hex(odrv0.serial_number)
print(sn)
exit()

# 3. Find the matching backup and restore it:
odrivetool restore-config odrv_node<NODE>_sn<SN_MATCHING>.json

# (output will scroll listing every parameter restored)

# 4. Save configuration to flash (so reboots don't wipe it):
odrivetool
odrv0.save_configuration()
exit()

# 5. Unplug, plug to next, repeat.
```

**Special handling for nodes 2 and 5:** their backups contain the
weird `vel_integrator_gain ≈ 8000` and other oddities. Restoring
those re-introduces the problem after DFU. You have two choices:

**Option 1 (safer):** restore each drive's own backup (preserves
calibration including the bad config). Then immediately overwrite the
specific bad fields on nodes 2 and 5 by running our CAN configurator
in Step 8. The vel_integrator_gain on those drives will go from 8000
back to 0.05.

**Option 2 (cleaner):** for nodes 2 and 5 only, **don't restore the
backup**. Instead, keep them at factory defaults after DFU and
re-calibrate from scratch (motor calibration, encoder offset, anti-
cogging if you've been using it). This loses any tuning ODrive did
on those drives but starts clean.

Option 1 is fewer steps and the bad fields are exactly what we'll
overwrite anyway. **Go with Option 1.**

---

## Step 6 — Re-extract endpoint IDs (sanity check)

If your Step 2 target firmware version is unchanged from the four
"good" drives, the endpoint IDs in
`stewart_bringup/data/odrive_endpoints.json` should still be correct.
Verify by re-running the dump:

```bash
# Plug USB to any drive (they're all identical now).
python3 ~/stable_bot_repo/stewart_bringup/scripts/dump_odrive_endpoints.py

# Compare the new JSON to the existing one:
diff ~/stable_bot_repo/stewart_bringup/data/odrive_endpoints.json \
     <(cat ~/stable_bot_repo/stewart_bringup/data/odrive_endpoints.json.new 2>/dev/null) || \
     echo "(no .new file — script wrote in place)"

# If the IDs changed (you upgraded firmware version), commit the new file:
cd ~/stable_bot_repo
git add stewart_bringup/data/odrive_endpoints.json
git commit -m "Refreshed endpoint IDs after fw reflash to <version>"
git push
```

Expected: if you stuck with Option A in Step 2, IDs are unchanged.
If you went with Option B (latest), they may shift slightly.

---

## Step 7 — Verify uniformity with the CAN probe

This is the moment of truth. After reflash, all 6 drives should have
the same endpoint table.

```bash
# Unplug any USB cable. Re-plug the USB-CAN adapter to the Pi if it's
# disconnected (so you can talk to all 6 drives over CAN at once).
sudo systemctl stop stable_bot   # in case it auto-started

python3 ~/stable_bot_repo/stewart_bringup/scripts/probe_odrive_config_via_can.py \
    --start 0 --end 600 \
    --good-node 0 --bad-node 2

# Look at "responsive endpoints per node:" line:
#   BEFORE reflash: {0: 521, 2: 600}      (different!)
#   AFTER reflash:  {0: ~600, 2: ~600}    (same! — mismatch ≤ a few)

# Look at "ENDPOINTS WITH DIFFERENT VALUES":
#   BEFORE: ~388 differences
#   AFTER:  ≤20 differences (just runtime values: vbus, encoder counts,
#                           fault counters — NOT config fields)
```

If you still see hundreds of differences after reflash, something
went wrong with one or more drives' DFU — re-check the
`commit_hash` for each drive (Step 1's command) and re-flash any
that don't match.

---

## Step 8 — Apply FF + verify CAN configuration works on all 6

This is what we've been chasing. With uniform firmware, the existing
CAN configurator will configure all 6 drives correctly:

```bash
# Stack still stopped from Step 7. If not:
sudo systemctl stop stable_bot

# Apply FF on all 6 (will work this time on nodes 2 and 5):
python3 ~/stable_bot_repo/stewart_bringup/scripts/set_odrive_feedforward_via_can.py --apply

# All "verify" lines should now print ✓ (not ✗) for all 6 drives.
# All vel_integrator_gain values should read back as 0.05 on subsequent --status.

python3 ~/stable_bot_repo/stewart_bringup/scripts/set_odrive_feedforward_via_can.py --status

# Expected: all 6 nodes show vel_integrator_gain = 0.05 and wL_FF_enable = True.

sudo systemctl start stable_bot stable_bot_gui
```

---

## Step 9 — Run the actual A/B sweep

Now (and only now) the data we collect represents iter-3 gains with
FF enabled across a uniform 6-drive setup. From the GUI:

1. Arm + Level ON.
2. Confirm IMU `err roll/pitch` settle within ~0.1° (eyeball check).
3. Run the auto-sweep: Z=25..55 step 5, 8 step trials, defaults.
4. Push the digest with a clear name:
   ```bash
   cd ~/stable_bot_repo
   git add tuning_data/
   git commit -m "Iter-3 sweep — FF enabled, post-firmware-reflash"
   git push
   ```
5. Drop the URL — that's the data point we've actually been working
   toward this whole thread.

---

## If something goes wrong

### DFU fails partway through

Symptom: `odrivetool new-dfu` errors out with "Verifying flash..." or
similar, drive doesn't reboot cleanly.

Recovery: ODrive Pro has built-in DFU recovery mode. Power-cycle the
drive while holding the DFU button (small tactile switch on the PCB —
check ODrive Pro docs for exact location, usually labeled "DFU"). It
will boot into pure-bootloader mode, USB-detectable. Re-run
`odrivetool new-dfu` against it. The flash gets rewritten cleanly.

### Drive doesn't enumerate over USB after DFU

Symptom: `odrivetool` hangs at "waiting for ODrive". `lsusb | grep 1209`
shows nothing.

Recovery: same as above — power-cycle into bootloader recovery
mode, re-flash.

### Backup restore fails with "no such field"

Symptom: `restore-config` complains about fields that don't exist on
the new firmware.

That's normal if your target version differs from what was backed up;
ODrive's backup format is best-effort across versions. Most fields
restore; a few may not. The unrestored fields will be at their
factory defaults — usually fine for non-tuned parameters.

### After Step 8, set_odrive_feedforward_via_can.py still shows ✗ verify

Means a drive's still on different firmware. Re-run Step 1's version
check on whichever node failed; reflash that drive specifically.

### Drive becomes unresponsive on CAN after reflash

Symptom: probe shows fewer than 6 responsive nodes, or one specific
node never replies.

Likely cause: the restored config didn't include the CAN node_id
(this happens if backup was taken under a different fw revision).
Recovery: USB-connect to the silent drive, run:
```bash
odrivetool
odrv0.axis0.config.can.node_id = <correct number>
odrv0.save_configuration()
exit()
```
Cycle power on the drive. It should respond on the bus again.

---

## After everything works

Update memory + push a commit noting the firmware version we
standardized on, so future sessions know what's running. I'll add a
note in `MEMORY.md` once you confirm the reflash went clean.

Total wall-clock time including backups + reflashes + restores:
~30 minutes if nothing goes wrong, ~60 if you have to reflash one
drive twice because of a hiccup. Plan accordingly.
