# Stewart Platform — Hardware Documentation

This document covers the physical hardware for the Stable-Bot Stewart platform: electrical wiring, bill of materials, 3D-printed parts, and raw material specs. All referenced files live alongside this document unless a repo path is noted.

---

## Electrical / Power Wiring

| File | Format | Description |
|---|---|---|
| `Wiring Diagrams/Power Flow Diagram.drawio` | draw.io | Editable source for the full power flow diagram |
| `Wiring Diagrams/Power Flow Diagram.png` | PNG | Rendered export of the power flow diagram |

The power flow diagram covers the full electrical chain: power supply → ODrive controllers → motors, plus the CAN bus topology and Raspberry Pi 5 connections. Open the `.drawio` file in [draw.io](https://app.diagrams.net/) to edit.

---

## Bill of Materials

| File | Format | Description |
|---|---|---|
| `BOM For Stewart's Platform.xlsx` | Excel | Full BOM with part numbers, quantities, suppliers, and cost |
| `CF Tubes for legs.txt` | Text | Carbon fiber tube lengths and specs per leg (6 legs total) |
| `inline-motor-linear-actuator-bom.webp` | Image | Reference spec sheet for the inline-motor linear actuator |

### Carbon Fiber Tube Summary (from `CF Tubes for legs.txt`)
- **Short tubes:** 0.11 m × 4 per leg, 8 mm OD / 6 mm ID (26.4 m total)
- **Long tubes:** 0.37 m × 3 per leg, 8 mm OD / 6 mm ID (6.66 m total)
- **Large tubes:** 0.40 m × 1 per leg, 16 mm OD / 14 mm ID (2.4 m total)

---

## 3D-Printed Parts

All STL files are in `Printing files/`. Use Git LFS when committing these to the repo — the full set is ~135 MB.

### Leg Assembly (`Printing files/Leg Assem/`)
One full leg comprises:
- `Leg_Base_Piece_V3_InlineMotor.stl` — motor mount base
- `MotorHousing_InlineMotor_V2_BoltedCFGrippers.stl` (×2) — upper/lower motor housing
- `UpperLeg_BearingBlock_Main_V4_InlineBearings_BoltedCFGrasper.stl` — bearing block
- `UpperLeg_Bearing_Inner.stl` — inner bearing race
- `StringPulley_V2_FlatProfile.stl` — string pulley
- `Pulley.stl` + `Pulley_Spacer.stl` — drive pulley set
- `SUJ_Inner.stl` + `SUJ_Outer.stl` + `SUJ_Leg_Cap.stl` — spherical universal joint (SUJ) components
- `CFTube_Bottom_EndCap_V4_For_SUJ.stl` — CF tube bottom end cap
- `Magnetic_Joint_Ball.stl` + `Magnetic_Joint_Magnet.stl` — magnetic joint pair
- `TPU_Bearing_Sleeve.stl` — TPU bearing sleeve (flexible filament)
- `M3_to_6mm_Spacer.stl` — spacer
- `Leg Base Attachment.stl` — base attachment bracket

### Revised End Caps (`Printing files/New End Caps & Reprints/`)
Replaces earlier end cap design with improved bearing seat:
- `Top end cap 128 density with better bearing seat_6x.stl`
- `bottom endcap 128 density_6x.stl`
- `SUJ_Outer_3x.stl`

### Full Platform Assembly
- `Printing files/JBV4 Assem(1)/JBV4 Assem.obj` + `.mtl` — complete Jugglebot V4 platform mesh (OBJ format, ~490 MB, use Git LFS)
- `Printing files/JBV4 Assem.zip` — zip of the above

### McDevitt Print Sets
Organized print batches for building one complete platform:
- `Printing files/McDevitt To Be Printed/` — first batch (1 leg worth)
- `Printing files/Rest of Prints for Stewarts McDevitt/` — remainder (×5 legs, ×5 platform nodes)

---

## Configuration Notes

- `Need 6 legs in this configuration.docx` — diagram and notes on the leg arrangement pattern required for the Special 6-6 Stewart platform geometry

---

## Related Repo Paths

When committing these hardware assets to `stable_bot`, suggested destinations:

```
stewart_bringup/docs/hardware/
├── HARDWARE.md                          ← this file
├── Power Flow Diagram.drawio
├── Power Flow Diagram.png
├── BOM For Stewart's Platform.xlsx
├── CF Tubes for legs.txt
├── inline-motor-linear-actuator-bom.webp
└── cad/
    ├── Leg Assem/                       ← STL files (Git LFS)
    ├── New End Caps & Reprints/         ← STL files (Git LFS)
    ├── McDevitt To Be Printed/          ← STL files (Git LFS)
    └── Rest of Prints for Stewarts McDevitt/  ← STL files (Git LFS)
```
