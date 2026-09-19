# 11 — CAD Start Guide (SolidWorks)

**Scope:** the CAD that stays in SolidWorks under the spec-driven plan (`10_spec_driven_line_plan.md`):
the drone part modifications, the stack check, and the exports that feed the spec, build123d and Gazebo.
Tooling (pallet, spider carrier, magazines, grippers) is generated in build123d (P4 of doc 10) and
comes back into SolidWorks as STEP for checking.

**Where to start:** Step 0 → Step 1 → Step 2. None of these need the press-nut decision, so you can
start today. The press-nut hole *diameter* is left as a variable and filled in later.

The order matters: **datum scheme → skeleton → plates → stack check → exports**. Everything
downstream (pallet pins, carrier pockets, gantry pick points, Gazebo frames) reads the numbers you
produce here. If you model a plate before the skeleton exists, you will redo it.

---

## Step 0: Setup (30 min)

### 0.1 Folder structure (Windows side)

```
C:\Users\Admin\Documents\drone-line-cad\
├── 00_stock\          # original Mark 4 files, copied in, then set to read-only
├── 10_skeleton\       # stack_skeleton.SLDPRT + cad_params.txt
├── 20_parts_mod\      # modified plates, arms, side plates
├── 30_assemblies\     # drone_stack_mod.SLDASM
├── 40_exports\
│   ├── step\
│   └── stl\
└── 50_drawings\       # fabrication drawings for the carbon supplier (later)
```

- Copy the stock files from `C:\Users\Admin\Documents\Drone Parts\Drone parts indiv\` into
  `00_stock\` and mark them read-only (right-click → Properties → Read-only). You never edit these.
- SolidWorks files do **not** go in git (binary, large). Back this folder up (OneDrive or an external
  drive). Only the exports in `40_exports\` enter the repo, through the mesh pipeline.

### 0.2 Document settings

For every new file: **Tools → Options → Document Properties → Units → MMGS**. Use a part template
with this already set so you don't forget.

### 0.3 Write the conventions down

Create `docs/00_cad_conventions.md` in the repo with the rules below. They are the contract between
SolidWorks, build123d and Gazebo.

| Rule | Value |
|---|---|
| Units | mm in SolidWorks and exports. Gazebo scales by 0.001 in the SDF. |
| Datum holes | **Two round holes, both Ø3.0 H7**, reamed, never shared with a fastener. The pallet carries one **round** pin and one **diamond** (relieved) pin. The diamond relief is on the pin, not the hole. |
| Datum placement | Longest available diagonal, same XY on bottom, mid and top plates, clear of the electronics footprint. |
| Datum frame | A coordinate system feature named `CS_DATUM`: origin at the round hole centre, on the bottom face, +X toward the diamond hole, +Z up. |
| Export frame | All STEP/STL exports use `CS_DATUM` as the output coordinate system. |
| Collision | STL is for visuals only. Gazebo collision is primitives from the spec. |
| Naming | `<part>_mod.SLDPRT`; exports `<part>.step` / `<part>.stl`, lowercase, snake_case (matches spec part IDs). |

> Correction to the earlier CAD plan: it said "one diamond/slot hole". Standard practice with a
> diamond pin is two identical round holes. Use a slot only if you use two round pins, and don't
> use two round pins (over-constrained).

---

## Step 1: Measure and record the stock geometry (1–2 h)

Before changing anything, capture the numbers the rest of the line depends on. Open each stock part
and use **Tools → Evaluate → Measure**. Use the physical drone you built and calipers for anything
not in the CAD.

Record into a `measurements.md` in the repo's `docs/` first, then transfer to `line_spec.yaml`.

| # | What | How | Goes to spec |
|---|---|---|---|
| 1.1 | Plate outline bounding box (bottom, mid, top, side) | Measure / Mass Properties | `parts.*.envelope` (checked later by the mesh pipeline) |
| 1.2 | Arm-screw hole pattern on bottom plate (8×) | Hole centres relative to plate centre | `parts.bottom_plate.arm_holes` |
| 1.3 | Stack screw pattern (4×), standoff positions (4×), top plate screws | Same | `parts.bottom_plate.stack_holes`, `…standoff_holes` |
| 1.4 | Plate thicknesses | Measure | `parts.*.thickness` |
| 1.5 | Stack heights: bottom → mid plate, mid → top plate, standoff length | Built drone + calipers | `parts.*.z_in_stack` |
| 1.6 | Arm root geometry, arm hole pattern at root | Measure arm part | `parts.arm.*` |
| 1.7 | ESC/FC board outline, mounting pattern, stack height with grommets | Calipers on real parts | `parts.spider.dims`, `parts.pod.dims` |
| 1.8 | Motor lead bundle diameter, sleeved | Calipers on built drone | Input for lead slot width (Step 3.2) |
| 1.9 | Masses of each plate, arm, motor, ESC, FC | Kitchen/lab scale on real parts (CAD mass is wrong for carbon unless the material is set) | `parts.*.mass` |

**Deliverable:** every `null` in `line_spec.yaml` that belongs to a stock part is filled in.

---

## Step 2: Stack skeleton (top-down master) (2–3 h)

One part holds every shared location. All plates reference it, so a datum moved once moves
everywhere. This is SolidWorks' version of the spec file.

### 2.1 Create `10_skeleton\stack_skeleton.SLDPRT`

1. **Tools → Equations → Global Variables.** Create:
   `datum_d = 3`, `datum_dx`, `datum_dy` (diamond hole position relative to round),
   `press_nut_hole_d = 4` (**placeholder** until the press nut is chosen), `lead_slot_w`,
   `lead_slot_l`, `z_mid`, `z_top`.
2. In the same dialog, tick **Link to external file** and point it at
   `10_skeleton\cad_params.txt`. The variables now live in a plain text file
   (`"datum_dx" = 62.5` per line). Later, a small script can write this file from `line_spec.yaml`,
   so the spec drives SolidWorks directly.
3. Sketch on the Top plane, one sketch per layer (`SK_Bottom`, `SK_Mid`, `SK_Top`):
   - Plate outline (converted from the stock part, or traced)
   - All existing hole centres from Step 1 as points
   - **Round datum hole** and **diamond datum hole** as circles dimensioned by the global variables
4. **Place the datum holes:**
   - On the longest diagonal that is clear on all three plates.
   - Edge distance and spacing from other holes: aim for ≥ 3× hole diameter in carbon (common
     composite guideline), then confirm with the carbon supplier.
   - Clear of the ESC/FC/pod footprint from 1.7, because the pins (or a pin-path check) run
     through the whole stack.
5. Add reference planes at `z = 0`, `z_mid`, `z_top`.
6. **Insert → Reference Geometry → Coordinate System** at the round hole centre, +X toward the
   diamond hole, named `CS_DATUM`.

### 2.2 Check

- Overlay the three layer sketches and verify that the datum holes land on material in all three
  plates.
- Ask whether standoff positions could coincide with arm-screw positions (open question §15.1 in
  the summary). If they visibly can, note it; this changes the fastener count.

**Deliverable:** `stack_skeleton.SLDPRT` + `cad_params.txt`. Datum XY goes into
`line_spec.yaml → parts.*.datums`.

---

## Step 3: Modify the plates (in this order)

For each: **File → Save As** from `00_stock\` into `20_parts_mod\` as `<part>_mod.SLDPRT`, then
**Insert → Part** to bring in `stack_skeleton.SLDPRT` (bodies off, sketches, planes and `CS_DATUM`
on). Then convert the skeleton entities into the plate's sketches, so plate features are driven by
the skeleton.

| # | File | Change | Needs decision? |
|---|---|---|---|
| A1 | `bottom_plate_mod.SLDPRT` | Datum holes (Ø3.0 H7 callout via Hole Wizard or cosmetic tolerance). Press-nut holes at the 8× arm, 4× stack, 4× standoff, side-plate and top-plate positions, **diameter = `press_nut_hole_d`**. Flange side on the underside, recessed if needed so the plate sits flat on the pallet. 3 fiducials (asymmetric). | Press-nut diameter/edge distance (update the variable later); fiducial method |
| A2 | `mid_plate_mod.SLDPRT` | Datum holes. **Motor lead exit slots** at each arm root: width = `lead_slot_w` (sleeved bundle from 1.8 + clearance), radiused edges (carbon abrades insulation), exit outboard of the plate edge. 3 fiducials. | No |
| A3 | `top_plate_mod.SLDPRT` | Datum holes at the same XY. Countersinks for the top-plate screws (drive from above). 3 fiducials. | No |
| A4 | `arm_mod.SLDPRT` | **Asymmetric key** at the root so a rear arm cannot seat in a front nest/magazine. Root tooling holes for the arm-prep nest and spider carrier. Pick flat if the gripper needs one. | No |
| A6 | `side_plate_mod.SLDPRT` | Key so inside/outside can't be swapped in the magazine. | No |

Rules while modeling:
- Every new dimension that another part depends on becomes a **global variable in the skeleton**,
  not a local number in the plate.
- Before continuing, check **Tools → Evaluate → Mass Properties**. Set the material so the mass is
  realistic, or override it with the measured value from 1.9.

**Start with A2 (lead slots) and the datum holes on A1–A3.** They depend on no open decision. Do
the press-nut holes last and leave them at the placeholder diameter.

---

## Step 4: Stack check assembly `drone_stack_mod.SLDASM`

This is the validation step. It replaces the old line-layout assembly, for the stack only.

1. Insert `stack_skeleton` first and **fix** it. Everything mates to it.
2. Add two simple **placeholder pins** (plain cylinders, Ø3, length = full stack height + 5 mm
   lead-in). The real pins come later from build123d (B1).
3. Insert A1, A2, A3: concentric mates on both datum holes to the pins, coincident to the layer
   planes.
4. Insert 4× A4 arms, standoffs, and envelope boxes for the ESC, FC/pod and motors (from 1.7).
5. **Tools → Evaluate → Interference Detection** → no interferences except intended
   screw/nut overlaps.
6. **Pin path check:** the pins must pass through every layer without hitting arms, ESC, pod or
   leads.
7. **Mid-plate drop check (main line step 3):** place the mid plate above the arms, then use
   **Move Component → Collision Detection → Stop at collision** and drag it down along Z. It must
   reach its seat, passing between the arms and the suspended ESC, with the lead slots clearing the
   lead bundles.
8. **Top-plate countersink check:** the screws drive from above and nothing needs a flip.

**Deliverable:** a clean stack, plus a screenshot of the Interference Detection result for the
decision log.

---

## Step 5: Exports

For each modified part (A1–A4, A6):

1. **STEP:** File → Save As → STEP AP214 → **Options → Output coordinate system: `CS_DATUM`**
   → `40_exports\step\<part>.step`.
   build123d (P4 in doc 10) imports these to derive the pallet and carrier geometry.
2. **STL:** File → Save As → STL → Options:
   - Output as: **Binary**
   - Unit: **Millimeters**
   - Resolution: **Fine** (the mesh pipeline decimates later)
   - **Tick "Do not translate STL output data to positive space"**
   - Output coordinate system: **`CS_DATUM`**
   → `40_exports\stl\<part>.stl`
3. From WSL, the mesh pipeline reads the exports directly:
   `python tools/mesh_pipeline.py --src /mnt/c/Users/Admin/Documents/drone-line-cad/40_exports/stl`
   It scales, decimates, checks that the origin sits at the round datum, and compares the bounding
   box against the spec. A mismatch means the CAD and the spec disagree, so fix one of them.

---

## Step 6: Close the loop with the generated tooling

After build123d produces the pallet and spider carrier (doc 10, P4):

1. Insert `pallet.step` and `spider_carrier.step` into `drone_stack_mod.SLDASM`.
2. Replace the placeholder pins with the generated pins.
3. Run Interference Detection again. The carrier's arm pockets were computed from the spec's
   `arm_holes`. If they don't line up with A1's holes, the spec values from Step 1 were transcribed
   wrong.
4. Check gripper clearance around the carrier's pick features.

---

## Step 7: Fabrication drawings (after the press nut is chosen)

In `50_drawings\`, for A1–A3:
- Outline + **Hole Table** (Insert → Tables → Hole Table) referenced to `CS_DATUM`.
- H7 callouts on datum holes; the press-nut hole size and tolerance from the nut datasheet.
- Note: press nuts installed by the supplier, flange side on the underside.
- Fiducial marking method (white ink or contrasting insert, per summary §7.3).
- Edge finish/deburr spec (summary: deburring belongs on the supplier drawing).
- Export DXF of the outline for the carbon cutter.

---

## Later CAD (not a start task)

| Item | Summary ref | Why later |
|---|---|---|
| Camera TPU snap mount | §7.6 | Sub-line B, not on the main-line critical path |
| Pod antenna mounts | §7.7 | Waits for the variant envelope numbers (§15.2) |
| Servo release mount | §7.8 | Parked; constraint: must fasten from above or horizontally |

---

## Critical path

Step 0 → Step 1 (measure) → Step 2 (skeleton + datum placement) → A2 lead slots + A1–A3 datum
holes → Step 4 stack check → Step 5 exports → *(build123d pallet + carrier, doc 10 P4)* → Step 6 →
press-nut decision → A1 press-nut holes → Step 7 drawings.

**Hardest single decision:** datum hole placement in Step 2.4. It has to be on material in all three
plates, clear of the electronics, with a clean pin path through the stack. Every fixture in the line
references it, so spend the time there.

## Blocking decisions

| Decision | Blocks | Can proceed meanwhile? |
|---|---|---|
| Press-nut part number (hole Ø, edge distance) | A1 press-nut holes, Step 7 | Yes, with the `press_nut_hole_d` placeholder |
| Fiducial method (ink vs. insert) | Fiducial geometry on A1–A3 | Yes, place them as sketch points first |
| Standoff / arm-nut merge | A1 hole count, gang-head pattern | Yes, check it visually in Step 2.2 |
