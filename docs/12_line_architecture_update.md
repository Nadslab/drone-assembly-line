# 12 — Line architecture update (October 2026)

**Status:** current architecture. **Date:** 2026-10-02.
**Supersedes:** the sections listed in §8 of this file, in `drone_assembly_line_summary.md` and
`10_spec_driven_line_plan.md`. Where those documents disagree with this one, this one wins.
Everything not listed in §8 is unchanged.

---

## 1. What changed, in one table

| Topic | Before (Sep 2026 docs) | Now |
|---|---|---|
| Pick-and-place | One gantry, then zoned gantries | **One SCARA per working station** |
| SCARA model | — | **Epson LS10-B, 700 mm reach, 300 mm Z stroke** (one model everywhere) |
| Screwdriving | Gang heads / SCARA drivers | **Gang heads kept**, each on a slide so it clears the SCARA |
| Station count | 13 (10 assembly + audit, test bank, pack) | **Loader + 5 working stations (A–E)**; end-of-line deferred |
| Spider handling | Carrier rides along (summary §5) | **Unchanged and confirmed**: carrier rides along to station B |
| End-of-line (audit, test bank, pack) | In scope | **Deferred**; specs kept in `docs/deferred_stations.yaml` |
| Robot-arm / learned-control work | SO-101 / LeRobot, VLA | **Out of scope** |

---

## 2. Why one SCARA per station

- At 20.2 s takt the gantry tour (8+ picks, metres of travel) did not fit. Zoned gantries fixed the
  time but put gang heads inside gantry travel.
- A SCARA per station makes each station a self-contained cell: robot, its feeders/buffers, one gang
  head. No shared rails, no robot passing over another station's equipment.
- Industry pattern for small-product pallet lines: SCARA for short, fast pick-and-place; fixed
  multi-spindle gang heads for fixed screw patterns.

---

## 3. Main line — current station plan

Line balancing: each station combines steps until its time approaches takt with margin.
Times below are **placeholder estimates** from the dev spec (D-014):
- the SCARA runs at 0.5 × motion calibrated to Epson's 0.41 s standard cycle, so a pick-and-place
  is ~1.8–2.3 s, including the move to and from its park pose around the drive;
- a carrier/tray removal is timed as its own lift-off and put-down;
- slide in+out takes 1.0 s;
- the index is per station: 1.83 s from the last queue slot, 2.33 s after a 1.4 m gap, stop unit
  included. It was a fixed 2.0 s.

| Station | ID | Steps (in order) | Robot / head | Part sources | Est. time | % of 20.2 s |
|---|---|---|---|---|---|---|
| Loader | `st_load_pallet` | Bottom plate onto pallet at the pallet return | Sub-line C loader | `buf_plate` | — | — |
| **A — Frame** | `st_build_frame` | Pins extend → place spider **in its carrier** → place mid plate between arms and suspended ESC → drive arm sandwich (~8) | SCARA A (carrier gripper + plate vacuum) + `gh_arm_sandwich` | `buf_spider`, `fd_mid_plate` | ~10.8 s | ~53% |
| **B — ESC** | `st_seat_esc` | Install 4 standoffs (one gang pick) → ESC set onto stack positions, drive 4 → lift carrier off → empty carrier to return | SCARA B (standoff gang + carrier gripper) + `gh_esc` | `fd_standoff`, `rt_carrier_spider` | ~10.3 s | ~51% |
| **C — Pod** | `st_fit_pod` | Place pod **in its tray** → drive pod screws → lift tray off → empty tray to return | SCARA C (tray gripper) + `gh_pod` | `buf_pod`, `rt_carrier_pod` | ~10.3 s | ~51% |
| **D — Sides** | `st_fit_side_plates` | Place both side plates (paired pick) → opposed horizontal heads drive (2 passes) → **servo mount (parked)** | SCARA D (pair gripper) + `gh_side_plates` | `fd_side_plate` | ~10.7 s + servo | ~53% + servo |
| **E — Top** | `st_fit_top_plate` | Place top plate → drive ~5 countersunk screws from above | SCARA E (plate vacuum) + `gh_top_plate` | `fd_top_plate` | ~8.1 s | ~40% |

**5 SCARAs** (down from 6), **5 gang heads**, all on slides.

Why 5 working stations, not 4: the design summary §5 includes three steps the first merge missed —
pod screws, spider-carrier removal and pod-tray removal. With them, a combined ESC + pod station is
~24.5 s, over takt (D-013 speeds). With the D-014 estimate the work of B plus C is
8.46 + 6.93 s, plus one 1.83 s index: ~17.2 s, inside takt. A merge would also need two gang heads
at one station, which the one-head rule (D-012) forbids. Those speeds rest on a placeholder derate
and a non-physical calibrated v_max (§7), so do not merge on them. The spec makes the merge a one-line change, and the takt
check will say whether it fits.

### Spider carrier (unchanged from summary §5, confirmed)
- The carrier holds the 4 arms at final spacing and the ESC at its final height above the mid-plate
  line. It travels with the drone from station A to station B.
- Arms are located by **retractable round + diamond pins** through **dedicated tooling holes**
  (summary §7.2, §6.4) — never through screw holes, which stay free for the gang head.
- Because the arms are screwed at station A before the pallet moves, nothing is loose during
  indexing.
- At station B the ESC is fastened, then the carrier is lifted off and returned to sub-line A on an
  **empty-carrier return path** (`rt_carrier_spider`). The pod tray follows the same pattern from
  station C (`rt_carrier_pod`).

---

## 4. SCARA selection — Epson LS10-B, 700 mm, Z 300 mm

Verified against Epson's own product pages (Oct 2026):

| Requirement | Need | LS10-B 700 | OK? |
|---|---|---|---|
| Reach | ~0.7 m (2–3 sources + pallet per station) | 700 mm (J1+J2) | ✓ |
| Z stroke | 200–300 mm (placeholder safe-Z) | 200 or 300 mm standard | ✓ (specify 300) |
| Payload | ~2.5–3.5 kg est. (dual-tool end-effector + spider in carrier) | Rated 5 kg / max 10 kg | ✓ (estimate — carrier mass unknown) |
| Repeatability | Pins Ø3 H7 with lead-in | ±0.02 mm (J1, J2) | ✓ |
| Wrist rotation | Dual-tool switch by J4 | ±360° | ✓ |
| Wrist inertia | Spider span ~0.3 m on J4 | **Rated 0.02 kg·m²** | **Check in CAD** |
| Air to tool | Vacuum + gripper | ø6 ×2, ø4 ×1 | ✓ |
| Cycle time | Short picks | 0.41 s standard cycle | ✓ |
| Mounting | Beside conveyor | Table top; J1 ±132°, J2 ±150° (not a full circle — face the work area toward the line) | ✓ |

**The one open check: wrist inertia.** A 7" spider in its carrier, spinning about J4, may approach
the rated 0.02 kg·m². Above the rated value the robot still works but must reduce J4 acceleration.
Verify with SolidWorks Mass Properties once the carrier and end-effector are modelled; keep heavy
tool mass close to the J4 axis.

Other notes:
- **ROS 2:** no official Epson ROS 2 robot driver found. Irrelevant for simulation (generic SCARA
  URDF with Epson dimensions); matters only if a real LS10-B is ever driven from ROS.
- **Price:** one non-Epson source quotes from ~€15,900 before tax; confirm with a quote.
- **CAD:** download the STEP for the exact variant from Epson's LS10-B support page (Drawings) into
  `C:\Users\Admin\Documents\drone-line-cad\05_vendor\`.

---

## 5. Build milestones (redefined)

| Milestone | Stations | Pallet state at the end |
|---|---|---|
| **1. Frame** | Loader, A | Rigid frame, arms screwed, carrier still holding the ESC |
| **2. Electronics** | B, C | ESC fastened, pod fastened, carrier and tray returned |
| **3. Closeout** | D, E | Closed airframe (servo parked) |

Milestones are build order only — stations are independent cells.

---

## 6. Deferred (re-add later)

`st_audit_torque`, the ~5-cell end-of-line test bank, test divert/branch, `st_pack`. Their spec
blocks live in `docs/deferred_stations.yaml` (not loaded by any tool). The pallet return path to
the loader stays in scope.

---

## 7. Open items

| Item | Blocks |
|---|---|
| Real SCARA motion: Epson's v_max/a_max ratio (the standard-cycle calibration gives a non-physical 14.2 m/s v_max with the dev ratio, D-014 K) and a measured `cycle_derate` | Final station times; whether B+C can merge |
| Park moves timed on TCP distance (A swings j1 101°, D-014 L); slide frame vs the arm's place paths not checked (D-014 M) | Station A time; slide frame design |
| Wrist inertia of carrier + end-effector (§4) | SCARA speed settings |
| Spider carrier geometry: bridge must leave room for the mid plate and the arm-sandwich head | Station A |
| Slide frame dimensions (post, rail) — PLACEHOLDER | Gang head design |
| Retractable pin actuation (where pins extend/retract) | Pallet design (P4 B2) |
| Carrier / tray return path routing | Layout |
| Gap B: fastener lengths for ESC, pod, side plates, top plate | Gang head design |
| Gap C: servo mount (parked) | Station D |
| AnyLogic durations, press-nut selection, standoff/arm-nut merge, carrier material | Unchanged from plan 10 §14 |

---

## 8. Sections superseded in older documents

**`drone_assembly_line_summary.md`**
- §4 "Automation primitives": gantry → SCARA per station; gang heads on slides.
- §5 "Main line" steps 2–13: station grouping and robot per §3 here; steps 11–13 deferred.
- §7.10 "New tooling": gantry tools → SCARA end-effectors (dual-tool per SCARA).
- §14 "Physical cell": "gantry placement" → SCARA placement.
- §16 "What this means for the main line": "the gantry never searches a bin" → "no robot ever
  searches a bin".

**`10_spec_driven_line_plan.md`**
- §2 naming: `gantry` row replaced by `scara_<station>`; add `rt_<carrier>` (return path) and
  capability `remove`.
- §3 spec example: `gantry:` block and per-step stations superseded by the current repo spec
  (D-012, D-013).
- P1 note "defaults for gantry axes", P5.3 gantry xacro, P5.4 gantry checks, P7 gantry
  DetachableJoint, P8a gantry motion: read "SCARA" for "gantry" throughout.
- §14 "Gantry axis spec" open item → replaced by "real SCARA motion parameters".

Decision log entries in the repo: D-009 (zoned gantries), D-010/D-011 (actor audit, station time),
D-012 (SCARA per station), **D-013 (this update)**, D-014 (park pose, real links, per-station
index, Epson calibration, slide frames).
