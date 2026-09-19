# FPV Drone Assembly Line — Project Summary

**Status:** design in progress. Architecture selected, sub-line decomposition drafted, CAD change
list specified, variant envelope defined, feeding and rework policies drafted. No fastener audit off CAD yet.

**Last updated:** 2026-09-18

---

## 1. Objective

Design a high-volume automated assembly line for FPV drones, developed and validated first in
simulation (ROS 2 + Gazebo), with one station built physically as a proof-of-concept cell.

The line serves front-line resupply. The driving requirement is **fast turnaround from component
availability to flyable airframe**, including when the required component mix changes.

**Scope:** validated simulation of the full line, plus one physical cell — **the main line** (§14).

---

## 2. Product baseline

Based on an existing Ukrainian FPV combat drone build (`Assembly_instructions.pdf`, "Defend your
country — How to build your own FPV drone"). Used as the **parts baseline**, not as a design to be
reproduced verbatim. A second reference (Mark 5X / DJI O3 freestyle build video) informs frame
assembly sequence detail.

| Subsystem | Reference build | This project |
|---|---|---|
| Frame | Mark 4, 7" | Mark 4 7" **with modified plates** (§7) |
| Motors | EMAX ECO II 2807, 6S, 1300KV (×4) | same |
| FC / ESC stack | SpeedyBee F405 V3, ESC + FC via 8-pin cable, 1000 µF cap | same |
| VTX | AKK Race Ranger 1.6 W, SmartAudio UART1 | **Any part meeting the variant envelope** (§9) |
| Antennas | Lollipop 4 V4, **SMA** | Per variant; connector from the allowed set (§9) |
| Camera | Caddx Ratel 2 | same, **snap mount** (§7.6) |
| RX | ExpressLRS Nano, 915 MHz | **Any part meeting the variant envelope** (§9) |
| Power | 6S Li-ion, XT60 | same, **battery shipped loose** |
| Propellers | HQProp 7" | **shipped loose** |
| Payload interface | Servo on FC S9, `SERVO_TILT` (drop / release) | **in — retained** |

Reference build sequence: frame build-up (M3, bolts left loose until final torque) → motor install
→ power lead and capacitor solder → motor-to-ESC solder with multimeter continuity check → VTX and
camera wiring → ELRS wiring → Betaflight configuration → props → final torque.

**Solder joint count:** 12 (4 motors × 3) + 2 (XT60) + 2 (capacitor) + 4 (VTX) + 3 (camera) +
8 (ELRS, both ends) + 3 (servo) = **34 joints.** Split: 16 in sub-line A, 18 in sub-line B.

---

## 3. Throughput, takt and line sizing

**Target: 1,000 drones per 8-hour shift, two shifts per day = 2,000/day.**

| | |
|---|---|
| Available time per shift | 28,800 s |
| Utilization assumption | 70% |
| Effective time | 20,160 s |
| **Effective takt** | **20.2 s** |

**The central consequence:** station count is set by takt, not by daily output. The second shift
doubles output on the same physical line. It does **not** reduce labor — it doubles headcount.

### Resulting cell counts

| Element | Work content | Cells needed (per shift) |
|---|---|---|
| Main line assembly | ~100 s across the sequence | ~6 stations |
| **Main line end-of-line test** | **~90 s** | **~5 parallel test cells** |
| Arm prep (A1) | ~15 s/arm, 4,000 arms/shift (5 s takt) | ~3 parallel cells |
| **Manual soldering (total)** | **34 joints × 20–30 s = 680–1,020 s** | **34–51 stations** |

**End-of-line test, not fastening, is the main line bottleneck.** At ~90 s against a 20 s takt it
needs ~5 parallel cells. Design it as a parallel test bank from the start.

**Soldering is the project.** 34–51 stations per shift, **68–102 operators across both shifts**,
depending on whether a production operator doing repetitive identical joints achieves 20 s or 30 s
per joint. Every other number in this document is rounding by comparison. See §8.

---

## 4. Architecture: marriage line with three sub-lines

Rather than one serial sequence, the line splits into parallel subassembly lines feeding a main
line, each decoupled by a WIP buffer so variance in one does not propagate.

```
Sub-line A  Propulsion module ("spider")  ─┐
            arms + motors + ESC, soldered  │
Sub-line B  Avionics pod                  ─┼─►  MAIN LINE ─► test bank ─► pack
            FC + RF + camera + servo       │
Sub-line C  Plate stack                   ─┘
            plates + threaded features
```

**Why:** it takes the 16 motor screws and all 34 solder joints off takt, puts the manual solder
operators behind a buffer, and — combined with the CAD changes in §7 — eliminates every flip *and*
every connector-mating station from the main line.

### The key decision: motors arrive pre-soldered to the ESC

The propulsion system is built as one subassembly — four arms, four motors, sixteen screws, ESC,
capacitor and XT60 — soldered and powered-tested before it touches a frame.

| | Connectorized (rejected) | Pre-soldered spider (chosen) |
|---|---|---|
| Motor connectors | 4 sets needed | none |
| Main line mating station | yes — compliant, force-feedback, likely manual | none |
| ESC pad access under FC | requires perimeter-facing connectors | non-issue; soldering precedes FC |
| Joint reliability in crash / vibration | connector is a known failure point | direct solder |
| Propulsion test before frame | resistance + drag only | **full powered spin, direction, per-motor current, ESC telemetry** |
| Subassembly handling | small, rigid, simple | large, semi-rigid, needs a carrier |
| Value at risk per unit | low | ~$110–130; see rework policy §17 |

Decisive points: the pre-soldered route **deletes a station** rather than adding tooling to serve
one, and it enables a genuine functional test of the propulsion system before it is committed to an
airframe.

### Automation primitives

- **Gantry — pick and place.** Cartesian, fixed planar work envelope. Chosen over articulated arms
  (earlier LeRobot concept) for speed and repeatability at volume.
- **SCARA / fixed-head drivers — fastening.** Vertical insertion, high in-plane speed. Gang
  multi-spindle heads where the pattern is fixed.
- **Manual — soldering (v1).** Off the main line, behind a buffer. A future version replaces this
  with an automated soldering station.
- **Simulation.** ROS 2 + Gazebo for kinematics, layout, reach envelopes, collision checks,
  sequencing and cycle time, before hardware.

---

## 5. Sub-line step sequences

### Sub-line A — Propulsion module ("spider")

**Output:** four arms with motors installed and screwed, leads soldered to a populated ESC, powered
and functionally tested.

**A1 — Arm prep** (~3 parallel cells, 4,000 arms/shift)

1. **Feed arm** from a keyed magazine (§16). Vision reads the notch and confirms front vs. rear,
   backed by a physical poka-yoke feature (§7.5).
2. **Load into nest.** Datum on the motor-mount hole pattern, not the arm outline — carbon outlines
   are cut to loose tolerance; holes are what everything downstream references.
3. **Place motor.** Bell down, align the 4× M3 pattern, from a non-ferrous tray (§16 — motors are
   magnetic).
4. **Gang-drive 4× M3.** Single cycle, blow-fed, pre-coated thread-locker screws, torque and angle
   logged per spindle.
5. **Route and dress leads.** Along the arm, sleeved or taped, left long. **Constraint:** leads must
   exit outboard of where the mid plate will land (§7.4).
6. **Press on TPU motor protector**, if retained.
7. **Pre-solder motor test.** Phase-to-phase resistance, phase-to-bell insulation, bearing drag by
   back-driving the rotor. Catches a bad motor (~$15) *before* it is soldered to an ESC. This is the
   primary lever on spider rework cost (§17).
8. **Transfer to spider carrier**, front/rear positions enforced by the carrier.

**A2 — Spider build**

9. **Load carrier.** Fixture holding four arms at final radial spacing, ESC nest at centre held at
   its final height above the future mid-plate line.
10. **Seat ESC** in the centre nest, silicone grommets fitted.
11. **Solder capacitor to ESC pads.** Heat shrink the legs.
12. **Solder XT60 leads to ESC pads.** Largest thermal joint, done while the board is warmed from
    the previous joint, per the reference build's ordering logic.
13. **Trim, strip, twist and solder 12 motor leads** to the ESC pads.
14. **Continuity check.** Contacts from different motors must not show continuity (explicit
    requirement in the reference build).
15. **Clean.** Isopropyl alcohol, flux residue removed.
16. **Powered functional test.** Current-limited supply through a bench DShot driver: spin all four
    motors, per-motor current draw, ESC telemetry, no thermal anomaly. Failures route per §17.
17. **Serialize.** UID and test results logged. Spider stays in its carrier through to the main line.

> Edge deburring belongs on the supplier drawing, not the line. Spec edge finish; reject at incoming.

### Sub-line B — Avionics pod

**Output:** flashed, configured, bench-tested FC pod. Manual, buffered 2–4 h ahead of the main line.

1. **Kit presentation.** FC, RX, VTX, antennas, camera, servo, harnesses, per-unit ESD tray. The
   kit's RX / VTX / antenna part numbers come from the active variant (§9).
2. **Mate antenna connectors** as required by the variant. If the connector is U.FL / IPEX, follow
   §9.2: manual, magnified, retention applied immediately. Preferably eliminated by purchasing
   parts with antennas pre-mated.
3. **Solder RX leads** (4 wires each end), or buy pre-wired receivers and delete this step.
4. **Solder VTX leads** (4) and **camera leads** (3) to the FC.
5. **Solder servo lead** to FC S9 (3 wires).
6. **Fit silicone grommets** to the FC; gyro arrow orientation verified by vision or poka-yoke.
7. **VTX into mount; video antenna routed** to the mount for its connector type.
8. **RX antenna into pod mount.** Every antenna terminates in the pod's TPU mount so the main line
   never handles a loose antenna.
9. **Camera into snap mount** with dampers. Captured by the side plates on the main line (§6.2).
10. **Flash and configure.** USB to a bench with N ports. Firmware plus a canned Betaflight CLI diff:
    ports, CRSF receiver, PID / rates / filters, OSD, modes, cell voltages, `SERVO_TILT`.
    ~60–90 s per unit, fully parallel, off takt.
11. **Load the variant configuration file** (§9). Receiver settings and binding, VTX table and
    channel settings, supplied by whoever owns the frequency plan. The line applies it; it never
    chooses it. Isolating the variant here keeps the rest of the factory variant-blind.
12. **Powered bench test.** Current-limited: 5V and 9V rails, VTX RF into a dummy load, video
    present, RX link against a bench transmitter per the variant config, servo actuation.
13. **Serialize.** UID, config hash, results logged. Pod into a carrier tray, and it stays there
    until screwed down.

> **Split testing note.** Motor direction is a property of the ESC–FC pairing (and is set in
> firmware, not by wiring). It is confirmed at end-of-line test, not in A or B.

### Sub-line C — Plate stack

**Output:** bottom plate with all threads live, X-plate attached.

**Recommended route (press nuts, supplier-installed — §7.1):**

1. **Receive bottom plates** with M3 press nuts already installed by the carbon supplier.
2. **Incoming sample test.** Torque-out and push-out on a sampled basis per lot.
3. **Load bottom plate**, fiducial read, revision and orientation confirmed.
4. **Place X-plate**, drive the center screw.
5. **Buffer to main line.**

**Fallback route (bonded nutplate — only if press nuts fail qualification):** adds surface prep,
adhesive dispense, nutplate placement, cure, and pull-test steps. See §7.1 and §18.

> **Standoffs are NOT installed here.** The arms sit between the bottom plate and the mid plate, and
> standoff screws pass up through both. A standoff threaded on here blocks the mid plate from
> dropping over the arms. Standoffs move to the main line, after the arm sandwich is driven.

### Main line — the physical build target

1. **Pallet load.** Plate stack clamped, fiducials read, per-unit offsets computed.
2. **Spider placement.** Gantry transfers the propulsion module in its carrier. Four arms land on the
   bottom plate, located by retractable round-and-diamond pins through **dedicated tooling holes**
   (§7.2). Carrier continues to hold the ESC above the mid-plate line.
3. **Mid plate placed** over the arms, passing between the arms and the suspended ESC. Motor leads
   route outboard through the lead-exit slots (§7.4).
4. **Gang-drive the arm sandwich** downward into the threaded features (~8 screws).
5. **Install standoffs** (rear, VTX support).
6. **Set ESC down** onto the stack positions; **4 screws driven down — no lock nuts.** Shoulder or
   depth-stop screws so the silicone grommets get controlled compression, not torque-to-spec.
   Spider carrier released and returned.
7. **Avionics pod drop-in** onto the ESC; 8-pin cable connected; pod screws driven from above.
   Carrier tray removed.
8. **Side plates / camera cage.** Horizontal screws, two opposed horizontal spindles, both sides in
   one cycle. Captures the camera.
9. **Servo / payload release mount.** Placement and fastening position to be fixed in CAD (§15).
10. **Top plate**, countersunk screws from above, battery straps fitted (battery ships loose).
11. **Torque audit**, sampled.
12. **End-of-line test — parallel bank of ~5 cells.** Current-limited power-up, **motor direction
    check** (corrected in firmware if wrong — never rewired), RX link per variant config, VTX RF,
    video, **servo actuation**, OSD render.
13. **Serialize and pack.** Kit: airframe + battery + propellers, packed loose (§12).

**Main line fastener estimate:** ~8 arm + ~4 standoff + 4 stack + ~4 pod + ~6 side plate/cage +
~2 servo mount + ~5 top plate ≈ **33**, all driving down or horizontal. At 20 s takt this closes
across ~6–7 stations.

**Zero flips and zero connector-mating stations on the main line** — conditional on §7.

---

## 6. Why the architecture is shaped this way

Record of resolved design problems, kept so the reasoning isn't relitigated.

**6.1 FC blocks the ESC motor pads — RESOLVED.** The FC mounts directly on the ESC; motor pads sit
underneath and are unreachable once it's on. The pre-soldered spider removes the problem entirely.

**6.2 Camera must precede side plates.** The camera mounts between the aluminum side plates. Pod and
camera drop in first; side plates then capture the camera in one horizontal gang cycle.

**6.3 Standoffs cannot be pre-installed on the plate stack.** They would block the mid plate from
dropping over the arms. Moved to the main line.

**6.4 Locating pins and fasteners cannot share holes.** Hence dedicated tooling holes (§7.2).

**6.5 No adhesive as a fixturing aid.** Locating pins and the spider carrier do the fixturing job.
Adhesive in the product to solve a fixture problem costs cure time, adds a dispense station, and
makes rework impossible.

**6.6 Stack lock nuts deleted.** An artifact of screws coming from below. With threaded features in
the bottom plate, screws drive down and no nuts are needed.

**6.7 Motor direction is never a rework item.** Spin direction is set in ESC firmware / Betaflight,
not by which wire goes to which pad. A "wrong direction" result at test is a configuration fix, not
a desolder.

---

## 7. CAD modification list (SolidWorks work package)

Every change below is required by the architecture in §4–§5. Grouped by part.

### 7.1 Bottom plate — threaded features  ★ highest priority

**Problem.** Mark 4 plates are ~2–2.5 mm 3K carbon. Tapping M3 directly into carbon gives roughly two
engaged threads — unreliable under vibration.

**Correction to an earlier draft:** an earlier version of this document dismissed press-fit nuts.
That was wrong. **Flanged M3 press nuts are a proven, commercial solution in carbon FPV frames** —
premium frames ship with them installed. Standard heat-set inserts (4–6 mm long) remain unsuitable;
purpose-made carbon-frame press nuts are not.

**Two options:**

| | **A. Press nuts (recommended)** | B. Bonded aluminum nutplate (fallback) |
|---|---|---|
| Process | Pressed into reamed holes | Surface prep, adhesive, cure |
| Who installs | **Carbon supplier**, or an in-house servo press | In-house, sub-line C |
| Cure / WIP | **None** | 60–500 plates in cure racks per shift (§18) |
| Mass | Lowest | Highest |
| Known failure mode | Loosens after repeated crashes | Galvanic corrosion if not isolated (§18) |
| Field repair | Nut can be tapped out and replaced | Plate is scrapped |
| Sub-line C content | Near zero | A full bonding line |

**Why A is recommended:** its known weakness — loosening after *many* crashes — matters much less
for an aircraft whose operational life is short, and it pushes the entire threaded-feature operation
onto the supplier. Sub-line C nearly disappears. B remains the fallback if A fails qualification.

**Model (option A):**
- Hole diameter and tolerance per the chosen press-nut datasheet (typically a tight reamed fit).
- Edge distance and hole spacing per the nut's recommendation — carbon cracks if a press nut sits
  too close to an edge or a neighbouring hole.
- Flange side on the underside; flush or recessed so the plate still sits flat on the pallet.
- Nut positions: 8× arm sandwich, 4× stack, 4× standoff, side plate and top plate positions.
- Qualification test: torque-out and push-out on coupons cut from the same carbon layup.

**Model (option B, if needed):** 2–3 mm **anodized** aluminum nutplate on the underside, tapped M3
at every position, keyed bond surface, 0.1–0.2 mm bond-line standoff features. See §18.

**Open:** can standoff positions be moved onto arm-screw positions so standoffs serve as the arm
nuts? On the Mark 5X these are different hole locations. If they can merge, a fastener set disappears.

### 7.2 All plates — dedicated tooling holes

- **Two per plate**, reamed, **non-fastener** — never shared with a screw.
- **One round pin, one diamond (relieved) pin.** Two round pins over-constrain and jam on tolerance
  or thermal variation.
- On the longest available diagonal for best angular constraint.
- Same pattern on bottom, mid and top plates so one pallet locates all three.
- Tolerance callout required — this is the datum for the whole main line.

### 7.3 All plates — fiducials

- **Three per plate**, asymmetric so orientation is unambiguous.
- **Contrast is the hard part.** 3K carbon is black and near-zero contrast. Laser etch alone is
  usually insufficient — plan on white ink marking or a machined pocket with a contrasting insert.
- Decide the method before committing the plate design.

### 7.4 Mid plate — motor lead exit slots

- Slots or notches at each arm root so the motor leads pass up to the ESC **outboard of the plate
  edge**, without pinching, when the mid plate drops at main line step 3.
- Size for a sleeved 3-lead bundle plus clearance, radiused edges (carbon abrades insulation).
- The single geometric feature that makes the spider architecture buildable.

### 7.5 Arms — poka-yoke and tooling

- **Asymmetric keying feature** at the arm root so a rear arm physically cannot seat in a front
  nest or magazine.
- Tooling holes at the root for the arm prep nest and spider carrier.

### 7.6 Camera mount — replace screwed cage

- **TPU snap mount with silicone dampers.** Camera clips into the pod in sub-line B, captured by the
  side plates on the main line.
- Verify camera tilt adjustability survives the change.

### 7.7 Pod antenna mounts

- TPU mount with one interchangeable antenna interface per allowed connector type, so antennas
  travel with the pod and the main line never handles a loose antenna.
- RX antenna routing must keep the U.FL joint strain-relieved (§9.2).

### 7.8 Servo / payload release mount

- Mount position, fastening direction, and servo lead routing to the FC. **Must fasten from above
  or horizontally** — a from-below servo mount would reintroduce a flip.

### 7.9 Fastener standardization

- **Pre-coated thread-locker (Nylok patch) screws throughout.**
- Minimize distinct screw lengths — each length is a feeder, a presenter and a mix-up risk.

### 7.10 New tooling to design (not frame parts)

| Item | Notes |
|---|---|
| **Spider carrier**  ★ | Holds 4 arms at final radial spacing + ESC at final height. Survives solder heat and IPA, presents all motor/XT60/cap pads to an operator, releases cleanly on the main line. |
| Main line pallet | Retractable round + diamond pins, plate clamping, fiducial-visible. |
| Gang driver heads | ~6–7 fixed patterns. |
| Arm prep nest | Datum on motor-mount holes, poka-yoke keyed. |
| Test bank fixtures | ×5 parallel, current-limited, RF dummy loads, RX link check driven by the variant config. |
| Returnable dunnage | Supplier-to-line trays for motors, arms, plates (§16). |

---

## 8. The soldering constraint

Still the dominant problem. Moving it off the main line does **not** solve it.

34 joints × 20–30 s ≈ **11–17 min of touch time per drone**, before rework.

| | Per shift | Both shifts |
|---|---|---|
| Solder stations at 20 s/joint | 34 | 34 (same line) |
| Solder stations at 30 s/joint | 51 | 51 (same line) |
| **Operators** | **34–51** | **68–102** |

**Buffering the solder work into sub-lines decouples human variance from takt but removes zero
operators.** Only connectorization or automation removes people.

Three ways out, in order of leverage:

1. **Design the peripheral joints out.** The 18 sub-line B joints (VTX, camera, RX, servo) are the
   target: plug-in harnesses, pre-wired receivers. The 12 motor joints stay direct solder by design.
2. **Pull the automated soldering station forward.** The spider presents 12 identical motor-lead
   joints at fixed coordinates in a rigid carrier — the most automatable solder task in the build.
3. **Accept manual soldering and plan the headcount explicitly.** 68–102 operators is a real
   organization: recruitment, training, ESD discipline, rework loops, quality variance.

---

## 9. Component variants — interface-based envelope

### 9.1 Principle: the line is band-agnostic

The frequency plan (which bands, which channels, which radio hardware) is **specified by the
customer, not chosen by the line**, and it changes often — potentially every few weeks. A line
designed around any particular band is out of date before it is commissioned.

So the line is designed around an **interface envelope** instead. Any receiver or VTX that meets the
envelope below can pass through sub-line B with no retooling. Changeover means swapping a kit and a
configuration file.

This also defines the thesis metric: **changeover time when the spec changes**, independent of what
the spec is or how often it changes.

### The variant envelope

| Interface | Rule | Where it's enforced |
|---|---|---|
| **Mechanical** | Fits within a defined footprint and height inside the pod mount | Pod mount CAD (§7.7); kit tray |
| **Electrical — power** | Runs off the FC's existing 5V or 9V rails; no added regulators | FC pad map; bench test (B12) |
| **Electrical — data** | Standard serial protocol: CRSF for the receiver; SmartAudio or Tramp for the VTX | FC UART assignment; config file |
| **Antenna** | One of a small allowed set of connector types, each with its own mount interface | Pod mount (§7.7); step B2 |
| **Configuration** | Supplied as a file (receiver settings, binding, VTX table and channels), loaded at the flash step | Step B11; config hash logged at B13 |
| **Solder interface** | Same pad count and wire count as the envelope's reference wiring, or pre-wired | Sub-line B joint count (§8) |

### How a variant change flows

1. The customer issues a variant: RX part number, VTX part number, antenna part numbers, config file.
2. **Qualification** (one-time per variant): confirm the parts meet the envelope — fit check in the pod
   mount, rail and protocol check, antenna connector on the allowed list, config loads and passes
   bench test.
3. **Release**: kits for sub-line B switch to the new part numbers; the config file is loaded into the
   flash bench.
4. **Nothing else changes.** Sub-line A, sub-line C and the main line never see the variant.

**Architectural rule:** all variant content lives in sub-line B. A part that would force a change
anywhere else is outside the envelope and needs an engineering change, not a variant release.

### 9.2 Handling U.FL / IPEX antenna connectors

The allowed connector set will very likely include U.FL (also sold as IPEX-1 / MHF1), because small
receivers commonly ship with it. This is a manufacturing concern regardless of which band a variant
uses.

**What it is.** A snap-on RF connector about 2 mm across, rated for few mating cycles, with a very
narrow window between "seated" and "socket torn off the PCB". It is the hardest operation to automate
in FPV assembly.

**Why it's manageable.**
- It lives in **sub-line B, which is manual and off takt**. A trained operator mates U.FL reliably; a
  robot does not. It is a line problem only if it reaches the main line — and it doesn't.
- **Preferred: buy parts with antennas pre-mated** — specify it as a purchasing requirement and the
  operation disappears.
- **Retention after mating.** U.FL is known to pop off under vibration and crash loads. Apply a
  retention method immediately after mating (a dab of UV-cure or hot-melt adhesive, or a retention
  clip). This is a legitimate use of adhesive: the job is retention, not fixturing.
- **Strain relief.** Route the antenna lead so loads go into the TPU mount (§7.7), not the connector.

---

## 10. What this discipline is called

- **Manufacturing system design** / **assembly system design** — the umbrella term for §4–§5.
- **Assembly sequence planning (ASP)** — feasible operation orderings from geometry and precedence
  constraints; precedence or AND/OR graphs.
- **Assembly line balancing (ALBP / SALBP)** — allocating operations to stations against takt.
- **Design for Automated Assembly (DFAA)** — DFA assuming a robot rather than a human.
  Boothroyd–Dewhurst is canonical.
- **Product–process co-design** / **concurrent engineering** — the loop actually being run here.

---

## 11. Techniques borrowed from SMT

**Transfers:** panelization (2–4 frames per pallet); fiducials (§7.3); feeders not trays — every
part at a fixed coordinate (§16); gang picking; single-sided processing as a design rule; carriers as
the datum (spider carrier is SMT's edge-clamped conveyor idea applied to a compliant assembly);
**parts arriving in feeder-ready packaging** (§16).

**Does not transfer:** reflow (no batch analogue for mechanical fastening — which is why every screw
designed out beats any robot bought); part rigidity (carbon, TPU and silicone are compliant;
compliance goes in the gripper and fixture, not the trajectory).

**Gang driving** (from automotive): multi-spindle nutrunners, 4–6 screws per cycle, blow-fed.

---

## 12. Ship state — DECIDED

**Battery and propellers ship loose.**

- Propeller installation station deleted — 4 nut-running operations off the main line.
- Straps are fitted; the pack is not.
- Lithium packs shipped loose vs. installed fall under different transport rules (UN3480 vs UN3481).
  Shipping loose keeps that decision separable from the line.
- Packaging handles a kit (airframe + battery + propellers), not a single unit.

---

## 13. Next steps

1. **Build the precedence graph.** Validates the §5 split and feeds everything downstream.
2. **Select and qualify the press nut** (§7.1). Get datasheets for 2–3 carbon-frame M3 press nuts,
   design coupons, run torque-out / push-out. Fall back to the nutplate only on failure.
3. **Write the variant envelope spec** (§9) — footprint, rails, protocols, allowed antenna
   connectors, config file format — and design the pod mount (§7.7) against it.
4. **Model the §7 CAD package**, in priority order: threaded features (7.1) → mid plate lead slots
   (7.4) → tooling holes (7.2) → spider carrier (7.10) → servo mount (7.8).
5. **Decide the soldering route** (§8).
6. **Freeze the CAD changes, then count fasteners off CAD** split by drive direction.
7. **Line balance** against the 20.2 s takt, including the ~5-cell test bank.
8. **Model the main line in ROS 2 / Gazebo**, then build it (§14).

---

## 14. Physical cell — DECIDED: main line

The main line is the physical build target. It demonstrates gantry placement, fixture and pallet
integration, gang driving, and the marriage of three subassemblies.

**Implication:** sub-lines A, B and C are simulated and their outputs presented to the physical cell
as pre-built subassemblies. The spider carrier and pallet (§7.10) must therefore be physically built
even though sub-line A itself is not.

---

## 15. Open questions

1. **Standoff / arm-nut merge** (§7.1). Gates the main line fastener count.
2. **Variant envelope dimensions.** Pod mount footprint and height, and the allowed antenna connector
   list — the numbers that make §9 concrete.
3. **Servo mount position and payload release geometry** (§7.8). **Parked** — finishing step, to be
   resolved later. Constraint to hold meanwhile: it must fasten from above or horizontally.

### Resolved
- Throughput → 1,000 per shift, two shifts (§3).
- Ship state → battery and propellers loose (§12).
- Physical cell → main line (§14).
- Payload interface → in (34 solder joints).
- Frequency → **band-agnostic line**; variants handled by interface envelope (§9). The frequency plan
  is a customer input, not a line design decision.
- Threaded features → press nuts recommended, nutplate as fallback (§7.1).
- Feeding strategy → §16. Rework policy → §17.

---

## 16. Feeding strategy

No single strategy — the right one depends on the part. The deciding properties are **size,
symmetry, fragility, count per drone, and whether a human or a robot takes it next.**

### Decision rules

1. **Small, high-count, near-symmetric → bulk feed** (vibratory bowl or step feeder).
2. **Large, asymmetric, or fragile → tray or magazine**, oriented by the container, not the robot.
3. **Anything going to a human → kit** (per-unit tray).
4. **Subassemblies → returnable carrier** that also serves as the fixture.
5. **Push orientation upstream.** The best feeder is a supplier shipping parts already oriented in
   your tray. This is SMT's real lesson: components arrive in tape-and-reel, never loose.

### By part

| Part | Per drone | Feeding | Why |
|---|---|---|---|
| M3 screws (all lengths) | ~50 | **Blow-feed** (bowl → tube → driver head) | High count, symmetric. One feeder per length — another reason to minimize lengths (§7.9). |
| Standoffs | ~4 | **Step or bowl feeder** | Cylindrical; end-orientation needed only if threaded asymmetrically. |
| Motors | 4 | **Thermoformed non-ferrous tray** | Heavy, asymmetric, **magnetic** — they attract each other and steel tooling. Trays must be non-ferrous and spaced. |
| Arms | 4 | **Keyed gravity magazine** | Flat and stackable. The poka-yoke key (§7.5) enforces front/rear in the magazine itself. |
| Bottom / mid / top plates | 1 each | **Stacked magazine with separation** | Flat carbon stacks well but slides; needs a separator or edge-pick. |
| Side plates | 2 | **Keyed magazine** | Have an inside and outside (reference build warns about this); key enforces it. |
| ESC, FC, RX, VTX, camera, servo | 1 each | **Kitted ESD tray** | Goes to human operators in sub-lines A and B. |
| Capacitor, XT60 pigtail | 1 each | **Kitted** at the solder station | Human-handled. |
| Spider | 1 | **Returnable carrier** | Carrier is the fixture (§7.10). |
| Avionics pod | 1 | **Returnable carrier tray** | Harnesses and antennas can't be gripped loose. |
| Plate stack | 1 | **Main line pallet** | Pallet is the datum. |
| Battery, propellers | 1 / 1 set | **Kitted at pack-out** | Never enter assembly. |

### What this means for the main line

Every main-line input is either **blow-fed, magazine-fed, or carrier-borne**. The gantry never
searches a bin — it goes to a fixed coordinate every cycle. That is what makes 20 s takt achievable.

**Supplier dunnage is a procurement requirement, not a line detail.** Specify motors, arms and
plates to arrive in returnable trays or magazines. Every part that arrives loose in a bag becomes a
decant operation someone has to staff.

---

## 17. Spider rework policy

### The problem

A spider contains roughly **$110–130 of parts** — four motors (~$15 each), the ESC half of the stack
(~$40–45), four arms, capacitor, XT60 — permanently joined by 16 solder joints. When it fails the
powered test at A2.16, the question is: repair, partial salvage, or scrap?

### Failure modes at A2.16 and the default response

| Failure found at powered test | Likely cause | Response | Rough labor |
|---|---|---|---|
| Motor doesn't spin / stutters | Cold joint, bridge | **Reflow in-station** | ~1 min |
| Short / smoke stopper trips | Solder bridge, debris | **Clean and reflow in-station** | ~2 min |
| Wrong spin direction | Configuration | **Firmware fix — never desolder** (§6.7) | ~10 s |
| One motor bad (bearing, winding) | Escaped A1.7 test | **Rework bench:** desolder 3 leads, swap arm module, resolder | ~5 min |
| ESC dead or damaged | Component defect, overheated pad | **Salvage:** scrap ESC, recover motors and arms | ~8 min |
| Lifted pad | Excess heat, repeat rework | **Scrap ESC**, salvage motors and arms | — |

### The decision rule

**Repair if labor + requalification is less than the value of what you'd lose.** In practice:

- **Joint faults → always repair, in-station.** Cheap, fast, no part lost.
- **Motor faults → repair at an off-line rework bench.** Swapping one arm module is ~5 min against a
  ~$15 motor plus the whole spider's value at risk. Worth it.
- **ESC faults → scrap the ESC, salvage the rest.** Desoldering 16 joints costs more than a new ESC,
  and a reworked ESC is a reliability risk in a crash-loaded aircraft.
- **Hard limit: one rework per ESC pad.** Repeated heating lifts pads and weakens joints. A second
  failure on the same pad → scrap the ESC.
- **Every reworked spider re-enters at A2.16.** No reworked unit skips the powered test.

### The real lever: prevention, not rework

- **A1.7 (pre-solder motor test) is the most important step in sub-line A.** Every motor fault caught
  there costs a $15 part and zero labor. Every motor fault that escapes costs a rework bench cycle.
- **Incoming ESC inspection** — at minimum a sampled powered check before soldering.
- **Track first-pass yield at A2.16.** If motor-related failures there exceed a small fraction, A1.7
  isn't catching enough and needs tightening. Rework should be a signal, not a routine.
- **Rework happens off-line, at a dedicated bench.** It never enters the main line flow.

---

## 18. Threaded-feature bonding (fallback detail)

*Applies only if the press nut (§7.1 option A) fails qualification and the bonded aluminum nutplate
(option B) is needed.*

### Why cure time sets the buffer

At 1,000 plates/shift, the line consumes **125 plates per hour.** Every hour of cure time is 125
plates sitting in racks:

| Adhesive / cure | Plates in cure (WIP) | Notes |
|---|---|---|
| Room-temp epoxy, ~4 h handling strength | **~500** | Racks, floor space, and a half-shift of inventory at risk if a batch is bad |
| Heat-accelerated epoxy, ~30 min | **~63** | Requires a cure oven, but WIP drops 8× |
| Fast methacrylate, ~15 min fixture | **~31** | Fastest; strong odor, ventilation required |
| **Press nuts (option A)** | **0** | No cure |

With two 8-hour shifts, there is an 8-hour idle window every day. A room-temperature cure could be
batched overnight, but that means a full day's plates — ~2,000 — curing at once, and a bad batch
costs a whole day of output.

### Adhesive families

- **Two-part structural epoxy.** Highest strength and environmental durability; long work life;
  slowest cure unless heat-accelerated. The conservative choice.
- **Toughened methacrylate (MMA).** Fast fixture, tolerant of less-than-perfect surface prep, bonds
  aluminum and composites well. Needs ventilation.
- Avoid cyanoacrylate — brittle, poor peel and impact resistance in a crash-loaded joint.

### Surface preparation (where most bond failures come from)

- **Carbon:** abrade to remove the glossy resin skin (fine abrasive pad or ~180 grit), then solvent
  wipe with IPA. Don't cut into the fibres.
- **Aluminum:** abrade or grit-blast, solvent wipe; a primer or silane treatment improves durability.
- **Bond soon after prep** — prepared surfaces recontaminate within hours.
- **Bond-line control:** 0.1–0.2 mm standoff features in the nutplate (or glass beads in the
  adhesive) keep the gap repeatable instead of clamp-dependent.

### Galvanic corrosion — the non-obvious risk

**Carbon fibre and aluminum in contact form a galvanic couple; the aluminum corrodes**, especially
with moisture or salt exposure. For an aircraft stored and used outdoors, this is a real failure
mode.

- **Anodize the nutplate** — the oxide layer is an insulator.
- **Keep the bond line continuous** — the adhesive layer itself isolates the two materials; a
  starved bond with metal-to-carbon contact defeats it.
- Tap threads after anodizing, or mask them.

### Acceptance criteria

- **Torque-out:** every thread must hold a proof torque without spinning. FPV M3 fasteners are
  typically tightened well under 1 N·m, so a proof of ~2× the assembly torque is a sensible start.
- **Push-out:** the nutplate or nut must not separate under an axial proof load.
- **Sampling:** per adhesive batch and per shift, on witness coupons made alongside production.
- **Lot hold:** if a coupon fails, the whole cure batch is quarantined — which is exactly why a
  500-plate cure batch is a risk and a 31-plate one is not.

---

## Sources

- [Example nano receiver shipping with a U.FL antenna — GetFPV](https://www.getfpv.com/radiomaster-rp1-v2-expresslrs-2-4ghz-nano-receiver-w-65mm-ufl-t-antenna.html)
- [Installing and removing press nuts on FPV frames — T-Hobby](https://www.t-hobby.com/blog/fastest-way-to-install-and-remove-press-nuts-on-fpv-frames)
- [M3 press nuts for carbon FPV frames — Jinjiuyi](https://www.jinjiuyi.net/nut/M3-Press-Nut-for-FPV-Racing-Drone-sunk-nut.html)
