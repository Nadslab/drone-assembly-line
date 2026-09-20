"""Main-line pallet, generated from line_spec.yaml (plan P4 B2).

    tools/.venv/bin/python tools/cad/pallet.py [--out DIR] [--spec PATH] [--dev]

The pallet is the datum for the whole main line (summary §7.2, `line.origin:
pallet_datum_round`), so none of its numbers are typed here: the pin positions, the pin length,
the nest size and the pallet's own footprint are read from the spec and the plates keep the
pallet honest. What *is* typed here (below) are fabrication choices local to the pallet — wall,
thickness, fit clearance — which no other part has to agree with.

Frame: the plate frame of `parts.bottom_plate`, i.e. SolidWorks `CS_DATUM` — origin at the round
datum hole, +Z up, z = 0 at the **rest plane** (the underside of the bottom plate). The body
hangs below it, the pins stand above it.

Built features:
  * two locating pins (tools/cad/pins.py) pressed through the body at `datums.round` /
    `datums.diamond`, long enough to clear the whole plate stack plus a lead-in;
  * a nest relieved below the rest plane over the whole plate footprint, so there is clearance
    under every screw site and every press-nut flange by construction — no hole pattern needed.
    The plate lands on four rest pads at the corners of its own footprint;
  * a V-notch in the leading face for the conveyor stop, so the pallet indexes to a hard stop.

The plate footprint is centred on the midpoint of the two datums: the spec gives the plate's
bbox and its datum XY, not the offset between them, and the datums sit on the longest diagonal
(§7.2), whose midpoint is the plate centre. `derived.plate_centre_mm` in the report is the number
to check against the CAD.

Outputs (`--out`, default `cad_out/`): `pallet.step` for review in SolidWorks (Step 6 of
docs/11_cad_start_guide.md) and `pallet.stl` in millimetres, put through mesh_pipeline.py so the
committed mesh obeys the 5 MB rule. Writes reports/cad_pallet.json: bbox, mass per solid, and
every derived position.

Any spec value the pallet needs that is null (or absent) is a FAIL that names the key; `--dev`
merges the placeholder overlay instead and writes to `<out>/dev/` and reports/cad_pallet.dev.json,
so a placeholder pallet can never be mistaken for a real one.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from build123d import (Align, Box, Compound, Cylinder, Polygon, Pos, Rectangle, export_step,
                       export_stl, extrude)

sys.path.insert(0, str(Path(__file__).resolve().parent))         # tools/cad: pins, common
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools: spec

import pins  # noqa: E402
from common import (Values, bbox_doc, lookup, new_report, solid, to_mesh,  # noqa: E402
                    write_report)
from common import masses as _masses  # noqa: E402
from spec import (DEFAULT_SPEC, REPO, SpecError, add_dev_argument, load_merged,  # noqa: E402
                  parse_spec, report_path)

DEFAULT_OUT = REPO / "cad_out"
PART = "pallet"

# ---- fabrication choices, millimetres. Local to the pallet: nothing else depends on them. ----
PIN_FIT_CLEARANCE = 0.010   # locating Ø under the reamed hole Ø (locational clearance fit)
PIN_LEAD_IN = 5.0           # pin above the top of the stack (guide step 4.2)
PIN_TIP_CHAMFER = 0.6       # 45° lead-in at the pin tip
PIN_SHANK_STEP = 1.5        # shank Ø = locating Ø + 2x this; the shoulder seats on the bore
PIN_SLENDER_MAX = 15.0      # free length / Ø above which a Ø3 pin is too whippy: WARN
THICKNESS = 12.0            # pallet body; the pin shanks pass through it and finish flush
WALL = 10.0                 # body beyond the plate footprint, all round
RELIEF_DEPTH = 3.0          # nest floor below the rest plane
NEST_CLEARANCE = 0.5        # relief beyond the plate footprint, per side
REST_PAD = 15.0             # square rest pad at each corner of the footprint
PALLET_GAP = 20.0           # smallest gap between two pallets standing at conveyor pitch
STOP_NOTCH_DEPTH = 6.0      # V-notch for the conveyor stop; 90° included at the vertex
DENSITY_BODY = 2700.0       # kg/m3, aluminium plate
DENSITY_PIN = 7850.0        # kg/m3, hardened steel dowel


# ---------------------------------------------------------------- geometry

@dataclass
class Layout:
    """Everything the build needs, in millimetres, derived from the spec."""

    round_xy: tuple[float, float]
    diamond_xy: tuple[float, float]
    plate: tuple[float, float]          # footprint x, y
    centre: tuple[float, float]         # footprint centre = datum midpoint
    stack_h: float                      # rest plane to the top of the top plate
    pitch: float                        # conveyor pitch
    pin: pins.PinDims
    relief_deg: float

    @property
    def size(self) -> tuple[float, float]:
        return self.plate[0] + 2.0 * WALL, self.plate[1] + 2.0 * WALL


def layout(v: Values) -> Layout | None:
    """Resolve the spec into a Layout, or None when a value the pallet needs is null."""
    p = "parts.bottom_plate"
    round_xy = (v.mm(f"{p}.datums.round[0]"), v.mm(f"{p}.datums.round[1]"))
    diamond_xy = (v.mm(f"{p}.datums.diamond[0]"), v.mm(f"{p}.datums.diamond[1]"))
    plate = (v.mm(f"{p}.envelope[0]"), v.mm(f"{p}.envelope[1]"))
    # The plates are located together, so the stack is as tall as the top plate's top face.
    stack_h = v.mm("parts.top_plate.z_in_stack") + v.mm("parts.top_plate.envelope[2]")
    hole_d = v.mm("cad.datum_d")
    pitch = v.mm("conveyor.pitch")
    if v.missing:
        return None
    return Layout(
        round_xy=round_xy, diamond_xy=diamond_xy, plate=plate,
        centre=((round_xy[0] + diamond_xy[0]) / 2.0, (round_xy[1] + diamond_xy[1]) / 2.0),
        stack_h=stack_h, pitch=pitch,
        pin=pins.dims(hole_d=hole_d, stack_h=stack_h, fit_clearance=PIN_FIT_CLEARANCE,
                      lead_in=PIN_LEAD_IN, shank_step=PIN_SHANK_STEP, press_depth=THICKNESS,
                      tip_chamfer=PIN_TIP_CHAMFER),
        relief_deg=pins.relief_angle_deg(round_xy, diamond_xy))


def nest_relief(lo: Layout):
    """Pocket under the whole footprint, minus a rest pad at each of its corners."""
    cx, cy = lo.centre
    sketch = Pos(cx, cy) * Rectangle(lo.plate[0] + 2.0 * NEST_CLEARANCE,
                                     lo.plate[1] + 2.0 * NEST_CLEARANCE)
    pad = REST_PAD + NEST_CLEARANCE                      # reaches the corner of the pocket
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            corner = (cx + sx * (lo.plate[0] / 2.0 + NEST_CLEARANCE - pad / 2.0),
                      cy + sy * (lo.plate[1] / 2.0 + NEST_CLEARANCE - pad / 2.0))
            sketch -= Pos(*corner) * Rectangle(pad, pad)
    return extrude(sketch, amount=RELIEF_DEPTH, dir=(0, 0, -1))


def stop_notch(lo: Layout):
    """90° V-notch in the leading (+x) face, on the pallet centreline: the conveyor hard stop."""
    cx, cy = lo.centre
    face_x = cx + lo.size[0] / 2.0
    d = STOP_NOTCH_DEPTH
    triangle = Polygon((face_x, cy - d), (face_x, cy + d), (face_x - d, cy), align=None)
    return extrude(triangle, amount=THICKNESS, dir=(0, 0, -1))


def build(lo: Layout) -> Compound:
    """Body + both pins, in the plate frame, with z = 0 at the rest plane."""
    cx, cy = lo.centre
    body = Pos(cx, cy) * Box(*lo.size, THICKNESS, align=(Align.CENTER, Align.CENTER, Align.MAX))
    body -= nest_relief(lo)
    body -= stop_notch(lo)
    for x, y in (lo.round_xy, lo.diamond_xy):                      # press-fit bores, through
        body -= Pos(x, y) * Cylinder(lo.pin.shank_d / 2.0, THICKNESS,
                                     align=(Align.CENTER, Align.CENTER, Align.MAX))
    body.label = "Pallet_Body"
    pallet = Compound(children=[body,
                                Pos(*lo.round_xy) * pins.round_pin(lo.pin),
                                Pos(*lo.diamond_xy) * pins.diamond_pin(lo.pin, lo.relief_deg)])
    pallet.label = "Pallet"
    return pallet


def masses(pallet: Compound) -> dict[str, float]:
    """kg per solid: an aluminium body, hardened steel pins."""
    return _masses(pallet, {"Pallet_Body": DENSITY_BODY}, default=DENSITY_PIN)


def derived(pallet: Compound, lo: Layout) -> dict:
    """Positions another tool can check against the spec, all in millimetres."""
    def axis_xy(label: str) -> list[float]:
        box = solid(pallet, label).bounding_box()        # both pins are symmetric about the axis
        return [round(box.center().X, 6), round(box.center().Y, 6)]

    cx, cy = lo.centre
    return {
        "frame": "parts.bottom_plate plate frame (CS_DATUM); z=0 at the plate rest plane",
        "pin_round_mm": axis_xy("Pin_Round"),
        "pin_diamond_mm": axis_xy("Pin_Diamond"),
        "datum_round_mm": [round(c, 6) for c in lo.round_xy],
        "datum_diamond_mm": [round(c, 6) for c in lo.diamond_xy],
        "datum_pitch_mm": round(((lo.diamond_xy[0] - lo.round_xy[0]) ** 2
                                 + (lo.diamond_xy[1] - lo.round_xy[1]) ** 2) ** 0.5, 6),
        "relief_angle_deg": round(lo.relief_deg, 6),
        "pin": {"locating_d_mm": round(lo.pin.locating_d, 6),
                "free_length_mm": round(lo.pin.free_length, 6),
                "shank_d_mm": round(lo.pin.shank_d, 6),
                "crown_w_mm": round(lo.pin.crown_w, 6),
                "stack_height_mm": round(lo.stack_h, 6),
                "lead_in_mm": PIN_LEAD_IN,
                "slenderness": round(lo.pin.slenderness, 3)},
        "plate_centre_mm": [round(cx, 6), round(cy, 6)],
        "plate_footprint_mm": [round(lo.plate[0], 6), round(lo.plate[1], 6)],
        "body_size_mm": [round(lo.size[0], 6), round(lo.size[1], 6), THICKNESS],
        "nest": {"relief_depth_mm": RELIEF_DEPTH, "clearance_mm": NEST_CLEARANCE,
                 "rest_pad_mm": REST_PAD,
                 "rest_pad_centres_mm": [
                     [round(cx + sx * (lo.plate[0] - REST_PAD) / 2.0, 6),
                      round(cy + sy * (lo.plate[1] - REST_PAD) / 2.0, 6)]
                     for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)]},
        "stop_notch": {"face": "+x", "included_angle_deg": 90.0, "depth_mm": STOP_NOTCH_DEPTH,
                       "vertex_mm": [round(cx + lo.size[0] / 2.0 - STOP_NOTCH_DEPTH, 6),
                                     round(cy, 6)]},
        "conveyor_pitch_mm": round(lo.pitch, 6),
        "pallet_gap_mm": round(lo.pitch - lo.size[0], 6),
    }


# ---------------------------------------------------------------- outputs

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate the main-line pallet from the spec")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help=f"default {DEFAULT_OUT}")
    ap.add_argument("--max-faces", type=int, default=50000)
    ap.add_argument("--bbox-tol-mm", type=float, default=0.2,
                    help="allowed difference between the STL bbox and the solid bbox")
    ap.add_argument("--stl-tolerance-mm", type=float, default=0.05,
                    help="STL tessellation tolerance")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    out_dir = args.out / "dev" if args.dev else args.out
    doc = new_report(PART, "tools/cad/pallet.py", out_dir, args.dev)
    report = report_path(f"cad_{PART}", args.dev)

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        write_report(report, doc)
        return code

    try:
        merged = load_merged(args.spec, args.dev)
        parse_spec(merged.raw)                      # schema first: a typo is not a missing value
    except SpecError as exc:
        doc["errors"] = exc.errors
        print(f"pallet: FAIL — invalid spec: {exc}", file=sys.stderr)
        return finish(1)

    v = Values(merged.raw, {p["path"] for p in merged.placeholders})
    try:
        lo = layout(v)
    except ValueError as exc:                   # the two datums do not define a datum line
        doc["errors"].append(f"parts.bottom_plate.datums: {exc}")
        doc["spec_values"] = v.used
        print(f"pallet: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return finish(1)
    doc["spec_values"] = v.used
    doc["placeholders"] = v.placeholders
    if lo is None:
        doc["missing"] = v.missing
        print(f"pallet: FAIL — {args.spec.name} has no value for: {', '.join(v.missing)}",
              file=sys.stderr)
        print("pallet: fill the key(s) in, or re-run with --dev to use the placeholder overlay",
              file=sys.stderr)
        return finish(1)

    if lo.size[0] > lo.pitch - PALLET_GAP:
        doc["errors"].append(
            f"pallet is {lo.size[0]:.1f} mm long but conveyor.pitch is {lo.pitch:.1f} mm: "
            f"two pallets would be {lo.pitch - lo.size[0]:.1f} mm apart, less than the "
            f"{PALLET_GAP:g} mm gap. Raise conveyor.pitch or shrink the nest wall")
        print(f"pallet: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return finish(1)

    pallet = build(lo)
    doc["bbox_mm"] = bbox_doc(pallet)
    doc["mass_kg"] = masses(pallet)
    doc["derived"] = derived(pallet, lo)

    if lo.pin.slenderness > PIN_SLENDER_MAX:
        doc["warnings"].append(
            f"pin free length {lo.pin.free_length:.1f} mm is {lo.pin.slenderness:.1f}x its "
            f"Ø{lo.pin.locating_d:.2f} mm: too whippy to locate a plate. Retractable pins, or a "
            "shorter pin that only clears the plate being placed")
    origin_off = max(abs(c) for c in lo.round_xy)
    if origin_off > 1e-6:
        doc["warnings"].append(
            f"line.origin is pallet_datum_round but parts.bottom_plate.datums.round is "
            f"{lo.round_xy} mm from the plate frame origin: the pallet model origin is offset")

    out_dir.mkdir(parents=True, exist_ok=True)
    step, stl = out_dir / f"{PART}.step", out_dir / f"{PART}.stl"
    try:
        export_step(pallet, str(step))
        export_stl(pallet, str(stl), tolerance=args.stl_tolerance_mm)
    except OSError as exc:
        doc["errors"].append(f"cannot write {out_dir}: {exc}")
        print(f"pallet: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return finish(1)
    doc["outputs"] = {"step": str(step), "stl": str(stl)}
    doc["mesh"] = to_mesh(stl, args.max_faces)

    drift = max(abs(doc["mesh"]["bbox_m"][i] * 1000.0 - doc["bbox_mm"]["size"][i])
                for i in range(3))
    doc["mesh"]["solid_bbox_drift_mm"] = round(drift, 4)
    if drift > args.bbox_tol_mm:
        doc["errors"].append(f"STL bbox differs from the solid by {drift:.3f} mm "
                             f"(tol {args.bbox_tol_mm:g} mm): tessellation is too coarse")
        print(f"pallet: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return finish(1)

    for w in doc["warnings"]:
        print(f"pallet: WARN {w}", file=sys.stderr)
    if args.dev:
        print(f"pallet: DEV MODE — {len(doc['placeholders'])} placeholder value(s); "
              f"{out_dir} is not a deliverable")
    print(f"pallet: OK — {doc['bbox_mm']['size'][0]:.1f} x {doc['bbox_mm']['size'][1]:.1f} x "
          f"{doc['bbox_mm']['size'][2]:.1f} mm, {doc['mass_kg']['total']:.3f} kg "
          f"({doc['mesh']['faces_after']} faces) -> {out_dir}")
    if lookup(merged.raw, "parts.pallet.mass") is None:
        print(f"pallet: parts.pallet.mass is null in the spec; measured "
              f"{doc['mass_kg']['total']:.3f} kg", file=sys.stderr)
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
