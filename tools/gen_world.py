"""The main-line Gazebo world, from line_spec.yaml (plan P5.1).

    tools/.venv/bin/python tools/gen_world.py [--dev] [--minimal] [--spec PATH]

Renders `templates/main_line.sdf.j2` to `drone_line_sim/worlds/main_line.sdf`. Every pose in it
comes from the spec — the generator places nothing of its own:

  * **static geometry as primitives**: the frame envelope under the whole line, the conveyor
    along the stations, a shelf at every buffer, a pad at every
    station, and one enclosure per `st_test` cell, laid out at `cell_pitch`;
  * **one `<include>` per feeder that has a model** (the magazines gen_models.py built) at
    `feeders.<id>.pose`, and the pallet at station 1. A feeder with no pose — a blow feeder lives
    on its gang head (§16) — has nothing to place, and is listed in the report as skipped;
  * **a support leg under every Buffer/Feeder primitive marker**: a thin post from the floor
    (world z = 0) up to the marker's own underside, so it does not render floating. Only a marker
    gets one — an `<include>` (a feeder with a real model) stands on its own model's geometry;
  * **world plugins**: Physics, SceneBroadcaster, UserCommands (the `set_pose` the sequencer and
    the checks use) and Contact (without it a contact sensor cannot exist).

Sizes of the static primitives are envelopes, not machines: they say where equipment is and how
much room it takes, which is what a layout check needs. They are the only numbers typed in this
file and are collected at the top.

The world names a model directory per include, and gz resolves those at parse time: a missing
model is a world that will not load at all. So every include is checked against the models tree
before the world is written, and a missing one is a FAIL naming the model and the generator that
makes it.

Writes reports/gen_world.json: every model in the world, its pose, and the spec key that pose came
from — which is what tools/checks/world_smoke.sh compares the running world against.

Output is a generated file (CLAUDE.md P0): edit the spec or the template, never main_line.sdf.
`--dev` merges the placeholder overlay, reads the dev models tree and writes
`worlds/main_line.dev.sdf` with the report at reports/gen_world.dev.json.

`--minimal` is a visualization-only rendering, independent of `--dev`: it never touches
line_spec.yaml/line_spec.dev.yaml and never writes to a deliverable report path. It always writes
`worlds/main_line.minimal.dev.sdf` and `reports/gen_world.minimal.dev.json` — fixed names, so the
output can never collide with or be mistaken for a real `main_line[.dev].sdf`. It changes only how
the world is drawn: `st_test`'s parallel cells collapse into one `Station_..._Group` envelope
spanning the cell group's full footprint (instead of one enclosure per cell), and every
envelope-only primitive (Frame, Station_* pads, the collapsed cell group, and the
Buffer/Feeder markers and their legs) renders at alpha 0.25. `<include>`s — the real generated
models — are left fully opaque either way.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

from jinja2 import Environment, FileSystemLoader, StrictUndefined

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cad.common import Values, lookup, pascal  # noqa: E402
from spec import (DEFAULT_SPEC, REPO, SpecError, add_dev_argument, load_merged,  # noqa: E402
                  parse_spec, report_path)

PKG = REPO / "drone_line_sim"
TEMPLATES = REPO / "templates"
SDF_VERSION = "1.10"
WORLD_NAME = "main_line"
HEADER = "GENERATED from line_spec.yaml by gen_world.py — do not edit"
ND = 6                              # decimals in the SDF: a micron, well under the 1 mm check

PHYSICS = {"name": "default", "type": "ode", "step_s": 0.001, "real_time_factor": 1.0}

# ---- envelopes, metres. Local to the world: they say where equipment is, not what it is. ----
FRAME_MARGIN = 0.6        # frame envelope beyond the outermost station, all round
STATION_PAD = (0.40, 0.40, 0.02)     # work-location pad at a station pose
CONVEYOR_W = 0.30         # conveyor belt width
CONVEYOR_T = 0.06         # conveyor bed thickness, below the station pose
BUFFER_SHELF = (0.35, 0.35, 0.02)
CELL_ENCLOSURE_H = 0.80   # test-cell enclosure height above the station pose
CELL_FILL = 0.85          # enclosure footprint as a fraction of cell_pitch: they stand side by side
CELL_AXIS = 1             # cells step along +y: parallel copies stand beside the line, not in it
FEEDER_MARKER = (0.25, 0.25, 0.20)   # a feeder with no model of its own is still somewhere
LEG_SECTION = (0.03, 0.03)           # support post under a floating Buffer/Feeder marker
MINIMAL_ALPHA = 0.25                 # --minimal: envelope-only primitives render at this alpha

COLOUR = {"frame": "0.45 0.45 0.48 1",
          "conveyor": "0.20 0.22 0.25 1", "station": "0.65 0.65 0.35 1",
          "cell": "0.35 0.55 0.40 1", "buffer": "0.60 0.45 0.30 1",
          "feeder": "0.50 0.35 0.55 1"}
# Kinds --minimal renders faded: every envelope-only primitive except the conveyor, which is
# drawn heavier because the pallet has to be seen sitting on it.
MINIMAL_FADED_KINDS = {"frame", "station", "cell", "buffer", "feeder"}


@dataclass
class Box:
    """A static primitive in the world: `pose` is the model's, `centre`/`size` the box's."""

    name: str
    kind: str
    pose: tuple[float, float, float]
    size: tuple[float, float, float]
    centre: tuple[float, float, float] = (0.0, 0.0, 0.0)
    spec_path: str | None = None
    note: str = ""


@dataclass
class Include:
    """A model the world brings in at a spec pose."""

    name: str
    ident: str                      # model directory / `model://<ident>`
    pose: tuple[float, float, float]
    spec_path: str
    yaw: float = 0.0


@dataclass
class Layout:
    """Everything the world needs, in metres, derived from the spec."""

    boxes: list[Box] = field(default_factory=list)
    includes: list[Include] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)


# ---------------------------------------------------------------- resolving the spec

def fmt(value: float) -> str:
    return f"{round(value, ND):.{ND}g}"


def xyz_str(xyz) -> str:
    return " ".join(fmt(c) for c in xyz)


def pose_str(xyz, yaw: float = 0.0) -> str:
    return f"{xyz_str(xyz)} 0 0 {fmt(yaw)}"


def with_alpha(colour: str, alpha: float) -> str:
    """A `COLOUR` string ("r g b a") with its alpha channel replaced."""
    r, g, b, _ = colour.split()
    return f"{r} {g} {b} {fmt(alpha)}"


def station_name(sid: str) -> str:
    """`st_place_spider` -> `Station_PlaceSpider` (CLAUDE.md naming conventions)."""
    return f"Station_{pascal(sid.removeprefix('st_'))}"


def pose_of(v: Values, path: str) -> tuple[float, float, float]:
    return tuple(v.m(f"{path}[{i}]") for i in range(3))


def yaw_of(v: Values, path: str) -> float:
    yaw = lookup(v.raw, f"{path}[3]")
    return float(yaw) if yaw is not None else 0.0


def stations(v: Values, lo: Layout, drop: float,
            minimal: bool = False) -> tuple[list[tuple], list[tuple]]:
    """A pad per station, an enclosure per cell of a station with parallel copies.

    A station pose is where the *workpiece* stands, so the pad is below it by the same amount as
    the conveyor bed — a pad sitting on the pose would be inside the pallet standing there.

    `--minimal` collapses a station's cells into one `_Group` envelope spanning their combined
    footprint, instead of one enclosure per cell — a visualization simplification, not a layout
    fact, so it only ever changes what gets drawn, never the poses or extents below it.

    Returns the station poses in index order, and every point the line has to reach over: the
    poses plus each parallel cell's own centre, which steps away from its station by `cell_pitch`
    and so lies outside the poses' own extent.
    """
    poses, extents = [], []
    for sid in sorted(v.raw.get("stations") or {},
                      key=lambda s: lookup(v.raw, f"stations.{s}.index") or 0):
        path = f"stations.{sid}.pose"
        pose = pose_of(v, path)
        count = v.plain(f"stations.{sid}.count") or 1
        poses.append(pose)
        extents.append(pose)
        if count == 1:
            lo.boxes.append(Box(station_name(sid), "station", pose, STATION_PAD,
                                (0.0, 0.0, -drop - STATION_PAD[2] / 2.0), path))
            continue
        pitch = v.m(f"stations.{sid}.cell_pitch")
        if v.missing:
            continue
        side = CELL_FILL * pitch
        count = int(count)
        last = [0.0, 0.0, 0.0]
        last[CELL_AXIS] = (count - 1) * pitch
        extents.append(tuple(pose[i] + last[i] for i in range(3)))   # the outermost cell
        if minimal:
            span = (count - 1) * pitch + side
            size = [side, side, CELL_ENCLOSURE_H]
            size[CELL_AXIS] = span
            mid = [0.0, 0.0, 0.0]
            mid[CELL_AXIS] = (count - 1) * pitch / 2.0
            centre_pose = tuple(pose[i] + mid[i] for i in range(3))
            lo.boxes.append(Box(
                f"{station_name(sid)}_Group", "cell", centre_pose,
                tuple(size), (0.0, 0.0, CELL_ENCLOSURE_H / 2.0), path,
                note=f"{count} cells collapsed into one envelope spanning {path} .. "
                     f"+{count - 1} x stations.{sid}.cell_pitch on {'xyz'[CELL_AXIS]} "
                     "(--minimal)"))
            continue
        for k in range(count):
            offset = [0.0, 0.0, 0.0]
            offset[CELL_AXIS] = k * pitch
            cell = tuple(pose[i] + offset[i] for i in range(3))
            extents.append(cell)
            lo.boxes.append(Box(
                f"{station_name(sid)}_Cell{k + 1}", "cell", cell,
                (side, side, CELL_ENCLOSURE_H), (0.0, 0.0, CELL_ENCLOSURE_H / 2.0), path,
                note=f"cell {k + 1} of {count}: {path} + {k} x stations.{sid}.cell_pitch on "
                     f"{'xyz'[CELL_AXIS]}"))
    return poses, extents


def pallet_drop(dev: bool) -> float:
    """How far the pallet's body hangs below its own origin, in metres.

    The pallet's frame is its rest plane (tools/cad/pallet.py), so a pallet standing at a station
    pose has its body *below* that pose. The conveyor has to end there, or the bed and the pallet
    are inside each other and physics throws it out. Measured, not assumed: the number is the
    bounding box the pallet generator reported.
    """
    try:
        box = json.loads(report_path("cad_pallet", dev).read_text())["bbox_mm"]
        return max(0.0, -float(box["min"][2]) / 1000.0)
    except (OSError, ValueError, KeyError, IndexError, TypeError):
        return 0.0


def frame_and_conveyor(v: Values, lo: Layout, poses: list[tuple], extents: list[tuple],
                       drop: float, work_height: float) -> None:
    """The base the line stands on and the conveyor that runs its length, spanning the stations."""
    if not poses:
        return
    # The frame has to be under everything the line reaches over, so it spans the cells too, not
    # just the station poses they hang off.
    xs = [p[0] for p in extents]
    ys = [p[1] for p in extents]
    centre = ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0)
    # The frame is the structure that holds the line at working height: its top face is the
    # conveyor bed's underside — so it is measured off station 1's pose, exactly as the bed is
    # below — and its foot is the floor.
    top = poses[0][2] - drop - CONVEYOR_T
    height = work_height + top          # ... down to world z = 0 once the line is raised
    if height <= 0.0:
        lo.errors.append(
            f"Frame: line.work_height ({work_height:.3f} m) leaves no room under the conveyor "
            f"bed, whose underside sits {-top:.3f} m below the datum plane")
        return
    lo.boxes.append(Box(
        "Frame", "frame", (centre[0], centre[1], top),
        (max(xs) - min(xs) + 2.0 * FRAME_MARGIN, max(ys) - min(ys) + 2.0 * FRAME_MARGIN, height),
        (0.0, 0.0, -height / 2.0), None,
        note="the station poses and their cells, plus a margin all round; stands on the floor and "
             f"carries the conveyor bed {height * 1000:.1f} mm up"))
    # The conveyor carries the pallet through every station, so it is as long as *they* reach —
    # a cell stands beside the line, not on the belt — and its bed ends where the pallet's
    # underside is: a station pose is where a pallet stands.
    sxs = [p[0] for p in poses]
    first = poses[0]
    lo.boxes.append(Box(
        "Conveyor", "conveyor", first,
        (max(sxs) - min(sxs) + 2.0 * FRAME_MARGIN, CONVEYOR_W, CONVEYOR_T),
        ((min(sxs) + max(sxs)) / 2.0 - first[0], 0.0, -drop - CONVEYOR_T / 2.0), None,
        note=f"station 1's pose, running the length of the line; bed top {drop * 1000:.1f} mm "
             "below the pose (the pallet's own body depth)"))


def buffers(v: Values, lo: Layout) -> None:
    for bid in v.raw.get("buffers") or {}:
        path = f"buffers.{bid}.pose"
        lo.boxes.append(Box(f"Buffer_{pascal(bid.removeprefix('buf_'))}", "buffer",
                            pose_of(v, path), BUFFER_SHELF,
                            (0.0, 0.0, -BUFFER_SHELF[2] / 2.0), path))


def feeders(v: Values, lo: Layout, models_dir: Path) -> None:
    """An include per feeder that has a model; an envelope for one that never will.

    A magazine is a model — tools/cad/magazine.py builds it and gen_models.py wraps it — so a
    magazine without one is a failure, not something to quietly draw a box for. A step or blow
    feeder has no model of its own, and an envelope at its pose is all the world can say.
    """
    for fid, f in (v.raw.get("feeders") or {}).items():
        name = f"Feeder_{pascal(fid.removeprefix('fd_'))}"
        path = f"feeders.{fid}.pose"
        if lookup(v.raw, path) is None:
            lo.skipped.append(f"{fid}: no {path} — nothing to place (a blow feeder rides its "
                              "gang head, summary §16)")
            continue
        pose = pose_of(v, path)
        if (models_dir / fid).is_dir():
            lo.includes.append(Include(name, fid, pose, path, yaw_of(v, path)))
        elif f.get("type") == "magazine":
            lo.errors.append(f"{name}: feeders.{fid} is a magazine but there is no model at "
                             f"{models_dir / fid} (run tools/gen_models.py first)")
        else:
            lo.boxes.append(Box(name, "feeder", pose, FEEDER_MARKER,
                                (0.0, 0.0, -FEEDER_MARKER[2] / 2.0), path,
                                note=f"'{f.get('type')}' feeder: an envelope, not a model"))


def pallet(v: Values, lo: Layout, models_dir: Path, poses: list[tuple]) -> None:
    """One pallet, standing at the first station — the line's origin part (§7.2)."""
    if not poses or not (models_dir / "pallet").is_dir():
        return
    first = min((v.raw.get("stations") or {}),
                key=lambda s: lookup(v.raw, f"stations.{s}.index") or 0)
    lo.includes.append(Include("Pallet_1", "pallet", poses[0], f"stations.{first}.pose",
                               yaw_of(v, f"stations.{first}.pose")))


def support_legs(lo: Layout) -> None:
    """A post under every Buffer/Feeder primitive marker so it does not render floating.

    Only a marker gets one — a feeder with a real model stands on that model's own geometry, not
    on a leg the world invents. Poses here are already world (floor-relative) ones: the post runs
    from world z = 0 up to the marker's own underside, same x/y, same colour family so it reads as
    part of it.
    """
    new = []
    for b in lo.boxes:
        if b.kind not in ("buffer", "feeder"):
            continue
        bottom = b.pose[2] + b.centre[2] - b.size[2] / 2.0
        if bottom <= 0.0:
            continue                # already at or below the floor: nothing to hold up
        new.append(Box(f"{b.name}_Leg", b.kind, (b.pose[0], b.pose[1], 0.0),
                       (LEG_SECTION[0], LEG_SECTION[1], bottom),
                       (0.0, 0.0, bottom / 2.0), b.spec_path,
                       note=f"support post, world z=0 to {b.name}'s underside"))
    lo.boxes.extend(new)


def layout(v: Values, models_dir: Path, dev: bool, minimal: bool = False) -> Layout:
    """Everything the world holds, built on the datum plane and then raised onto the frame.

    Every pose above is datum-relative — z = 0 is the plane the stations work on — because that is
    how the spec states them and how the parts stack. The line itself stands `line.work_height`
    above the floor, and that is one offset applied once, here, rather than a term every function
    has to remember to carry.
    """
    lo = Layout()
    drop = pallet_drop(dev)
    work_height = v.m("line.work_height")
    poses, extents = stations(v, lo, drop, minimal)
    frame_and_conveyor(v, lo, poses, extents, drop, work_height)
    buffers(v, lo)
    feeders(v, lo, models_dir)
    pallet(v, lo, models_dir, poses)
    if v.missing:
        return lo               # main() fails on the missing key; raising by a NaN would hide it
    for b in lo.boxes:
        b.pose = (b.pose[0], b.pose[1], b.pose[2] + work_height)
    for i in lo.includes:
        i.pose = (i.pose[0], i.pose[1], i.pose[2] + work_height)
    support_legs(lo)             # world-relative: after the raise, so "z=0" really means the floor
    return lo


# ---------------------------------------------------------------- writing

def render(env: Environment, lo: Layout, minimal: bool = False) -> str:
    def colour_of(kind: str) -> str:
        base = COLOUR[kind]
        if minimal and kind in MINIMAL_FADED_KINDS:
            return with_alpha(base, MINIMAL_ALPHA)
        return base

    return env.get_template("main_line.sdf.j2").render(
        header=HEADER, sdf_version=SDF_VERSION, world_name=WORLD_NAME, physics=PHYSICS,
        primitives=[{"name": b.name, "pose": pose_str(b.pose), "size": xyz_str(b.size),
                     "centre": xyz_str(b.centre), "colour": colour_of(b.kind),
                     "source": b.spec_path or b.note or b.kind} for b in lo.boxes],
        includes=[{"name": i.name, "uri": f"model://{i.ident}", "pose": pose_str(i.pose, i.yaw),
                   "source": i.spec_path} for i in lo.includes])


def write_report(path: Path, doc: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate the main-line world from the spec")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--out", type=Path, default=None,
                    help=f"world file (default {PKG / 'worlds' / 'main_line.sdf'})")
    ap.add_argument("--models", type=Path, default=PKG / "models",
                    help="package models/ directory (default %(default)s)")
    ap.add_argument("--minimal", action="store_true",
                    help="visualization only: collapse st_test's cells into one envelope and "
                         "fade every envelope-only primitive; independent of --dev; always writes "
                         "worlds/main_line.minimal.dev.sdf and reports/gen_world.minimal.dev.json "
                         "so it can never be mistaken for a deliverable")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    models_dir = args.models / "dev" if args.dev else args.models
    if args.minimal:
        out = args.out or (PKG / "worlds" / "main_line.minimal.dev.sdf")
        report = report_path("gen_world.minimal", True)
    else:
        out = args.out or (PKG / "worlds" / f"main_line{'.dev' if args.dev else ''}.sdf")
        report = report_path("gen_world", args.dev)
    doc: dict = {"passed": False, "dev": args.dev, "minimal": args.minimal, "out": str(out),
                 "models_dir": str(models_dir), "world": WORLD_NAME, "models": {},
                 "skipped": [], "spec_values": {}, "placeholders": [], "missing": [],
                 "warnings": [], "errors": []}

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        write_report(report, doc)
        return code

    try:
        merged = load_merged(args.spec, args.dev)
        parse_spec(merged.raw)                      # schema first: a typo is not a missing value
    except SpecError as exc:
        doc["errors"] = exc.errors
        print(f"gen_world: FAIL — invalid spec: {exc}", file=sys.stderr)
        return finish(1)

    v = Values(merged.raw, {p["path"] for p in merged.placeholders})
    lo = layout(v, models_dir, args.dev, args.minimal)
    doc["spec_values"] = v.used
    doc["placeholders"] = v.placeholders
    doc["skipped"] = lo.skipped
    if v.missing:
        doc["missing"] = v.missing
        for m in v.missing:
            print(f"gen_world: FAIL — no value for {m}", file=sys.stderr)
        print("gen_world: fill the key(s) in, or re-run with --dev to use the placeholder overlay",
              file=sys.stderr)
        return finish(1)

    doc["errors"] += lo.errors
    names = [b.name for b in lo.boxes] + [i.name for i in lo.includes]
    for name in sorted(set(n for n in names if names.count(n) > 1)):
        doc["errors"].append(f"two models are both called '{name}': every model in a world needs "
                             "its own name")
    # gz resolves an include at parse time, so a missing model directory is not a missing visual:
    # it is a world that will not load.
    for inc in lo.includes:
        if not (models_dir / inc.ident).is_dir():
            doc["errors"].append(f"{inc.name}: no model at {models_dir / inc.ident} "
                                 "(run tools/gen_models.py first)")
    if doc["errors"]:
        for e in doc["errors"]:
            print(f"gen_world: FAIL — {e}", file=sys.stderr)
        return finish(1)

    env = Environment(loader=FileSystemLoader(TEMPLATES), undefined=StrictUndefined,
                      keep_trailing_newline=True, trim_blocks=False)
    try:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(render(env, lo, args.minimal))
    except OSError as exc:
        doc["errors"].append(f"cannot write {out}: {exc}")
        print(f"gen_world: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return finish(1)

    for b in lo.boxes:
        doc["models"][b.name] = {"kind": b.kind, "static": True,
                                 "pose": [round(c, ND) for c in b.pose],
                                 "size_m": [round(c, ND) for c in b.size],
                                 "spec_path": b.spec_path, "note": b.note}
    for i in lo.includes:
        doc["models"][i.name] = {"kind": "include", "static": False,
                                 "pose": [round(c, ND) for c in i.pose],
                                 "yaw": round(i.yaw, ND),
                                 "uri": f"model://{i.ident}",
                                 "model_dir": os.path.relpath(models_dir / i.ident, REPO),
                                 "spec_path": i.spec_path, "note": ""}

    for s in lo.skipped:
        print(f"gen_world: skipped {s}", file=sys.stderr)
    if args.dev:
        print(f"gen_world: DEV MODE — {len(doc['placeholders'])} placeholder value(s); "
              f"{out} is not a deliverable")
    if args.minimal:
        print(f"gen_world: MINIMAL MODE — visualization only; {out} is not a deliverable")
    print(f"gen_world: OK — {len(lo.boxes)} primitive(s), {len(lo.includes)} include(s) -> {out}")
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
