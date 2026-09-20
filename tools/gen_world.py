"""The main-line Gazebo world, from line_spec.yaml (plan P5.1).

    tools/.venv/bin/python tools/gen_world.py [--dev] [--spec PATH]

Renders `templates/main_line.sdf.j2` to `drone_line_sim/worlds/main_line.sdf`. Every pose in it
comes from the spec — the generator places nothing of its own:

  * **static geometry as primitives**: the frame envelope under the whole line, the gantry rail
    envelope over it, the conveyor along the stations, a shelf at every buffer, a pad at every
    station, and one enclosure per `st_test` cell, laid out at `cell_pitch`;
  * **one `<include>` per feeder that has a model** (the magazines gen_models.py built) at
    `feeders.<id>.pose`, and the pallet at station 1. A feeder with no pose — a blow feeder lives
    on its gang head (§16) — has nothing to place, and is listed in the report as skipped;
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
FRAME_T = 0.08            # frame/base plate thickness, under the line
RAIL_T = 0.10             # gantry rail envelope thickness, above the line
STATION_PAD = (0.40, 0.40, 0.02)     # work-location pad at a station pose
CONVEYOR_W = 0.30         # conveyor belt width
CONVEYOR_T = 0.06         # conveyor bed thickness, below the station pose
BUFFER_SHELF = (0.35, 0.35, 0.02)
CELL_ENCLOSURE_H = 0.80   # test-cell enclosure height above the station pose
CELL_FILL = 0.85          # enclosure footprint as a fraction of cell_pitch: they stand side by side
CELL_AXIS = 1             # cells step along +y: parallel copies stand beside the line, not in it
FEEDER_MARKER = (0.25, 0.25, 0.20)   # a feeder with no model of its own is still somewhere

COLOUR = {"frame": "0.45 0.45 0.48 1", "rail": "0.25 0.45 0.65 1",
          "conveyor": "0.20 0.22 0.25 1", "station": "0.65 0.65 0.35 1",
          "cell": "0.35 0.55 0.40 1", "buffer": "0.60 0.45 0.30 1",
          "feeder": "0.50 0.35 0.55 1"}


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


def station_name(sid: str) -> str:
    """`st_place_spider` -> `Station_PlaceSpider` (CLAUDE.md naming conventions)."""
    return f"Station_{pascal(sid.removeprefix('st_'))}"


def pose_of(v: Values, path: str) -> tuple[float, float, float]:
    return tuple(v.m(f"{path}[{i}]") for i in range(3))


def yaw_of(v: Values, path: str) -> float:
    yaw = lookup(v.raw, f"{path}[3]")
    return float(yaw) if yaw is not None else 0.0


def stations(v: Values, lo: Layout, drop: float) -> list[tuple[float, float, float]]:
    """A pad per station, an enclosure per cell of a station with parallel copies. Returns poses.

    A station pose is where the *workpiece* stands, so the pad is below it by the same amount as
    the conveyor bed — a pad sitting on the pose would be inside the pallet standing there.
    """
    poses = []
    for sid in sorted(v.raw.get("stations") or {},
                      key=lambda s: lookup(v.raw, f"stations.{s}.index") or 0):
        path = f"stations.{sid}.pose"
        pose = pose_of(v, path)
        count = v.plain(f"stations.{sid}.count") or 1
        poses.append(pose)
        if count == 1:
            lo.boxes.append(Box(station_name(sid), "station", pose, STATION_PAD,
                                (0.0, 0.0, -drop - STATION_PAD[2] / 2.0), path))
            continue
        pitch = v.m(f"stations.{sid}.cell_pitch")
        if v.missing:
            continue
        side = CELL_FILL * pitch
        for k in range(int(count)):
            offset = [0.0, 0.0, 0.0]
            offset[CELL_AXIS] = k * pitch
            cell = tuple(pose[i] + offset[i] for i in range(3))
            lo.boxes.append(Box(
                f"{station_name(sid)}_Cell{k + 1}", "cell", cell,
                (side, side, CELL_ENCLOSURE_H), (0.0, 0.0, CELL_ENCLOSURE_H / 2.0), path,
                note=f"cell {k + 1} of {count}: {path} + {k} x stations.{sid}.cell_pitch on "
                     f"{'xyz'[CELL_AXIS]}"))
    return poses


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


def frame_and_conveyor(v: Values, lo: Layout, poses: list[tuple], drop: float) -> None:
    """The base the line stands on and the conveyor that runs its length, spanning the stations."""
    if not poses:
        return
    xs = [p[0] for p in poses]
    ys = [p[1] for p in poses]
    span_x, span_y = max(xs) - min(xs), max(ys) - min(ys)
    centre = ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0, 0.0)
    lo.boxes.append(Box(
        "Frame", "frame", centre,
        (span_x + 2.0 * FRAME_MARGIN, span_y + 2.0 * FRAME_MARGIN, FRAME_T),
        (0.0, 0.0, -FRAME_T / 2.0), None, note="the station poses, plus a margin all round"))
    # The conveyor carries the pallet through every station, so it is as long as they reach and
    # its bed ends where the pallet's underside is: a station pose is where a pallet stands.
    first = poses[0]
    lo.boxes.append(Box(
        "Conveyor", "conveyor", first,
        (span_x + 2.0 * FRAME_MARGIN, CONVEYOR_W, CONVEYOR_T),
        (centre[0] - first[0], 0.0, -drop - CONVEYOR_T / 2.0), None,
        note=f"station 1's pose, running the length of the line; bed top {drop * 1000:.1f} mm "
             "below the pose (the pallet's own body depth)"))


def gantry_rail(v: Values, lo: Layout) -> None:
    """The rail envelope: the volume the gantry's x/y travel sweeps, above the line."""
    mount = tuple(v.m(f"gantry.mount[{i}]") for i in range(3))
    span = {}
    for axis in "xy":
        span[axis] = (v.m(f"gantry.axes.{axis}.min"), v.m(f"gantry.axes.{axis}.max"))
    top = v.m("gantry.axes.z.max")
    if v.missing:
        return
    size = (span["x"][1] - span["x"][0], span["y"][1] - span["y"][0], RAIL_T)
    centre = ((span["x"][0] + span["x"][1]) / 2.0, (span["y"][0] + span["y"][1]) / 2.0,
              top + RAIL_T / 2.0)
    lo.boxes.append(Box("Rail_Gantry", "rail", mount, size, centre, "gantry.mount",
                        note="gantry.axes x/y travel, RAIL_T thick, above gantry.axes.z.max"))


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


def layout(v: Values, models_dir: Path, dev: bool) -> Layout:
    lo = Layout()
    drop = pallet_drop(dev)
    poses = stations(v, lo, drop)
    frame_and_conveyor(v, lo, poses, drop)
    gantry_rail(v, lo)
    buffers(v, lo)
    feeders(v, lo, models_dir)
    pallet(v, lo, models_dir, poses)
    return lo


# ---------------------------------------------------------------- writing

def render(env: Environment, lo: Layout) -> str:
    return env.get_template("main_line.sdf.j2").render(
        header=HEADER, sdf_version=SDF_VERSION, world_name=WORLD_NAME, physics=PHYSICS,
        primitives=[{"name": b.name, "pose": pose_str(b.pose), "size": xyz_str(b.size),
                     "centre": xyz_str(b.centre), "colour": COLOUR[b.kind],
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
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    models_dir = args.models / "dev" if args.dev else args.models
    out = args.out or (PKG / "worlds" / f"main_line{'.dev' if args.dev else ''}.sdf")
    doc: dict = {"passed": False, "dev": args.dev, "out": str(out),
                 "models_dir": str(models_dir), "world": WORLD_NAME, "models": {},
                 "skipped": [], "spec_values": {}, "placeholders": [], "missing": [],
                 "warnings": [], "errors": []}
    report = report_path("gen_world", args.dev)

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
    lo = layout(v, models_dir, args.dev)
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
        out.write_text(render(env, lo))
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
    print(f"gen_world: OK — {len(lo.boxes)} primitive(s), {len(lo.includes)} include(s) -> {out}")
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
