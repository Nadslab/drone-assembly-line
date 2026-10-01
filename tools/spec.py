"""Typed loader for drone_line_sim/config/line_spec.yaml.

Every model forbids unknown keys, so a typo (`duraton_s`) is an error instead of a silent null.
A key that is present with value `null` means "not yet known"; check_line.py reports those as
UNRESOLVED. No ROS imports: this runs in tools/.venv.

    tools/.venv/bin/python tools/spec.py [path] [--dev]   # validate and print a summary

Dev overlay: `load_spec(dev=True)` deep-merges config/line_spec.dev.yaml over the base spec so
work that needs numbers (Gazebo, control, sequencer) can run before CAD/AnyLogic values exist.
Rules, enforced in `merge_overlay`:
  - a base value that is not null always wins; the overlay only fills nulls and absent optional
    fields (an overlay value that disagrees with a real base value is reported as "shadowed");
  - the overlay cannot add entities (a new part, feeder, station, ...) or resize lists;
  - every value that came from the overlay is returned as a placeholder so reports can list it.
No deliverable number may come from the overlay: dev runs write reports/<name>.dev.json.
"""

from __future__ import annotations

import argparse
import copy
import re
import sys
from fnmatch import fnmatchcase
from dataclasses import dataclass, field
from pathlib import Path
from typing import Annotated, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator

REPO = Path(__file__).resolve().parent.parent
DEFAULT_SPEC = REPO / "drone_line_sim" / "config" / "line_spec.yaml"

Pos = Annotated[float, Field(gt=0)]
NonNeg = Annotated[float, Field(ge=0)]
PosInt = Annotated[int, Field(ge=1)]
NonNegInt = Annotated[int, Field(ge=0)]
Vec2 = tuple[float | None, float | None]
Vec3 = tuple[float | None, float | None, float | None]
Pose = tuple[float | None, float | None, float | None, float | None]  # x, y, z, yaw


class Model(BaseModel):
    model_config = ConfigDict(extra="forbid", populate_by_name=True)


class Units(Model):
    length: Literal["m"]
    mass: Literal["kg"]
    time: Literal["s"]
    angle: Literal["rad"]


class Line(Model):
    takt_s: Pos
    units: Units
    origin: str
    work_height: Pos | None = None   # datum plane's height above the world ground plane


class Datums(Model):
    round: Vec2 | None = None
    diamond: Vec2 | None = None


class ArmHoles(Model):
    """The arm-sandwich screw pattern on a plate, in that plate's frame (summary §7.1).

    One list of every hole, plus how many of them hold one arm down: the holes group into
    `len(positions) / per_arm` arms by their angle about the plate centre. The spider carrier's
    arm pockets are computed from this pattern, so the carrier can never drift from the plate.
    """

    per_arm: PosInt
    positions: list[Vec2]

    @model_validator(mode="after")
    def _groups(self):
        if not self.positions or len(self.positions) % self.per_arm:
            raise ValueError(f"{len(self.positions)} positions do not divide into groups of "
                             f"per_arm={self.per_arm}")
        return self


class Part(Model):
    source: Literal["solidworks", "build123d", "primitive"]
    mesh: str | None = None
    generator: str | None = None
    shape: Literal["box", "cylinder"] | None = None
    dims: list[float | None] | None = None
    envelope: Vec3 | None = None
    mass: Pos | None = None
    datums: Datums | None = None
    pick: Pose | None = None
    carried_by: str | None = None
    z_in_stack: Pos | None = None  # bottom face above the bottom plate (CAD stack height)
    lead_slot: tuple[float | None, float | None] | None = None  # [width, length], mid plate
    arm_holes: ArmHoles | None = None  # plates: the arm-sandwich screw pattern
    hole_inset: Pos | None = None  # arm: inboard end -> centroid of its own mount holes
    # Poka-yoke key (summary 7.5, 16): the corner cut [x, y] at the part's +x,+y corner in its own
    # frame. The magazine grows a matching rib there, so a mirrored part cannot seat (P4 B5).
    key: tuple[float | None, float | None] | None = None

    @model_validator(mode="after")
    def _source_fields(self):
        need = {"solidworks": ["mesh"], "build123d": ["generator"],
                "primitive": ["shape", "dims"]}[self.source]
        missing = [f for f in need if getattr(self, f) is None]
        if missing:
            raise ValueError(f"source '{self.source}' requires {missing}")
        if self.shape == "box" and self.dims is not None and len(self.dims) != 3:
            raise ValueError("box dims must be [x, y, z]")
        if self.shape == "cylinder" and self.dims is not None and len(self.dims) != 2:
            raise ValueError("cylinder dims must be [radius, height]")
        return self


class Cad(Model):
    """Shared SolidWorks dimensions written to cad_params.txt by gen_sw_params.py."""

    datum_d: Pos
    press_nut_hole_d: Pos | None = None


class Fastener(Model):
    length: Pos
    feeder: str


class Feeder(Model):
    type: Literal["magazine", "step", "blow", "nest"]
    part: str
    # Parts presented per pick: a fixtured nest hands over a whole set (D-009). Absent = 1.
    per_pick: PosInt | None = None
    pose: Pose | None = None
    capacity: PosInt | None = None
    refill_s: Pos | None = None
    target: str | None = None
    feed_s: Pos | None = None


class Source(Model):
    output: str
    buffer: str
    interval_s: Pos
    buffer_capacity: NonNegInt | None


class Buffer(Model):
    pose: Pose


class Axis(Model):
    min: float | None
    max: float | None
    v_max: Pos | None
    a_max: Pos | None
    # Joint dynamics and controller gain from the P1 spike (docs/decisions.md D-002).
    damping: NonNeg | None = None
    friction: NonNeg | None = None
    position_gain: Pos | None = None

    @model_validator(mode="after")
    def _ordered(self):
        if self.min is not None and self.max is not None and self.min >= self.max:
            raise ValueError(f"axis min {self.min} must be < max {self.max}")
        return self


class GantryAxes(Model):
    x: Axis
    y: Axis
    z: Axis
    c: Axis | None = None  # absent = no yaw axis


class Tool(Model):
    """A gantry end-effector. `dims` is the envelope it needs at the pick feature, in metres:
    [x, y, reach] — x,y across the pick point and reach = how far it comes down over it. The
    fixture generators (P4 B3-B4) size their pick features from this."""

    grip: Literal["vacuum", "finger"]
    dims: Vec3


class Gantry(Model):
    axes: GantryAxes
    tools: dict[str, Tool]
    mount: Vec3
    # The stations this gantry serves (docs/decisions.md D-008). Zones never overlap.
    zone: list[str] | None


class GantryDefaults(Model):
    """Pick-and-place timing and linear-axis dynamics shared by every gantry.

    `damping`/`friction` are the P1 values (D-002) for the x/y/z axes of every gantry; an axis
    that states its own wins, and the rotary C axis only ever uses its own (other units).
    """

    grip_s: NonNeg | None
    release_s: NonNeg | None
    settle_s: NonNeg | None
    # safe_z is derived per gantry (tools/gantry_geometry.py, D-009): the tallest thing under it,
    # plus the tallest part it carries, plus this margin.
    z_margin: NonNeg | None
    # Moves are planned at this fraction of v_max/a_max (D-007: a_max is a ceiling).
    plan_speed_scale: Annotated[float, Field(gt=0, le=1)] | None
    damping: NonNeg | None
    friction: NonNeg | None


class GangHead(Model):
    direction: Literal["down", "horizontal", "up"]  # up = driven from below the workpiece
    spindles: PosInt | None
    pattern_from: str | None = None
    opposed: bool = False
    stroke: Pos | None
    # The stroke joint's own limits, like an axis: a URDF joint cannot be written without them.
    v_max: Pos | None = None
    a_max: Pos | None = None
    # One pass: advance, drive every spindle home, retract (D-011). A step with more screws than
    # spindles takes ceil(qty / spindles) passes; check_line.py `station_time` adds them up.
    drive_s: Pos | None = None


class Scara(Model):
    base: Vec3 | None = None  # in line frame
    reach: Pos | None = None  # horizontal radius from base


class Conveyor(Model):
    # No index time here: the pallet advances one pitch at layout.conveyor.belt_speed, so the
    # index time is derived (`index_s()`), never stated twice (D-010).
    pitch: Pos | None


ACTORS = ("gantry", "driver", "operator", "sub_line", "test")


class Station(Model):
    index: PosInt
    count: PosInt
    pose: Pose
    cell_pitch: Pos | None = None
    # Who works here (D-010). Derived from the steps for gantry/driver/test; stated only where a
    # step cannot say it: an operator, or a sub_line that loads the pallet from `source` (a buffer
    # or a source). check_line.py `actors` holds every station to exactly one actor type.
    actor: Literal[ACTORS] | None = None
    source: str | None = None


class Step(Model):
    id: str
    station: str
    capability: Literal["load", "place", "pick", "drive", "inspect", "test", "pack", "advance"]
    duration_s: NonNeg | None
    requires: list[str]
    part: str | None = None
    from_: str | None = Field(default=None, alias="from")
    tool: str | None = None
    head: str | None = None
    fastener: str | None = None
    qty: PosInt | None = None
    status: Literal["active", "parked"] = "active"
    note: str | None = None


class LayoutConveyor(Model):
    width: Pos | None
    top_height: Pos | None      # belt top above the floor
    belt_speed: Pos | None      # m/s


class PalletBlock(Model):
    length: Pos | None          # along the conveyor (x)
    width: Pos | None
    thickness: Pos | None


class GantryFrame(Model):
    post_size: Pos | None       # square post section
    beam_height: Pos | None     # beam section, vertical
    beam_depth: Pos | None      # beam section, horizontal
    conveyor_clearance: NonNeg | None = None   # post footprint to the conveyor edge, at least


class TestBranch(Model):
    offset_y: float | None      # first cell centre from st_test's pose, in y; cells step away
    cell_w: Pos | None          # cell size along the branch (y), <= cell_pitch
    cell_d: Pos | None          # cell size across the branch (x)


class OperatorAisle(Model):
    side: Literal["+y", "-y"] | None
    width: Pos | None


class Layout(Model):
    """Floor-plan dimensions for tools/cad/layout.py (SolidWorks review). Metres."""

    conveyor: LayoutConveyor
    pallet_block: PalletBlock
    stop_unit_length: Pos | None
    stop_unit_cycle_s: NonNeg | None = None    # stop, lift/locate, release: part of each transfer
    buffer_slots_between_stations: NonNegInt | None
    gantry_frame: GantryFrame
    # [w (x), d (y), h (floor to top)] keyed by feeder or buffer id: both stand on the floor.
    feeder_footprints: dict[str, Vec3]
    test_branch: TestBranch
    operator_aisle: OperatorAisle
    guard_offset: Pos | None    # gantry travel envelope to the guard line, all round


class LineSpec(Model):
    schema_version: Literal[1]
    line: Line
    parts: dict[str, Part]
    cad: Cad | None = None
    fasteners: dict[str, Fastener]
    feeders: dict[str, Feeder]
    sources: dict[str, Source]
    buffers: dict[str, Buffer]
    gantries: Annotated[dict[str, Gantry], Field(min_length=1)]
    gantry_defaults: GantryDefaults
    gang_heads: dict[str, GangHead]
    scaras: dict[str, Scara]
    conveyor: Conveyor
    stations: dict[str, Station]
    sequence: list[Step]
    layout: Layout | None = None   # only tools/cad/layout.py needs it

    @model_validator(mode="after")
    def _zones(self):
        errors = []
        for gid in self.gantries:
            if not GANTRY_ID.fullmatch(gid):
                errors.append(f"gantries.{gid}: a gantry id is gantry_<n> (docs/naming.md)")
        owner: dict[str, str] = {}
        for gid, g in self.gantries.items():
            for st in g.zone or []:
                if st not in self.stations:
                    errors.append(f"gantries.{gid}.zone: station '{st}' does not exist")
                elif st in owner:
                    errors.append(f"gantries.{gid}.zone: station '{st}' is already in "
                                  f"{owner[st]}'s zone; zones may not overlap")
                else:
                    owner[st] = gid
        # With a zone still null, a station missing from the known zones may be in that one.
        if all(g.zone is not None for g in self.gantries.values()):
            for s in self.sequence:
                if gantry_served(s) and s.station in self.stations and s.station not in owner:
                    errors.append(f"sequence.{s.id}: moves a part with a gantry but station "
                                  f"'{s.station}' is in no gantry's zone")
        # A gantry places a part; a driver drives fasteners. One step, one kind of qty (D-009).
        for s in self.sequence:
            if gantry_served(s) and (s.capability == "drive" or s.fastener is not None):
                errors.append(f"sequence.{s.id}: moves part '{s.part}' with a gantry and drives "
                              f"fasteners ({s.capability}, qty {s.qty}) in one step; split it "
                              "into a place step and a drive step")
        # One tool id is one physical design: the CAD generators build it once.
        first: dict[str, tuple[str, Tool]] = {}
        for gid, g in self.gantries.items():
            for tid, tool in g.tools.items():
                if tid in first and first[tid][1] != tool:
                    errors.append(f"gantries.{gid}.tools.{tid} differs from "
                                  f"gantries.{first[tid][0]}.tools.{tid}; one tool id is one "
                                  "tool")
                first.setdefault(tid, (gid, tool))
        if errors:
            raise ValueError("; ".join(errors))
        return self


GANTRY_ID = re.compile(r"gantry_[1-9]\d*")


def gantry_served(step: Step | dict) -> bool:
    """A step the gantry performs: it names a tool, or draws a part `from` a feeder/buffer."""
    if isinstance(step, dict):
        tool, part, source = step.get("tool"), step.get("part"), step.get("from")
    else:
        tool, part, source = step.tool, step.part, step.from_
    return tool is not None or (part is not None and source is not None)


def zone_owner(spec: LineSpec) -> dict[str, str]:
    """{station: gantry id} over every known zone."""
    return {st: gid for gid, g in spec.gantries.items() for st in (g.zone or [])}


def station_gantry(raw: dict, station: str | None) -> str | None:
    """The gantry whose zone holds `station`, from plain spec data (generators, xacro helpers)."""
    for gid, g in (raw.get("gantries") or {}).items():
        if isinstance(g, dict) and station in (g.get("zone") or []):
            return gid
    return None


def index_s(spec: LineSpec) -> float | None:
    """Seconds the pallet takes to advance one station: conveyor.pitch / layout.conveyor.belt_speed.
    The only index time there is (D-010); None while either is unknown."""
    lay = spec.layout
    if spec.conveyor.pitch is None or lay is None or lay.conveyor.belt_speed is None:
        return None
    return spec.conveyor.pitch / lay.conveyor.belt_speed


def gantry_ids(raw: dict) -> list[str]:
    """gantry_1, gantry_2, ... in numeric order (gantry_10 after gantry_9)."""
    ids = list(raw.get("gantries") or {})
    return sorted(ids, key=lambda g: (int(g.split("_")[-1]) if g.split("_")[-1].isdigit()
                                      else 0, g))


class SpecError(Exception):
    """The spec file is unreadable or does not match the schema."""

    def __init__(self, errors: list[str]):
        super().__init__("; ".join(errors))
        self.errors = errors


def format_validation_error(exc: ValidationError) -> list[str]:
    return [f"{'.'.join(str(p) for p in e['loc'])}: {e['msg']}" for e in exc.errors()]


def overlay_path(base: Path) -> Path:
    """line_spec.yaml -> line_spec.dev.yaml, next to it."""
    return base.with_name(f"{base.stem}.dev{base.suffix}")


def report_path(name: str, dev: bool = False) -> Path:
    """reports/<name>.json, or reports/<name>.dev.json for a dev run so it can never be
    mistaken for (or overwrite) a report built from real values."""
    return REPO / "reports" / f"{name}{'.dev' if dev else ''}.json"


def add_dev_argument(parser: argparse.ArgumentParser) -> None:
    """Every generator and check calls this so `--dev` means the same thing everywhere."""
    parser.add_argument("--dev", action="store_true",
                        help="merge line_spec.dev.yaml (PLACEHOLDER values) over the spec; "
                             "output goes to *.dev.* and is never a deliverable")


# Collections whose keys are entities: the overlay may fill their fields but not add members.
ENTITY_COLLECTIONS = {"parts", "fasteners", "feeders", "sources", "buffers", "gang_heads",
                      "scaras", "stations", "gantries", "gantries.*.tools",
                      "layout.feeder_footprints"}


def _is_entity_collection(path: str) -> bool:
    return any(fnmatchcase(path, pattern) for pattern in ENTITY_COLLECTIONS)


@dataclass
class Merged:
    raw: dict
    placeholders: list[dict] = field(default_factory=list)  # {"path", "value"} from the overlay
    shadowed: list[dict] = field(default_factory=list)  # overlay values ignored: base is real


def _leaves(node, path):
    if node is None:
        return
    if isinstance(node, dict):
        for k, v in node.items():
            yield from _leaves(v, f"{path}.{k}" if path else str(k))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from _leaves(v, f"{path}[{i}]")
    else:
        yield path, node


def merge_overlay(base: dict, overlay: dict) -> Merged:
    """Deep-merge `overlay` into `base` without ever changing a non-null base value."""
    out = Merged(raw={})

    def fill(o, path):
        out.placeholders.extend({"path": p, "value": v} for p, v in _leaves(o, path))
        return copy.deepcopy(o)

    def merge(b, o, path):
        if o is None:
            return copy.deepcopy(b)
        if b is None:
            return fill(o, path)
        if isinstance(b, dict) and isinstance(o, dict):
            if "id" in b and "id" in o and b["id"] != o["id"]:
                raise SpecError([f"overlay {path}: id '{o['id']}' does not match base "
                                 f"'{b['id']}' (list entries are matched by position)"])
            res = {}
            for k, bv in b.items():
                res[k] = merge(bv, o.get(k), f"{path}.{k}" if path else k)
            for k, ov in o.items():
                if k in b or ov is None:
                    continue
                kp = f"{path}.{k}" if path else k
                if _is_entity_collection(path):
                    raise SpecError([f"overlay adds new entity {kp}; the overlay may only "
                                     "fill values of entities the base spec defines"])
                res[k] = fill(ov, kp)
            return res
        if isinstance(b, list) and isinstance(o, list):
            if len(b) != len(o):
                raise SpecError([f"overlay {path}: {len(o)} items, base has {len(b)}"])
            return [merge(bi, oi, f"{path}[{i}]") for i, (bi, oi) in enumerate(zip(b, o))]
        if b != o:
            out.shadowed.append({"path": path, "base": b, "overlay": o})
        return copy.deepcopy(b)

    out.raw = merge(base, overlay, "")
    return out


def _read_yaml(p: Path) -> dict:
    try:
        data = yaml.safe_load(p.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise SpecError([f"cannot read {p}: {exc}"]) from exc
    if not isinstance(data, dict):
        raise SpecError([f"{p}: top level must be a mapping"])
    return data


def load_merged(path: str | Path | None = None, dev: bool = False) -> Merged:
    """The spec as plain YAML data (nulls preserved), plus overlay provenance when dev."""
    p = Path(path) if path else DEFAULT_SPEC
    base = _read_yaml(p)
    if not dev:
        return Merged(raw=base)
    return merge_overlay(base, _read_yaml(overlay_path(p)))


def load_raw(path: str | Path | None = None, dev: bool = False) -> dict:
    return load_merged(path, dev).raw


def parse_spec(raw: dict) -> LineSpec:
    try:
        return LineSpec.model_validate(raw)
    except ValidationError as exc:
        raise SpecError(format_validation_error(exc)) from exc


def load_spec(path: str | Path | None = None, dev: bool = False) -> LineSpec:
    return parse_spec(load_raw(path, dev))


DUMP_HEADER = ("# GENERATED by tools/spec.py --dump — do not edit.\n"
               "# line_spec.yaml{overlay} as one file, for a consumer that cannot merge the\n"
               "# overlay itself (xacro's load_yaml reads one path).\n")


def dump_merged(merged: Merged, out: Path, dev: bool) -> None:
    """Write the merged spec as a single YAML file.

    `xacro.load_yaml` takes one path and knows nothing about the dev overlay, so a dev expansion
    of the descriptions needs the merged spec on disk. It is a generated file: it goes to
    reports/ (never installed, never committed) and carries the header that says so.
    """
    out.parent.mkdir(parents=True, exist_ok=True)
    overlay = " + line_spec.dev.yaml (PLACEHOLDER values)" if dev else ""
    out.write_text(DUMP_HEADER.format(overlay=overlay)
                   + yaml.safe_dump(merged.raw, sort_keys=False, default_flow_style=False))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Validate line_spec.yaml")
    ap.add_argument("path", nargs="?")
    ap.add_argument("--dump", type=Path,
                    help="also write the merged spec as one YAML file (what xacro loads)")
    add_dev_argument(ap)
    args = ap.parse_args()
    try:
        m = load_merged(args.path, args.dev)
        s = parse_spec(m.raw)
    except SpecError as exc:
        print("spec: INVALID", file=sys.stderr)
        for e in exc.errors:
            print(f"  {e}", file=sys.stderr)
        sys.exit(1)
    print(f"spec: OK — {len(s.parts)} parts, {len(s.feeders)} feeders, "
          f"{len(s.stations)} stations, {len(s.gantries)} gantries, {len(s.sequence)} steps")
    if args.dump:
        dump_merged(m, args.dump, args.dev)
        print(f"spec: merged spec -> {args.dump}")
    if args.dev:
        print(f"spec: DEV — {len(m.placeholders)} placeholder value(s) from the overlay")
