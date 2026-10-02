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
    d: Pos | None = None    # nominal thread diameter (arm_pin_clearance, D-012)


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
    # A fixture that goes back to this sub-line empty once its load is taken (option B, D-012:
    # carrier_spider stays in buf_spider). A no-op in every tool; check_line.py checks the name.
    returns: str | None = None


class Buffer(Model):
    pose: Pose


class Tool(Model):
    """A SCARA end-effector. `dims` is the envelope it needs at the pick feature, in metres:
    [x, y, reach] — x,y across the pick point and reach = how far it comes down over it. The
    fixture generators (P4 B3-B4) size their pick features from this."""

    grip: Literal["vacuum", "finger"]
    dims: Vec3


class Reach(Model):
    """The annulus about the base the TCP can work in, metres."""

    r_min: Pos | None
    r_max: Pos | None

    @model_validator(mode="after")
    def _ordered(self):
        if self.r_min is not None and self.r_max is not None and self.r_min >= self.r_max:
            raise ValueError(f"reach r_min {self.r_min} must be < r_max {self.r_max}")
        return self


class Motion(Model):
    """TCP limits, Cartesian: the trapezoid estimate_moves.py plans with (D-012)."""

    v_max: Pos | None
    a_max: Pos | None


class Scara(Model):
    """One SCARA per pick-and-place station (D-012), keyed `scara_<station without st_>`.

    Its base stands at the station's x, `base_offset_y` out from the station on `base_side` (the
    side of its part source): `scara_base()` is the one place that arithmetic lives. The TCP works
    in the annulus [reach.r_min, reach.r_max] about the base and at z 0 .. `z_stroke` above the
    datum plane."""

    station: str
    base_side: Literal["+y", "-y"] | None
    base_offset_y: Pos | None
    reach: Reach
    z_stroke: Pos | None
    payload_kg: Pos | None
    tools: dict[str, Tool]
    motion: Motion


class ScaraDefaults(Model):
    """Pick-and-place timing shared by every SCARA."""

    grip_s: NonNeg | None
    release_s: NonNeg | None
    settle_s: NonNeg | None
    # safe_z is derived per SCARA (tools/robot_geometry.py): the tallest item in its reach
    # envelope, plus the part it carries, plus this margin.
    z_margin: NonNeg | None
    # Moves are planned at this fraction of v_max/a_max (D-007's gantry value; re-derive, D-012).
    plan_speed_scale: Annotated[float, Field(gt=0, le=1)] | None


class Slide(Model):
    """A gang head that shares its station with a SCARA rides in on a slide from the side
    opposite the SCARA (D-012): place, SCARA clears, slide in, drive, slide out."""

    stroke: Pos | None      # m, retracted position to the working position, along y
    t_in: NonNeg | None     # s
    t_out: NonNeg | None    # s


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
    slide: Slide | None = None


class Drone(Model):
    """Features of the drone the line's tooling uses (D-012, option B)."""

    # One per arm, bottom plate frame: the pallet's arm pin passes through it into the arm.
    arm_tooling_holes: list[Vec2] | None = None
    tooling_pin_d: Pos | None = None
    pin_engagement: Pos | None = None


class Conveyor(Model):
    # No index time here: the pallet advances one pitch at layout.conveyor.belt_speed, so the
    # index time is derived (`index_s()`), never stated twice (D-010).
    pitch: Pos | None


ACTORS = ("scara", "driver", "operator", "sub_line", "test")


class Station(Model):
    index: PosInt
    count: PosInt
    pose: Pose
    cell_pitch: Pos | None = None
    # Who works here (D-010). Derived from the steps for scara/driver/test; stated only where a
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


class ScaraBase(Model):
    diameter: Pos | None        # floor-standing column under the shoulder
    height: Pos | None          # floor to its top


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
    scara_base: ScaraBase
    gang_head_box: Vec3         # [x, y, z] envelope of a gang head body (layout, safe_z)
    # [w (x), d (y), h (floor to top)] keyed by feeder or buffer id: both stand on the floor.
    feeder_footprints: dict[str, Vec3]
    # A multi-cell station's cells branch off in y. The end-of-line test bank that needed it is
    # deferred (docs/deferred_stations.yaml), so it is optional.
    test_branch: TestBranch | None = None
    operator_aisle: OperatorAisle
    guard_offset: Pos | None    # robot reach envelopes to the guard line, all round


class LineSpec(Model):
    schema_version: Literal[1]
    line: Line
    parts: dict[str, Part]
    cad: Cad | None = None
    drone: Drone | None = None
    fasteners: dict[str, Fastener]
    feeders: dict[str, Feeder]
    sources: dict[str, Source]
    buffers: dict[str, Buffer]
    scaras: dict[str, Scara]
    scara_defaults: ScaraDefaults
    gang_heads: dict[str, GangHead]
    conveyor: Conveyor
    stations: dict[str, Station]
    # {name: [station ids]}, in line order (D-012). Every station is in exactly one.
    milestones: dict[str, list[str]] | None = None
    sequence: list[Step]
    layout: Layout | None = None   # only tools/cad/layout.py needs it

    @model_validator(mode="after")
    def _scaras(self):
        errors = []
        for sid, sc in self.scaras.items():
            if sc.station not in self.stations:
                errors.append(f"scaras.{sid}.station: station '{sc.station}' does not exist")
            elif sid != scara_id(sc.station):
                errors.append(f"scaras.{sid}: the SCARA at {sc.station} is "
                              f"{scara_id(sc.station)} (scara_<station without st_>, "
                              "docs/naming.md)")
        # Every step that moves a part with a robot stands where a SCARA is.
        for s in self.sequence:
            if robot_served(s) and s.station in self.stations \
                    and scara_id(s.station) not in self.scaras:
                errors.append(f"sequence.{s.id}: moves a part with a robot but station "
                              f"'{s.station}' has no SCARA ({scara_id(s.station)})")
        # A SCARA places a part; a driver drives fasteners. One step, one kind of qty (D-009).
        for s in self.sequence:
            if robot_served(s) and (s.capability == "drive" or s.fastener is not None):
                errors.append(f"sequence.{s.id}: moves part '{s.part}' with a SCARA and drives "
                              f"fasteners ({s.capability}, qty {s.qty}) in one step; split it "
                              "into a place step and a drive step")
        # One tool id is one physical design: the CAD generators build it once.
        first: dict[str, tuple[str, Tool]] = {}
        for sid, sc in self.scaras.items():
            for tid, tool in sc.tools.items():
                if tid in first and first[tid][1] != tool:
                    errors.append(f"scaras.{sid}.tools.{tid} differs from "
                                  f"scaras.{first[tid][0]}.tools.{tid}; one tool id is one "
                                  "tool")
                first.setdefault(tid, (sid, tool))
        errors += self._milestone_errors()
        if errors:
            raise ValueError("; ".join(errors))
        return self

    def _milestone_errors(self) -> list[str]:
        """Every station in exactly one milestone, and the milestones, each read in order and
        taken by their first station, are the stations in line (index) order: a milestone is a
        contiguous run of the line. (Mapping order is not relied on: a YAML dump may sort it.)"""
        if self.milestones is None:
            return []
        errors, seen = [], {}
        for name, members in self.milestones.items():
            for st in members:
                if st not in self.stations:
                    errors.append(f"milestones.{name}: station '{st}' does not exist")
                elif st in seen:
                    errors.append(f"milestones.{name}: station '{st}' is already in milestone "
                                  f"'{seen[st]}'; a station is in exactly one")
                else:
                    seen[st] = name
        for st in self.stations:
            if st not in seen:
                errors.append(f"milestones: station '{st}' is in no milestone")
        if not errors:
            runs = sorted((m for m in self.milestones.values() if m),
                          key=lambda m: self.stations[m[0]].index)
            listed = [st for members in runs for st in members]
            line = sorted(self.stations, key=lambda s: self.stations[s].index)
            if listed != line:
                errors.append(f"milestones: read in order they give {listed}, but the line "
                              f"order (station index) is {line}")
        return errors


def scara_id(station: str) -> str:
    """st_place_spider -> scara_place_spider (D-012, docs/naming.md)."""
    return f"scara_{station.removeprefix('st_')}"


def robot_served(step: Step | dict) -> bool:
    """A step a SCARA performs: it names a tool, or draws a part `from` a feeder/buffer."""
    if isinstance(step, dict):
        tool, part, source = step.get("tool"), step.get("part"), step.get("from")
    else:
        tool, part, source = step.tool, step.part, step.from_
    return tool is not None or (part is not None and source is not None)


def station_scara(raw: dict, station: str | None) -> str | None:
    """The SCARA at `station`, from plain spec data (generators, xacro helpers)."""
    if station is None:
        return None
    sid = scara_id(station)
    return sid if sid in (raw.get("scaras") or {}) else None


def scara_ids(raw: dict) -> list[str]:
    """Every SCARA id, in the line order of the station it serves."""
    stations = raw.get("stations") or {}

    def order(sid):
        st = ((raw.get("scaras") or {}).get(sid) or {}).get("station")
        return ((stations.get(st) or {}).get("index") or 0, sid)
    return sorted(raw.get("scaras") or {}, key=order)


def scara_base(spec: LineSpec, sid: str) -> tuple[float, float, float] | None:
    """The base centre in the line frame: station x, station y +/- base_offset_y, station z.
    None while any of it is null."""
    sc = spec.scaras[sid]
    st = spec.stations.get(sc.station)
    if st is None or None in st.pose[:3] or sc.base_side is None or sc.base_offset_y is None:
        return None
    sign = 1.0 if sc.base_side == "+y" else -1.0
    return st.pose[0], st.pose[1] + sign * sc.base_offset_y, st.pose[2]


def head_station(raw: dict, hid: str) -> str | None:
    """The one station the sequence drives a gang head at, or None if it names zero or several."""
    stations = {s.get("station") for s in (raw.get("sequence") or [])
                if isinstance(s, dict) and s.get("head") == hid}
    return stations.pop() if len(stations) == 1 else None


def milestone_subset(raw: dict, name: str) -> dict:
    """The spec cut down to one milestone (D-012): its stations, the steps that stand there, their
    SCARAs and gang heads, the feeders and buffers those steps (and a sub_line station) draw
    from, the fasteners they drive and the sources that fill those buffers. A `requires` on a step
    outside the milestone is dropped: that step ran upstream. Raises SpecError for an unknown
    milestone. Parts, line, layout and defaults are kept whole."""
    milestones = raw.get("milestones") or {}
    if name not in milestones:
        raise SpecError([f"milestone '{name}' does not exist (have: "
                         f"{', '.join(milestones) or 'none'})"])
    out = copy.deepcopy(raw)
    keep = set(milestones[name])
    out["milestones"] = {name: list(milestones[name])}
    out["stations"] = {k: v for k, v in (raw.get("stations") or {}).items() if k in keep}
    steps = [s for s in raw.get("sequence") or [] if s.get("station") in keep]
    ids = {s["id"] for s in steps}
    out["sequence"] = []
    for s in steps:
        s = copy.deepcopy(s)
        s["requires"] = [r for r in s.get("requires") or [] if r in ids]
        out["sequence"].append(s)
    out["scaras"] = {k: v for k, v in (raw.get("scaras") or {}).items()
                     if (v or {}).get("station") in keep}
    out["gang_heads"] = {k: v for k, v in (raw.get("gang_heads") or {}).items()
                         if head_station(raw, k) in keep}
    drawn = {s.get("from") for s in steps} | {st.get("source") for st in out["stations"].values()}
    srcs = raw.get("sources") or {}
    drawn |= {srcs[d]["buffer"] for d in list(drawn) if d in srcs}
    fasteners = {s.get("fastener") for s in steps}
    out["fasteners"] = {k: v for k, v in (raw.get("fasteners") or {}).items() if k in fasteners}
    drawn |= {v.get("feeder") for v in out["fasteners"].values()}
    out["feeders"] = {k: v for k, v in (raw.get("feeders") or {}).items() if k in drawn}
    out["buffers"] = {k: v for k, v in (raw.get("buffers") or {}).items() if k in drawn}
    out["sources"] = {k: v for k, v in srcs.items() if v.get("buffer") in out["buffers"]}
    lay = out.get("layout")
    if isinstance(lay, dict) and isinstance(lay.get("feeder_footprints"), dict):
        lay["feeder_footprints"] = {k: v for k, v in lay["feeder_footprints"].items()
                                    if k in out["feeders"] or k in out["buffers"]}
    return out


def index_s(spec: LineSpec) -> float | None:
    """Seconds the pallet takes to advance one station: conveyor.pitch / layout.conveyor.belt_speed.
    The only index time there is (D-010); None while either is unknown."""
    lay = spec.layout
    if spec.conveyor.pitch is None or lay is None or lay.conveyor.belt_speed is None:
        return None
    return spec.conveyor.pitch / lay.conveyor.belt_speed


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
                      "scaras", "scaras.*.tools", "stations", "milestones",
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
          f"{len(s.stations)} stations, {len(s.scaras)} SCARAs, {len(s.sequence)} steps")
    if args.dump:
        dump_merged(m, args.dump, args.dev)
        print(f"spec: merged spec -> {args.dump}")
    if args.dev:
        print(f"spec: DEV — {len(m.placeholders)} placeholder value(s) from the overlay")
