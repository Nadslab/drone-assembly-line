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
import math
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
    # Station-owned (D-013): `offset` in its station's anchor frame; `pose` is derived. A blow
    # feeder has neither: it rides its gang head.
    station: str | None = None
    offset: Pose | None = None
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
    # Station-owned (D-013): `offset` in its station's anchor frame; `pose` is derived from it.
    station: str | None = None
    offset: Pose | None = None
    pose: Pose | None = None


class ReturnPath(Model):
    """An empty-carrier return path (D-013): a chute or conveyor segment owned by the station
    whose SCARA lifts the carrier off the pallet (a `remove` step), running from where the
    SCARA puts the carrier down (`offset` in that station's frame: x, y, the put-down plane z,
    and the direction it runs) for `length`, to `sink` — the sub-line source (or buffer) the
    carrier goes back to. `pose` is derived."""

    carrier: str
    station: str
    kind: Literal["chute", "conveyor"]
    offset: Pose | None = None
    pose: Pose | None = None
    length: Pos | None
    width: Pos | None
    sink: str


class Tool(Model):
    """A SCARA tool (top-level `tools:`, D-013). `dims` is the envelope it needs at the pick
    feature, in metres: [x, y, reach] — x,y across the pick point and reach = how far it comes
    down over it. The fixture generators (P4 B3-B4) size their pick features from this. On a
    dual-tool end-effector each tool sits `offset_r` from the J4 axis, the two 180 deg apart;
    `mass` and `offset_r` feed the payload and wrist_inertia checks."""

    grip: Literal["vacuum", "finger"]
    dims: Vec3
    mass: Pos | None = None
    offset_r: NonNeg | None = None


class RobotJointLimits(Model):
    """Symmetric joint limits, +/- radians: j1 shoulder, j2 elbow, j4 wrist."""

    j1: Pos
    j2: Pos
    j4: Pos


class RobotMotion(Model):
    """TCP limits, Cartesian (the trapezoid estimate_moves.py plans with), and the wrist speed
    that times a dual-tool switch (180 deg about J4)."""

    v_max: Pos | None
    a_max: Pos | None
    j4_speed: Pos | None = None     # rad/s


class RobotModel(Model):
    """A SCARA product (D-013). Every `scaras` entry names one; nothing about the arm itself is
    stated per SCARA. `reach` = link1 + link2; the TCP works at z = base z .. base z + z_stroke
    (the base pose's z is the bottom of the stroke)."""

    vendor: str | None = None
    reach: Pos
    z_stroke: Pos
    payload_rated: Pos              # kg
    payload_max: Pos                # kg
    joint_limits: RobotJointLimits
    inertia_rated: Pos              # kg m^2 about J4
    repeatability: Pos              # m
    step: str | None = None         # vendor STEP (layout.py places it at each base)
    link1: Pos | None
    link2: Pos | None
    motion: RobotMotion

    @model_validator(mode="after")
    def _links(self):
        if self.link1 is not None and self.link2 is not None \
                and abs(self.link1 + self.link2 - self.reach) > 1e-6:
            raise ValueError(f"link1 {self.link1} + link2 {self.link2} != reach {self.reach}")
        return self


class Scara(Model):
    """One SCARA per pick-and-place station (D-012), keyed `scara_<station without st_>`.

    D-013: the arm is a `robot_models` entry; the base is placed by `offset` [dx, dy, dz, yaw] in
    its station's anchor frame — dz is the bottom of the TCP's z stroke, yaw the direction j1 = 0
    points — and `pose` is derived from it (resolve_anchors). `tools` is the end-effector: one
    tool, or a dual-tool pair switched by a 180 deg J4 turn."""

    station: str
    model: str
    offset: Pose | None = None
    pose: Pose | None = None
    tools: list[str]


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
    # D-013: the working pose of the head frame in its station's anchor frame (the station is the
    # one its drive step stands at); `pose` is derived.
    offset: Pose | None = None
    pose: Pose | None = None


class ArmPins(Model):
    """The pallet's arm pins are retractable (D-013): an `actuate` step extends them, and they
    retract at `unload`, the end-of-line station that is deferred (docs/deferred_stations.yaml);
    it gets its own actuate step when that station is re-added."""

    retractable: bool
    extend_step: str
    retract_at: Literal["unload"]


class Drone(Model):
    """Features of the drone the line's tooling uses (D-012, D-013)."""

    # One per arm, bottom plate frame: the pallet's arm pin passes through it into the arm.
    arm_tooling_holes: list[Vec2] | None = None
    tooling_pin_d: Pos | None = None
    pin_engagement: Pos | None = None
    arm_pins: ArmPins | None = None


class Conveyor(Model):
    # No index time here: the pallet advances one pitch at layout.conveyor.belt_speed, so the
    # index time is derived (`index_s()`), never stated twice (D-010).
    pitch: Pos | None


ACTORS = ("scara", "driver", "operator", "sub_line", "test")


class Anchor(Model):
    """Where a station stands (D-013): `gap` metres along +x of the station it comes `after` (its
    datum to this one's), or of the line origin for the first station (`after: origin`)."""

    after: str
    gap: NonNeg | None


class Station(Model):
    index: PosInt
    count: PosInt
    anchor: Anchor | None = None
    pose: Pose = (None, None, None, None)   # derived from the anchor chain (resolve_anchors)
    cell_pitch: Pos | None = None
    # Who works here (D-010). Derived from the steps for scara/driver/test; stated only where a
    # step cannot say it: an operator, or a sub_line that loads the pallet from `source` (a buffer
    # or a source). check_line.py `actors` holds every station to exactly one actor type.
    actor: Literal[ACTORS] | None = None
    source: str | None = None


class Step(Model):
    id: str
    station: str
    # actuate (D-013): a station device moves (the pallet's arm pins extend); no actor.
    # remove (D-013): the SCARA lifts `part` off the pallet and puts it on return path `to`.
    capability: Literal["load", "place", "pick", "drive", "inspect", "test", "pack", "advance",
                        "actuate", "remove"]
    duration_s: NonNeg | None
    requires: list[str]
    part: str | None = None
    from_: str | None = Field(default=None, alias="from")
    tool: str | None = None
    to: str | None = None           # a remove step's return path
    head: str | None = None
    fastener: str | None = None
    qty: PosInt | None = None
    status: Literal["active", "parked"] = "active"
    device: Literal["arm_pins"] | None = None     # what an `actuate` step moves
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
    return_paths: dict[str, ReturnPath] = Field(default_factory=dict)
    robot_models: dict[str, RobotModel]
    tools: dict[str, Tool]
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
        # D-013: the arm is a robot model, the end-effector a list of top-level tools.
        for sid, sc in self.scaras.items():
            if sc.model not in self.robot_models:
                errors.append(f"scaras.{sid}.model: robot model '{sc.model}' does not exist")
            for tid in sc.tools:
                if tid not in self.tools:
                    errors.append(f"scaras.{sid}.tools: tool '{tid}' does not exist")
            if len(sc.tools) != len(set(sc.tools)):
                errors.append(f"scaras.{sid}.tools: a tool is listed twice")
            if len(sc.tools) > 2:
                errors.append(f"scaras.{sid}.tools: {len(sc.tools)} tools; an end-effector is "
                              "one tool or a dual-tool pair")
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
    """The base centre in the line frame (its resolved pose; z = the bottom of the TCP's z
    stroke). None while any of it is null."""
    pose = spec.scaras[sid].pose
    if pose is None or None in pose[:3]:
        return None
    return pose[0], pose[1], pose[2]


def scara_yaw(spec: LineSpec, sid: str) -> float | None:
    """The direction j1 = 0 points, line frame, radians."""
    pose = spec.scaras[sid].pose
    return None if pose is None else pose[3]


def scara_model(spec: LineSpec, sid: str) -> "RobotModel":
    return spec.robot_models[spec.scaras[sid].model]


def scara_side(spec: LineSpec, sid: str) -> str | None:
    """"+y" or "-y": the side of its station the SCARA's base stands on (station frame)."""
    off = spec.scaras[sid].offset
    if off is None or off[1] is None or off[1] == 0:
        return None
    return "+y" if off[1] > 0 else "-y"


# ---------------------------------------------------------------- anchors (D-013)

ORIGIN = "origin"
# Everything a station owns is placed by `offset` in that station's anchor frame.
OWNED = ("feeders", "buffers", "scaras", "gang_heads", "return_paths")


def compose(frame, offset) -> list[float | None]:
    """`offset` [dx, dy, dz, yaw] in `frame` [x, y, z, yaw] -> a line-frame pose. A component is
    None when anything it depends on is."""
    fx, fy, fz, fyaw = (list(frame) + [None] * 4)[:4]
    dx, dy, dz, dyaw = (list(offset) + [None] * 4)[:4]
    if None in (fx, fyaw, dx, dy):
        x = None
    else:
        x = fx + math.cos(fyaw) * dx - math.sin(fyaw) * dy
    y = None if None in (fy, fyaw, dx, dy) else fy + math.sin(fyaw) * dx + math.cos(fyaw) * dy
    z = None if None in (fz, dz) else fz + dz
    yaw = None if None in (fyaw, dyaw) else fyaw + dyaw
    return [None if c is None else round(c, 12) for c in (x, y, z, yaw)]


def owner_station(raw: dict, coll: str, eid: str) -> str | None:
    """The station whose anchor frame an owned entity's offset is in: its `station`, or for a
    gang head the one station its drive step stands at."""
    if coll == "gang_heads":
        return head_station(raw, eid)
    return ((raw.get(coll) or {}).get(eid) or {}).get("station")


def _compose_deps(frame, fdeps, offset, opaths) -> list[set[str]]:
    """Which spec paths each component of compose(frame, offset) depends on: x on the frame's x,
    and on dx / dy only where the frame's yaw gives them a share (a straight line along +x does
    not make y depend on the gap)."""
    yaw = frame[3] or 0.0
    c, s_ = abs(math.cos(yaw)) > 1e-12, abs(math.sin(yaw)) > 1e-12
    dx, dy, dz, dyaw = opaths
    out = [fdeps[0] | fdeps[3] | ({dx} if c else set()) | ({dy} if s_ else set()),
           fdeps[1] | fdeps[3] | ({dx} if s_ else set()) | ({dy} if c else set()),
           fdeps[2] | {dz}, fdeps[3] | {dyaw}]
    return [d - {None} for d in out]


def resolve_anchors(raw: dict) -> tuple[dict, dict[str, set[str]]]:
    """Station poses from their anchor chain, then every owned entity's pose from its station
    and offset (D-013). Returns (a copy of raw with `pose` filled in, {resolved pose path: the
    spec paths it was computed from}), per pose component.

    Idempotent: a station whose `after` is not in this spec (a milestone subset: that station
    ran upstream) keeps the pose it was resolved to, if any. Entities without an anchor/offset
    are left alone — the `anchors` check reports them as absolute poses, and a dangling `after`
    too. Raises SpecError on an anchor cycle."""
    out = copy.deepcopy(raw)
    stations = out.get("stations") or {}
    deps: dict[str, set[str]] = {}
    none4 = [set(), set(), set(), set()]
    done: dict[str, tuple[list, list[set[str]]]] = {}

    def station(sid: str, chain: tuple[str, ...]):
        if sid in done:
            return done[sid]
        if sid in chain:
            raise SpecError([f"stations.{sid}.anchor: anchor cycle "
                             f"{' -> '.join(chain + (sid,))}"])
        st = stations[sid] or {}
        anchor = st.get("anchor")
        if not isinstance(anchor, dict):
            done[sid] = (list(st.get("pose") or [None] * 4), none4)
            return done[sid]
        after, gap = anchor.get("after"), anchor.get("gap")
        if after == ORIGIN:
            frame, fdeps = [0.0, 0.0, 0.0, 0.0], none4
        elif after in stations:
            frame, fdeps = station(after, chain + (sid,))
        else:
            # Anchored upstream of a milestone subset (resolved before the cut), or to a
            # station that does not exist: the `anchors` check reports the latter.
            done[sid] = (list(st.get("pose") or [None] * 4), none4)
            return done[sid]
        pose = compose(frame, [gap, 0.0, 0.0, 0.0])
        pdeps = _compose_deps(frame, fdeps, [gap, 0.0, 0.0, 0.0],
                              [f"stations.{sid}.anchor.gap", None, None, None])
        st["pose"] = pose
        for i in range(4):
            deps[f"stations.{sid}.pose[{i}]"] = pdeps[i]
        done[sid] = (pose, pdeps)
        return done[sid]

    for sid in stations:
        station(sid, ())
    for coll in OWNED:
        for eid, ent in (out.get(coll) or {}).items():
            if not isinstance(ent, dict) or ent.get("offset") is None:
                continue
            st = owner_station(out, coll, eid)
            if st not in stations:
                continue        # resolved before a subset cut, or a dangling station (schema)
            frame, fdeps = station(st, ())
            ent["pose"] = compose(frame, ent["offset"])
            pdeps = _compose_deps(frame, fdeps, ent["offset"],
                                  [f"{coll}.{eid}.offset[{i}]" for i in range(4)])
            for i in range(4):
                deps[f"{coll}.{eid}.pose[{i}]"] = pdeps[i]
    return out, deps


def authored_pose_errors(data: dict, where: str) -> list[str]:
    """A `pose` written into the file next to an anchor or offset: poses are derived (D-013)."""
    errors = []
    for sid, st in (data.get("stations") or {}).items():
        if isinstance(st, dict) and "anchor" in st and "pose" in st:
            errors.append(f"{where}: stations.{sid} has both `anchor` and `pose`; the pose is "
                          "derived from the anchor")
    for coll in OWNED:
        for eid, ent in (data.get(coll) or {}).items():
            if isinstance(ent, dict) and "offset" in ent and "pose" in ent:
                errors.append(f"{where}: {coll}.{eid} has both `offset` and `pose`; the pose is "
                              "derived from the offset")
    return errors


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
    out, _ = resolve_anchors(raw)       # poses first: the cut drops the stations they chain from
    raw = out
    out = copy.deepcopy(out)
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
    out["return_paths"] = {k: v for k, v in (raw.get("return_paths") or {}).items()
                           if (v or {}).get("station") in keep}
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
                      "scaras", "robot_models", "tools", "return_paths", "stations",
                      "milestones", "layout.feeder_footprints"}


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
    """The spec as plain YAML data (nulls preserved), anchors resolved into line-frame poses
    (D-013), plus overlay provenance when dev: a resolved pose computed from any placeholder is a
    placeholder too (its record carries `derived_from`)."""
    p = Path(path) if path else DEFAULT_SPEC
    base = _read_yaml(p)
    errors = authored_pose_errors(base, p.name)
    if dev:
        overlay = _read_yaml(overlay_path(p))
        errors += authored_pose_errors(overlay, overlay_path(p).name)
    if errors:
        raise SpecError(errors)
    m = merge_overlay(base, overlay) if dev else Merged(raw=base)
    m.raw, deps = resolve_anchors(m.raw)
    ph = {r["path"] for r in m.placeholders}
    for path, used in deps.items():
        if used & ph:
            node = m.raw
            name, i = path.rsplit("[", 1)
            for k in name.split("."):
                node = node[k]
            m.placeholders.append({"path": path, "value": node[int(i[:-1])],
                                   "derived_from": sorted(used & ph)})
    return m


def load_raw(path: str | Path | None = None, dev: bool = False) -> dict:
    return load_merged(path, dev).raw


def parse_spec(raw: dict) -> LineSpec:
    """Validate; anchors are resolved first (idempotent), so a raw dict built by hand parses the
    same as one from load_merged."""
    raw, _ = resolve_anchors(raw)
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
