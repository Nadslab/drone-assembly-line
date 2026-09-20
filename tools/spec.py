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
import sys
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
    type: Literal["magazine", "step", "blow"]
    part: str
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


class GangHead(Model):
    direction: Literal["down", "horizontal", "up"]  # up = driven from below the workpiece
    spindles: PosInt | None
    pattern_from: str | None = None
    opposed: bool = False
    stroke: Pos | None


class Scara(Model):
    base: Vec3 | None = None  # in line frame
    reach: Pos | None = None  # horizontal radius from base


class Conveyor(Model):
    pitch: Pos | None
    index_s: Pos | None


class Station(Model):
    index: PosInt
    count: PosInt
    pose: Pose
    cell_pitch: Pos | None = None


class Step(Model):
    id: str
    station: str
    capability: Literal["load", "place", "drive", "inspect", "test", "pack", "advance"]
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


class LineSpec(Model):
    schema_version: Literal[1]
    line: Line
    parts: dict[str, Part]
    cad: Cad | None = None
    fasteners: dict[str, Fastener]
    feeders: dict[str, Feeder]
    sources: dict[str, Source]
    buffers: dict[str, Buffer]
    gantry: Gantry
    gang_heads: dict[str, GangHead]
    scaras: dict[str, Scara]
    conveyor: Conveyor
    stations: dict[str, Station]
    sequence: list[Step]


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
                      "scaras", "stations", "gantry.tools"}


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
                if path in ENTITY_COLLECTIONS:
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


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Validate line_spec.yaml")
    ap.add_argument("path", nargs="?")
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
          f"{len(s.stations)} stations, {len(s.sequence)} steps")
    if args.dev:
        print(f"spec: DEV — {len(m.placeholders)} placeholder value(s) from the overlay")
