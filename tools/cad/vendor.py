"""A vendor SCARA STEP, posed by its joints, for the layout (D-013).

    tools/.venv/bin/python tools/cad/vendor.py STEP [--model ID]   # print what was measured;
                                                    # with --model also write the capsules to
                                                    # reports/vendor_<ID>.json

The Epson LS10-B STEP (robot_models.<id>.step) is one static assembly: base, J1 link, J2 link
(a sub-assembly), Z shaft and a cable duct, in Epson's frame — Y up, the base mounting face at
y = 0, J1 on the Y axis, the arm stretched out along -X with the Z shaft at the top of its stroke.
Nothing about its joints is typed here: they are measured from the file.

  * J1: the Y axis (the base's mounting bore is centred on it);
  * J2: the axis of the largest vertical bore (< BORE_MAX_MM) in the J1 link away from J1;
  * the Z shaft: the centre of its bounding box in plan; its lower end (`shaft_bottom`, above the
    mounting face) is where the TCP is at the top of the stroke.

`posed(j1, j2)` returns the arm's solids, each named by its role (Base, Link1, Link2_<k>, Shaft),
in the robot's own frame: Z up, origin on J1 at the mounting face, +x where j1 = 0 points — one
leaf each in the layout tree, because a STEP compound leaf loses its colour on re-import. The
cable duct is left out: it is drawn for the stretched pose only and would cut through a folded
arm.

A file that is missing or not shaped like this raises VendorError; layout.py then keeps its
cylinder placeholder and WARNs.

Capsules (D-014): `capsules()` sizes the arm for tools/robot_geometry.py's park clearance — link1
around the segment J1 -> J2, link2 (without the shaft) around J2 -> the shaft axis, the shaft
around its axis: each radius is the farthest tessellated point of those solids from the segment,
in plan, and each z range is relative to the TCP at the top of its stroke (the shaft's lower
end). Written to reports/vendor_<robot model>.json by `--model`, and by layout.py.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from build123d import Axis, Compound, GeomType, Location, Pos, Rot, Shape, import_step

LINK_MIN_MM = 100.0      # a bore this far from J1 belongs to the far end of the J1 link
BORE_MAX_MM = 100.0      # a joint bore, not a large-radius styling surface


class VendorError(Exception):
    pass


@dataclass
class Scara:
    path: str
    base: list                 # (role, solid) that never move
    link1: list                # turn with j1
    link2: list                # turn with j1 and j2
    j2_axis: tuple[float, float]           # robot frame, mm
    shaft_xy: tuple[float, float]          # robot frame, mm
    shaft_bottom: float                    # mm above the mounting face, top of the stroke
    j1_step: float                         # rad, the pose the file is drawn in
    j2_step: float

    @property
    def link_lengths_mm(self) -> tuple[float, float]:
        return (math.hypot(*self.j2_axis),
                math.hypot(self.shaft_xy[0] - self.j2_axis[0], self.shaft_xy[1] - self.j2_axis[1]))

    def measured(self) -> dict:
        l1, l2 = self.link_lengths_mm
        return {"step": self.path, "link1_mm": round(l1, 2), "link2_mm": round(l2, 2),
                "reach_mm": round(l1 + l2, 2), "shaft_bottom_mm": round(self.shaft_bottom, 2),
                "pose_in_file_deg": [round(math.degrees(self.j1_step), 2),
                                     round(math.degrees(self.j2_step), 2)]}

    def posed(self, j1: float, j2: float) -> list[tuple[str, object]]:
        """The arm at (j1, j2), robot frame, millimetres: [(role, solid)]. Every solid is the
        loaded one at a new OCC location over the same geometry, never a copy (build123d's
        `moved()` deep-copies first): five arms of an 11 MB vendor STEP as copies ran a 7.8 GB
        machine out of memory (D-013)."""
        d1, d2 = math.degrees(j1 - self.j1_step), math.degrees(j2 - self.j2_step)
        r1 = Location((0, 0, 0), (0, 0, d1))
        jx, jy = self.j2_axis
        r2 = Location((jx, jy, 0)) * Location((0, 0, 0), (0, 0, d2)) * Location((-jx, -jy, 0))
        out = [(n, _at(s, Location())) for n, s in self.base]
        out += [(n, _at(s, r1)) for n, s in self.link1]
        out += [(n, _at(s, r1 * r2)) for n, s in self.link2]
        return out


def _at(shape, loc: Location):
    """`shape` at `loc`, sharing its geometry (no deep copy)."""
    return Shape.cast(shape.wrapped.Moved(loc.wrapped))


def _to_robot(shape):
    """Epson frame (Y up, arm along -X) -> robot frame (Z up, arm along +x):
    (x, y, z) -> (-x, z, y), a +90 deg turn about X then 180 deg about Z."""
    return Rot(0, 0, 180) * (Rot(90, 0, 0) * shape)


def _children(node) -> dict:
    return {c.label: c for c in (getattr(node, "children", ()) or ())}


@lru_cache(maxsize=4)
def load(path: str) -> Scara:
    p = Path(path)
    if not p.is_file():
        raise VendorError(f"{path}: no such file")
    try:
        top = import_step(str(p))
    except Exception as exc:                       # OCCT raises plain RuntimeErrors
        raise VendorError(f"{path}: cannot import ({exc})") from exc
    kids = _children(top)
    if len(kids) == 1 and not any(k.endswith(("_BASE", "_J1")) for k in kids):
        kids = _children(next(iter(kids.values())))
    role = {}
    for label, node in kids.items():
        for suffix in ("BASE", "J1", "J2", "Z", "CABLE"):
            if label.upper().endswith("_" + suffix):
                role[suffix] = node
    missing = [r for r in ("BASE", "J1", "J2", "Z") if r not in role]
    if missing:
        raise VendorError(f"{path}: no {', '.join(missing)} part (have {sorted(kids)})")
    robot = {r: [_to_robot(s) for s in node.solids()] for r, node in role.items()}
    names = {"BASE": "Base", "J1": "Link1", "J2": "Link2", "Z": "Shaft"}

    bores = []
    for s in robot["J1"]:
        for f in s.faces():
            if f.geom_type != GeomType.CYLINDER:
                continue
            ax = f.axis_of_rotation
            if abs(abs(ax.direction.Z) - 1.0) > 1e-3:
                continue
            xy = (ax.position.X, ax.position.Y)
            if math.hypot(*xy) > LINK_MIN_MM and f.radius < BORE_MAX_MM:
                bores.append((f.radius, xy))
    if not bores:
        raise VendorError(f"{path}: no J2 bore found in the J1 link")
    j2 = max(bores)[1]
    zb = Compound(robot["Z"]).bounding_box()
    shaft = ((zb.min.X + zb.max.X) / 2.0, (zb.min.Y + zb.max.Y) / 2.0)
    mount = Compound(robot["BASE"]).bounding_box().min.Z
    j1_step = math.atan2(j2[1], j2[0])
    j2_step = math.atan2(shaft[1] - j2[1], shaft[0] - j2[0]) - j1_step
    if abs(mount) > 1.0:                            # put the mounting face at z = 0
        robot = {r: [Pos(0, 0, -mount) * s for s in v] for r, v in robot.items()}
        zb = Compound(robot["Z"]).bounding_box()
    def named(r):
        sol = robot[r]
        return [(names[r] if len(sol) == 1 else f"{names[r]}_{i + 1}", x)
                for i, x in enumerate(sol)]
    return Scara(path=str(p), base=named("BASE"), link1=named("J1"),
                 link2=named("J2") + named("Z"), j2_axis=j2, shaft_xy=shaft,
                 shaft_bottom=zb.min.Z, j1_step=j1_step, j2_step=j2_step)


TESS_TOL_MM = 1.0
REPORTS = Path(__file__).resolve().parent.parent.parent / "reports"


def _seg_dist(a, b, p) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = dx * dx + dy * dy
    t = 0.0 if n == 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / n))
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1])


def capsules(model: Scara) -> dict:
    """{link1, link2, shaft: {radius_m, z_m: [lo, hi]}}, metres, z relative to the TCP at the top
    of its stroke. Tessellated points, so a curved face counts, not just its edges' vertices."""
    def points(solids):
        pts = []
        for _, s in solids:
            pts += [(v.X, v.Y, v.Z) for v in s.vertices()]
            try:
                verts, _ = s.tessellate(TESS_TOL_MM, 0.5)
            except AttributeError:          # a face OCCT cannot mesh: mesh the others alone
                verts = []
                for f in s.faces():
                    try:
                        verts += f.tessellate(TESS_TOL_MM, 0.5)[0]
                    except AttributeError:
                        continue
            pts += [(v.X, v.Y, v.Z) for v in verts]
        return pts

    link2 = [(n, s) for n, s in model.link2 if n != "Shaft"]
    shaft = [(n, s) for n, s in model.link2 if n == "Shaft"]
    out = {}
    for name, solids, a, b in (("link1", model.link1, (0.0, 0.0), model.j2_axis),
                               ("link2", link2, model.j2_axis, model.shaft_xy),
                               ("shaft", shaft, model.shaft_xy, model.shaft_xy)):
        pts = points(solids)
        out[name] = {"radius_m": round(max(_seg_dist(a, b, p) for p in pts) / 1000.0, 4),
                     "z_m": [round((min(p[2] for p in pts) - model.shaft_bottom) / 1000.0, 4),
                             round((max(p[2] for p in pts) - model.shaft_bottom) / 1000.0, 4)]}
    return out


def write_report(model: Scara, model_id: str, directory: Path = REPORTS) -> Path:
    """reports/vendor_<model id>.json: what was measured and the capsules (robot_geometry)."""
    path = directory / f"vendor_{model_id}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"generator": "tools/cad/vendor.py", "model": model_id,
                                **model.measured(), "capsules": capsules(model)}, indent=2)
                    + "\n")
    return path


def placed(model: Scara, j1: float, j2: float, x_mm: float, y_mm: float, z_mm: float,
           yaw: float) -> list[tuple[str, object]]:
    """The posed arm with its mounting face at (x, y, z) mm, j1 = 0 along `yaw` (rad)."""
    loc = Location((x_mm, y_mm, z_mm), (0, 0, math.degrees(yaw)))
    # Its own geometry (a parentless copy: ~15 MB, 0.2 s per arm): arms sharing one geometry are
    # written as instances of one STEP product, and re-import keeps one name for all of them.
    return [(n, copy.deepcopy(_at(s, loc))) for n, s in model.posed(j1, j2)]


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Measure a vendor SCARA STEP")
    ap.add_argument("step")
    ap.add_argument("--model", help="robot_models id: write reports/vendor_<ID>.json")
    args = ap.parse_args()
    try:
        m = load(args.step)
    except VendorError as exc:
        print(f"vendor: FAIL — {exc}", file=sys.stderr)
        sys.exit(1)
    print(m.measured())
    if args.model:
        print(f"vendor: capsules {capsules(m)} -> {write_report(m, args.model)}")
