"""Plumbing shared by the build123d generators in tools/cad/ (plan P4).

Every generator reads the spec in metres and builds in millimetres (build123d's unit), refuses to
run when a value it needs is null, and writes reports/cad_<part>.json. That bookkeeping is the
same for the pallet, the carriers and the magazines, so it lives here; the geometry never does.

`Values` is the only way a generator touches the spec: it converts to millimetres, records every
path it read, and remembers which paths came from the dev overlay — so a report can list its
placeholders and a generator can refuse to treat a placeholder as a deliverable number.
"""

from __future__ import annotations

import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import trimesh
from build123d import Compound

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools: mesh_pipeline

import mesh_pipeline as mp  # noqa: E402

MM3_PER_M3 = 1e9


# ---------------------------------------------------------------- spec values

def lookup(data, path: str):
    """`parts.bottom_plate.envelope[0]` -> value, or None if any step is null or absent.

    Same path syntax as the overlay placeholders in spec.py, so a required key can be matched
    against `Merged.placeholders` to see whether it came from the dev overlay.
    """
    node = data
    for token in path.split("."):
        name, _, rest = token.partition("[")
        if not isinstance(node, dict):
            return None
        node = node.get(name)
        for index in re.findall(r"\d+", rest):
            if not isinstance(node, (list, tuple)) or int(index) >= len(node):
                return None
            node = node[int(index)]
        if node is None:
            return None
    return node


@dataclass
class Values:
    """Reads the spec in metres, hands out millimetres, and remembers what was missing."""

    raw: dict
    overlay: set[str] = field(default_factory=set)
    missing: list[str] = field(default_factory=list)
    used: dict[str, float] = field(default_factory=dict)
    placeholders: list[str] = field(default_factory=list)

    def mm(self, path: str) -> float:
        value = lookup(self.raw, path)
        if value is None:
            self.missing.append(path)
            return float("nan")
        self.used[path] = float(value)
        if path in self.overlay:
            self.placeholders.append(path)
        return float(value) * 1000.0

    def m(self, path: str) -> float:
        """The same read in the spec's own unit. Gazebo works in metres; build123d in mm."""
        return self.mm(path) / 1000.0

    def plain(self, path: str, default=None):
        """A value that is not a length: counts, names, enums. `default` means "optional"."""
        value = lookup(self.raw, path)
        if value is None:
            if default is None:
                self.missing.append(path)
            return default
        if path in self.overlay:
            self.placeholders.append(path)
        return value

    def is_placeholder(self, path: str) -> bool:
        """True when the value at `path` came from the dev overlay, so it is not a real number."""
        return path in self.overlay


# ---------------------------------------------------------------- shared spec reads

def placing_tool(v: Values, part_id: str) -> str | None:
    """The gantry tool the sequence uses to place `part_id`.

    A fixture's pick feature is sized by the tool that lifts it, and the spec already says which
    tool that is — the sequence step that places the fixture. Reading it here keeps the generator
    from naming a tool of its own.
    """
    steps = v.raw.get("sequence") or []
    named = [s for s in steps if isinstance(s, dict) and s.get("part") == part_id]
    tools = [s["tool"] for s in named if s.get("tool")]
    if not tools:
        v.missing.append(f"sequence: no step places '{part_id}' with a tool")
        return None
    if len(set(tools)) > 1:
        v.missing.append(f"sequence: '{part_id}' is placed with more than one tool: "
                         f"{sorted(set(tools))}")
        return None
    return tools[0]


def z_travel(v: Values) -> float:
    """Gantry z stroke in millimetres: everything the gantry carries has to fit inside it."""
    return v.mm("gantry.axes.z.max") - v.mm("gantry.axes.z.min")


def pick_xy(v: Values, part_id: str, default: tuple[float, float]) -> tuple[float, float]:
    """Where the tool grips, in millimetres: the spec's `pick` x,y if it states them.

    x,y are an input — they say where the operator wants the fixture picked up. z is not: it is
    wherever the generator's own pick feature ends up, so it is reported, never read.
    """
    stated = [lookup(v.raw, f"parts.{part_id}.pick[{i}]") for i in (0, 1)]
    if any(c is None for c in stated):
        return default
    for i in (0, 1):
        v.used[f"parts.{part_id}.pick[{i}]"] = float(stated[i])
    return (float(stated[0]) * 1000.0, float(stated[1]) * 1000.0)


def pick_z_note(v: Values, part_id: str, built_z: float, tol: float, doc: dict) -> None:
    """Compare a `pick` z the spec states against the pick feature actually built.

    The generator is the authority, so a real spec value that disagrees is an error — the CAD and
    the spec have drifted apart. A placeholder that disagrees is only a warning: the dev overlay
    is never a deliverable number (CLAUDE.md), so it has nothing to be right about.
    """
    path = f"parts.{part_id}.pick[2]"
    stated = lookup(v.raw, path)
    if stated is None:
        return
    delta = abs(float(stated) * 1000.0 - built_z)
    if delta <= tol:
        return
    message = (f"{path} is {float(stated) * 1000.0:.2f} mm but the pick boss tops out at "
               f"{built_z:.2f} mm ({delta:.2f} mm apart)")
    (doc["warnings"] if v.is_placeholder(path) else doc["errors"]).append(message)


def fouling(assembly: Compound, keep_out, boss_label: str) -> float:
    """Material of `assembly` inside the volume the gantry jaws sweep, ignoring the boss itself.

    Anything but zero is an interference: the tool cannot reach the pick feature.
    """
    return sum((child & keep_out).volume
               for child in assembly.children if child.label != boss_label)


# ---------------------------------------------------------------- solids

def solid(assembly: Compound, label: str):
    return next(c for c in assembly.children if c.label == label)


def masses(assembly: Compound, densities: dict[str, float], default: float) -> dict[str, float]:
    """kg per child solid, from volume x material density (kg/m3), plus the total."""
    out = {}
    for child in assembly.children:
        out[child.label] = round(child.volume / MM3_PER_M3
                                 * densities.get(child.label, default), 6)
    out["total"] = round(sum(out.values()), 6)
    return out


def bbox_doc(shape) -> dict:
    box = shape.bounding_box()
    return {"min": [round(c, 6) for c in tuple(box.min)],
            "max": [round(c, 6) for c in tuple(box.max)],
            "size": [round(c, 6) for c in tuple(box.size)]}


def xyz(point, nd: int = 6) -> list[float]:
    return [round(point.X, nd), round(point.Y, nd), round(point.Z, nd)]


# ---------------------------------------------------------------- outputs

def to_mesh(stl: Path, max_faces: int) -> dict:
    """Put an exported STL through mesh_pipeline: decimate and keep it under the 5 MB rule.

    `watertight` is reported, not enforced: solids that share a face with the pocket or bore they
    sit in leave the merged assembly mesh non-manifold there by construction. Gazebo takes the STL
    as a visual only (collision comes from primitives), so this is information.
    """
    mesh = trimesh.load_mesh(stl, file_type="stl", process=True)
    before, extents = len(mesh.faces), mesh.extents
    out, target = mesh, max_faces
    if before > max_faces:
        out = mp.decimate(mesh, max_faces)
    while mp.stl_bytes(out) >= mp.MAX_BYTES and target > 100:
        target = min(target, len(out.faces)) // 2
        out = mp.decimate(mesh, target)
    size = mp.write_mesh(out, stl)
    return {"faces_before": before, "faces_after": len(out.faces), "size_bytes": size,
            "watertight": bool(out.is_watertight),
            "bbox_m": [round(float(e) / 1000.0, 6) for e in out.extents],
            "decimation_bbox_drift_mm": round(float(abs(out.extents - extents).max()), 4)}


def export_solids(doc: dict, assembly, out_dir: Path, part: str, *, stl_tolerance_mm: float,
                  max_faces: int, bbox_tol_mm: float) -> str | None:
    """Write `<part>.step` + `<part>.stl`, mesh the STL, and check the two agree.

    Fills doc["outputs"] and doc["mesh"]; returns an error message, or None when all is well.
    A tessellation coarse enough to move the bounding box is an error, not a warning: the STL is
    what Gazebo sees, and every downstream check compares it against the spec envelope.
    """
    from build123d import export_step, export_stl

    out_dir.mkdir(parents=True, exist_ok=True)
    step, stl = out_dir / f"{part}.step", out_dir / f"{part}.stl"
    try:
        export_step(assembly, str(step))
        export_stl(assembly, str(stl), tolerance=stl_tolerance_mm)
    except OSError as exc:
        return f"cannot write {out_dir}: {exc}"
    doc["outputs"] = {"step": str(step), "stl": str(stl)}
    doc["mesh"] = to_mesh(stl, max_faces)
    drift = max(abs(doc["mesh"]["bbox_m"][i] * 1000.0 - doc["bbox_mm"]["size"][i])
                for i in range(3))
    doc["mesh"]["solid_bbox_drift_mm"] = round(drift, 4)
    if drift > bbox_tol_mm:
        return (f"STL bbox differs from the solid by {drift:.3f} mm "
                f"(tol {bbox_tol_mm:g} mm): tessellation is too coarse")
    return None


def write_report(path: Path, doc: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n")


def new_report(part: str, generator: str, out_dir: Path, dev: bool) -> dict:
    """The report skeleton every cad generator fills in, so the shape never drifts between them."""
    return {"passed": False, "dev": dev, "part": part, "generator": generator,
            "out": str(out_dir), "outputs": {}, "bbox_mm": {}, "mass_kg": {}, "derived": {},
            "mesh": {}, "spec_values": {}, "placeholders": [], "missing": [], "warnings": [],
            "errors": []}


def new_run_report(generator: str, out_dir: Path, dev: bool) -> dict:
    """The same skeleton for a generator that builds one solid per spec entry (P4 B5-B7).

    `items` holds one `new_item` per entry, keyed by the spec id it was built from; the run-level
    lists gather every item's, so a reader sees the whole run without walking the items.
    """
    return {"passed": False, "dev": dev, "generator": generator, "out": str(out_dir),
            "items": {}, "spec_values": {}, "placeholders": [], "missing": [], "warnings": [],
            "errors": []}


def new_item(part: str) -> dict:
    """One solid inside a run report — the per-part half of `new_report`, so export_solids fits."""
    return {"status": "FAIL", "part": part, "outputs": {}, "bbox_mm": {}, "mass_kg": {},
            "derived": {}, "mesh": {}, "missing": [], "warnings": [], "errors": []}


def collect(run: dict, key: str, item: dict, v: Values) -> None:
    """Fold one finished item, and the spec reads that built it, into the run report."""
    item["status"] = "FAIL" if item["errors"] or item["missing"] else "PASS"
    run["items"][key] = item
    run["spec_values"].update(v.used)
    for path in v.placeholders:
        if path not in run["placeholders"]:
            run["placeholders"].append(path)
    run["missing"] += [f"{key}: {m}" for m in item["missing"]]
    run["warnings"] += [f"{key}: {w}" for w in item["warnings"]]
    run["errors"] += [f"{key}: {e}" for e in item["errors"]]


def pascal(spec_id: str) -> str:
    """`mid_plate` -> `MidPlate`: the Gazebo name form of a yaml id (CLAUDE.md conventions)."""
    return "".join(word[:1].upper() + word[1:] for word in spec_id.split("_") if word)
