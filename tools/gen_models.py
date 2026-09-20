"""Gazebo models for every part and magazine, from line_spec.yaml (plan P5.2).

    tools/.venv/bin/python tools/gen_models.py [--dev] [--spec PATH]

One `models/<id>/model.sdf` + `model.config` per `parts` entry and per magazine feeder. Each model
is the part as the rest of the line already describes it — nothing here is drawn or typed:

  * **collision** is a primitive from the envelope: `parts.<id>.envelope` for a SolidWorks part,
    `dims` for a primitive one, and the measured bounding box out of `reports/cad_<part>.json` for
    anything build123d generated (so the box is the solid that was actually built, not a guess);
  * **inertia** is computed from that primitive and `parts.<id>.mass` — the box (or cylinder)
    formula, about the part's own centre, which is why the collision pose and the inertial pose
    are the same number;
  * **visual** is the STL at scale 0.001 (the meshes are millimetres, the SDF is metres, and the
    STL is in the part's own frame so it needs no pose). A part whose mesh has not been built yet
    falls back to the collision primitive with a WARN — a missing mesh file is a hard world-load
    error in gz, so a world must never name one;
  * **frames** are the named poses the spec states: `<Part>_pick` from `parts.<id>.pick`, and
    `<Part>_datum_round` / `_datum_diamond` from `parts.<id>.datums` (CLAUDE.md naming).

A build123d part's mesh lives in `cad_out/`, which is not part of the package, so it is copied to
`meshes/tooling/<id>.stl` — the same place mesh_pipeline.py puts the SolidWorks meshes, and the
only copy a model refers to. Mesh URIs are written relative to the model file.

Mass: `parts.<id>.mass` when the spec states it. When it is null and a generator measured one
(`reports/cad_<part>.json`), that measurement is used and the report says so — it is a real
number, not a placeholder. A part with neither is a FAIL that names the key.

Outputs are generated files (CLAUDE.md P0): everything under `models/` is rewritten from the spec,
never hand-edited. `--dev` merges the placeholder overlay and quarantines the whole tree under
`models/dev/` and `meshes/tooling/dev/`, with the report at reports/gen_models.dev.json.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
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
MESH_SCALE = 0.001                 # the meshes are millimetres, the SDF is metres
HEADER = "GENERATED from line_spec.yaml by gen_models.py — do not edit"
ND = 9                             # significant digits in the SDF: far finer than a micron
ND_REPORT = 12                     # decimals in the report: a small part's inertia is ~1e-8

# Equipment stands still; a part is carried, placed and attached to (P7), so it needs physics.
STATIC_KINDS = {"magazine"}
COLOUR = {"part": "0.55 0.58 0.62 1", "magazine": "0.30 0.34 0.40 1"}


@dataclass
class Frame:
    name: str
    pose: str
    source: str


@dataclass
class Body:
    """One model, resolved from the spec (and, for generated parts, its generator's report)."""

    ident: str                      # spec id: the model directory, and `model://<ident>`
    name: str                       # Gazebo model name (CLAUDE.md naming)
    kind: str                       # part | magazine
    shape: str                      # box | cylinder
    size: tuple                     # box: (x, y, z); cylinder: (radius, length)
    centre: tuple[float, float, float]
    mass: float
    mass_from: str
    mesh: Path | None = None        # source STL, in millimetres
    frames: list[Frame] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def static(self) -> bool:
        return self.kind in STATIC_KINDS

    @property
    def inertia(self) -> tuple[float, float, float]:
        """(ixx, iyy, izz) about the body centre — the primitive's own formula."""
        if self.shape == "cylinder":
            r, h = self.size
            radial = self.mass * (3.0 * r * r + h * h) / 12.0
            return radial, radial, self.mass * r * r / 2.0
        x, y, z = self.size
        return (self.mass * (y * y + z * z) / 12.0,
                self.mass * (x * x + z * z) / 12.0,
                self.mass * (x * x + y * y) / 12.0)


# ---------------------------------------------------------------- resolving the spec

def fmt(value: float) -> str:
    return f"{round(value, ND):.{ND}g}"


def xyz_str(xyz) -> str:
    return " ".join(fmt(c) for c in xyz)


def pose_str(xyz, yaw: float = 0.0) -> str:
    return f"{xyz_str(xyz)} 0 0 {fmt(yaw)}"


def part_name(pid: str) -> str:
    """`carrier_spider` -> `Carrier_Spider` (CLAUDE.md naming conventions)."""
    return "_".join(pascal(word) for word in pid.split("_"))


def cad_report(part: str, dev: bool) -> dict:
    """Whatever the build123d generator for this part last wrote, or {}."""
    try:
        return json.loads(report_path(f"cad_{part}", dev).read_text())
    except (OSError, ValueError):
        return {}


def bbox_m(doc: dict) -> tuple[tuple, tuple] | None:
    """(size, centre) in metres from a cad report's millimetre bounding box."""
    box = doc.get("bbox_mm") or {}
    if not box.get("size") or not box.get("min"):
        return None
    size = tuple(c / 1000.0 for c in box["size"])
    centre = tuple((box["min"][i] + box["max"][i]) / 2000.0 for i in range(3))
    return size, centre


def part_frames(v: Values, pid: str, name: str) -> list[Frame]:
    """The named poses the spec states for this part: the pick point and the plate datums."""
    out = []
    pick = lookup(v.raw, f"parts.{pid}.pick")
    if pick and all(c is not None for c in pick[:3]):
        xyz = [v.m(f"parts.{pid}.pick[{i}]") for i in range(3)]
        out.append(Frame(f"{name}_pick", pose_str(xyz, float(pick[3] or 0.0)),
                         f"parts.{pid}.pick"))
    for datum in ("round", "diamond"):
        xy = lookup(v.raw, f"parts.{pid}.datums.{datum}")
        if xy and all(c is not None for c in xy):
            out.append(Frame(f"{name}_datum_{datum}", pose_str((xy[0], xy[1], 0.0)),
                             f"parts.{pid}.datums.{datum}"))
    return out


def part_body(v: Values, pid: str, dev: bool) -> Body | None:
    """Resolve one `parts` entry, or None when a value the model needs is null."""
    p = f"parts.{pid}"
    source = v.plain(f"{p}.source")
    name = part_name(pid)
    doc = cad_report(pid, dev) if source == "build123d" else {}
    shape, size, centre, mesh = "box", None, (0.0, 0.0, 0.0), None
    warnings = []

    if source == "build123d":
        measured = bbox_m(doc)
        if measured is None:
            v.missing.append(f"reports/{report_path(f'cad_{pid}', dev).name} "
                             f"(run {lookup(v.raw, f'{p}.generator')} first)")
            return None
        size, centre = measured
        mesh = Path(doc.get("outputs", {}).get("stl", "")) if doc.get("outputs") else None
    elif source == "primitive":
        shape = v.plain(f"{p}.shape")
        if shape == "cylinder":
            size = (v.m(f"{p}.dims[0]"), v.m(f"{p}.dims[1]"))
        else:
            size = tuple(v.m(f"{p}.dims[{i}]") for i in range(3))
    else:                                                   # solidworks
        size = tuple(v.m(f"{p}.envelope[{i}]") for i in range(3))
        # A plate is centred on its datum midpoint, not on its round datum (the frame origin);
        # it rests with its underside on z = 0. Same rule the pallet and the carriers use.
        rnd = lookup(v.raw, f"{p}.datums.round")
        dia = lookup(v.raw, f"{p}.datums.diamond")
        if rnd and dia and all(c is not None for c in (*rnd, *dia)):
            centre = ((rnd[0] + dia[0]) / 2.0, (rnd[1] + dia[1]) / 2.0, size[2] / 2.0)
        else:
            centre = (0.0, 0.0, size[2] / 2.0)
        rel = v.plain(f"{p}.mesh")
        mesh = PKG / rel if rel else None

    # A generated part's mass is measured, not stated: if the spec has no number and the
    # generator reported one, that measurement is the real value (and the report says so).
    measured = (doc.get("mass_kg") or {}).get("total")
    if lookup(v.raw, f"{p}.mass") is not None:
        mass, mass_from = v.plain(f"{p}.mass"), f"{p}.mass"
    elif measured:
        mass, mass_from = measured, f"reports/{report_path(f'cad_{pid}', dev).name}"
    else:
        v.missing.append(f"{p}.mass")
        return None
    if v.missing:
        return None
    return Body(ident=pid, name=name, kind="part", shape=shape, size=size, centre=centre,
                mass=float(mass), mass_from=mass_from, mesh=mesh,
                frames=part_frames(v, pid, name), warnings=warnings)


def magazine_bodies(v: Values, dev: bool) -> list[Body]:
    """One model per magazine, from what tools/cad/magazine.py measured and built."""
    try:
        run = json.loads(report_path("cad_magazines", dev).read_text())
    except (OSError, ValueError):
        run = {}
    out = []
    for fid, f in (v.raw.get("feeders") or {}).items():
        if not isinstance(f, dict) or f.get("type") != "magazine":
            continue
        item = (run.get("items") or {}).get(fid)
        name = f"Feeder_{pascal(fid.removeprefix('fd_'))}"
        measured = bbox_m(item) if item and item.get("status") == "PASS" else None
        if measured is None:
            v.missing.append(f"reports/{report_path('cad_magazines', dev).name}: no built "
                             f"magazine for {fid} (run tools/cad/magazine.py first)")
            continue
        size, centre = measured
        stl = item.get("outputs", {}).get("stl")
        out.append(Body(
            ident=fid, name=name, kind="magazine", shape="box", size=size, centre=centre,
            mass=float(item["mass_kg"]["total"]),
            mass_from=f"reports/{report_path('cad_magazines', dev).name}",
            mesh=Path(stl) if stl else None,
            # The magazine's origin is its pick plane, so the pick frame is the origin itself.
            frames=[Frame(f"{name}_pick", pose_str((0.0, 0.0, 0.0)),
                          "the rim: z=0 of tools/cad/magazine.py")]))
    return out


# ---------------------------------------------------------------- writing

def place_mesh(body: Body, mesh_dir: Path, model_dir: Path) -> str | None:
    """Make sure the model's mesh is inside the package, and return the URI to it.

    A SolidWorks mesh is already there (mesh_pipeline.py wrote it); a generated one lives in
    `cad_out/`, which is not installed with the package, so it is copied in. Returns None when
    nothing was built yet — the caller falls back to the collision primitive.
    """
    if body.mesh is None or not body.mesh.is_file():
        return None
    dest = body.mesh
    if PKG not in body.mesh.parents:
        dest = mesh_dir / f"{body.ident}.stl"
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(body.mesh, dest)
    return os.path.relpath(dest, model_dir)


def render(env: Environment, body: Body, mesh_uri: str | None) -> str:
    model = {"name": body.name, "static": body.static, "shape": body.shape,
             "size": [fmt(c) for c in body.size], "centre": xyz_str(body.centre),
             "mass": fmt(body.mass), "inertia": [fmt(c) for c in body.inertia],
             "mesh_uri": mesh_uri, "colour": COLOUR[body.kind],
             "frames": [f.__dict__ for f in body.frames]}
    return env.get_template("model.sdf.j2").render(
        m=model, header=HEADER, sdf_version=SDF_VERSION, mesh_scale=MESH_SCALE)


def config_xml(body: Body) -> str:
    return (f'<?xml version="1.0"?>\n<model>\n  <name>{body.name}</name>\n'
            f"  <version>1.0</version>\n"
            f'  <sdf version="{SDF_VERSION}">model.sdf</sdf>\n'
            f"  <description>{HEADER}</description>\n</model>\n")


def write_report(path: Path, doc: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate Gazebo models from the spec")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--models", type=Path, default=PKG / "models",
                    help="package models/ directory (default %(default)s)")
    ap.add_argument("--meshes", type=Path, default=PKG / "meshes" / "tooling",
                    help="where generated meshes are copied (default %(default)s)")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    models_dir = args.models / "dev" if args.dev else args.models
    mesh_dir = args.meshes / "dev" if args.dev else args.meshes
    doc: dict = {"passed": False, "dev": args.dev, "models_dir": str(models_dir),
                 "models": {}, "spec_values": {}, "placeholders": [], "missing": [],
                 "warnings": [], "errors": []}
    report = report_path("gen_models", args.dev)

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        write_report(report, doc)
        return code

    try:
        merged = load_merged(args.spec, args.dev)
        parse_spec(merged.raw)                      # schema first: a typo is not a missing value
    except SpecError as exc:
        doc["errors"] = exc.errors
        print(f"gen_models: FAIL — invalid spec: {exc}", file=sys.stderr)
        return finish(1)

    v = Values(merged.raw, {p["path"] for p in merged.placeholders})
    bodies = []
    for pid in merged.raw.get("parts") or {}:
        body = part_body(v, pid, args.dev)
        if body is not None:
            bodies.append(body)
    bodies += magazine_bodies(v, args.dev)
    doc["spec_values"] = v.used
    doc["placeholders"] = v.placeholders
    if v.missing:
        doc["missing"] = v.missing
        for m in v.missing:
            print(f"gen_models: FAIL — no value for {m}", file=sys.stderr)
        print("gen_models: fill the key(s) in, or re-run with --dev to use the placeholder "
              "overlay", file=sys.stderr)
        return finish(1)

    env = Environment(loader=FileSystemLoader(TEMPLATES), undefined=StrictUndefined,
                      keep_trailing_newline=True, trim_blocks=False)
    for body in bodies:
        model_dir = models_dir / body.ident
        try:
            model_dir.mkdir(parents=True, exist_ok=True)
            uri = place_mesh(body, mesh_dir, model_dir)
            (model_dir / "model.sdf").write_text(render(env, body, uri))
            (model_dir / "model.config").write_text(config_xml(body))
        except OSError as exc:
            doc["errors"].append(f"{body.ident}: cannot write {model_dir}: {exc}")
            print(f"gen_models: FAIL — {doc['errors'][-1]}", file=sys.stderr)
            return finish(1)
        if uri is None:
            body.warnings.append(
                f"no mesh built for {body.ident}: the visual falls back to the collision "
                "primitive (run mesh_pipeline.py or the part's generator)")
        doc["models"][body.ident] = {
            "name": body.name, "kind": body.kind, "static": body.static, "dir": str(model_dir),
            "shape": body.shape, "size_m": [round(c, 6) for c in body.size],
            "centre_m": [round(c, 6) for c in body.centre],
            "mass_kg": round(body.mass, 6), "mass_from": body.mass_from,
            "inertia_kgm2": [round(c, ND_REPORT) for c in body.inertia],
            "mesh_uri": uri, "frames": {f.name: f.pose for f in body.frames},
            "warnings": body.warnings}
        doc["warnings"] += [f"{body.ident}: {w}" for w in body.warnings]

    for w in doc["warnings"]:
        print(f"gen_models: WARN {w}", file=sys.stderr)
    if args.dev:
        print(f"gen_models: DEV MODE — {len(doc['placeholders'])} placeholder value(s); "
              f"{models_dir} is not a deliverable")
    meshed = sum(1 for m in doc["models"].values() if m["mesh_uri"])
    print(f"gen_models: OK — {len(doc['models'])} model(s), {meshed} with a mesh -> {models_dir}")
    return finish(0)


if __name__ == "__main__":
    sys.exit(main())
