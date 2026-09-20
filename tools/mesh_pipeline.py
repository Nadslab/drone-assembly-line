"""Raw SolidWorks STL (mm) -> Gazebo-ready meshes, checked against line_spec.yaml (plan P3.3).

    tools/.venv/bin/python tools/mesh_pipeline.py [--src DIR] [--spec PATH] [--dev]
    tools/.venv/bin/python tools/mesh_pipeline.py --src /mnt/c/Users/Admin/Documents/drone-line-cad/40_exports/stl

For every `source: solidworks` part the spec names a mesh, `meshes/<group>/<name>.stl`. The raw
file is looked up as `<src>/<name>.stl` (then `<src>/<group>/<name>.stl`) and, per part:

  1. loaded and its units confirmed by bounding-box magnitude (mm expected);
  2. decimated (quadric edge collapse) when above --max-faces, and always kept under 5 MB;
  3. origin check: the round-datum hole (Ø cad.datum_d) must sit at parts.<id>.datums.round,
     within --origin-tol-mm. Parts without datums.round in the spec skip this with a WARN;
  4. bbox extents in metres compared with parts.<id>.envelope: a null component is filled, a
     set one that differs by more than --bbox-tol-mm is a FAIL.

Output stays in millimetres: the 0.001 scale is applied in the SDF, not here. Bbox, hole and
origin are measured on the raw mesh; the decimated mesh must keep the same bbox.

Outputs are committed only when they pass: a failing part writes no mesh, and the spec is only
patched in place (ruamel locates the nulls; comments and layout untouched) when nothing failed. A part whose raw STL is missing is
a WARN, so the tool can run while CAD is incomplete; a missing --src directory is a FAIL.
Writes reports/meshes.json. Exits non-zero on any FAIL.

--dev merges the placeholder overlay for the comparisons, never writes the spec, and reports to
reports/meshes.dev.json.
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pymeshlab
import trimesh
from ruamel.yaml import YAML
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

sys.path.insert(0, str(Path(__file__).resolve().parent))

from spec import (DEFAULT_SPEC, REPO, LineSpec, SpecError, add_dev_argument,  # noqa: E402
                  load_merged, parse_spec, report_path)

MESH_ROOT = REPO / "drone_line_sim"
DEFAULT_SRC = REPO / "cad_raw"
MAX_BYTES = 5 * 1024 * 1024          # hard limit; the repo never commits an STL above this
STL_BYTES_PER_FACE = 50              # binary STL: 50 bytes per triangle + 84 header
MM_EXTENT_RANGE = (5.0, 3000.0)      # largest bbox side of a plausible part, in mm
CIRCLE_RADIAL_TOL = 0.02             # max radial spread / radius for a loop to count as a circle


@dataclass
class Result:
    part: str
    status: str = "PASS"             # PASS | WARN | FAIL
    messages: list[str] = field(default_factory=list)
    info: dict = field(default_factory=dict)
    mesh: trimesh.Trimesh | None = None      # decimated mesh, ready to write
    fills: dict[int, float] = field(default_factory=dict)   # envelope index -> metres

    def warn(self, msg: str) -> None:
        self.messages.append(f"WARN {msg}")
        if self.status == "PASS":
            self.status = "WARN"

    def fail(self, msg: str) -> None:
        self.messages.append(f"FAIL {msg}")
        self.status = "FAIL"


# ---------------------------------------------------------------- geometry

def section_loops(mesh: trimesh.Trimesh, z: float) -> list[np.ndarray]:
    """Closed loops (as XY point sets) where the plane Z=z cuts the mesh.

    Works on the raw plane/triangle segments so it needs neither networkx nor shapely.
    """
    segs = trimesh.intersections.mesh_plane(mesh, [0.0, 0.0, 1.0], [0.0, 0.0, z])
    if len(segs) == 0:
        return []
    pts = segs[:, :, :2].reshape(-1, 2)
    _, first, ids = np.unique(np.round(pts, 6), axis=0, return_index=True, return_inverse=True)
    ids = ids.reshape(-1, 2)
    n = len(first)
    graph = coo_matrix((np.ones(len(ids)), (ids[:, 0], ids[:, 1])), shape=(n, n))
    _, label = connected_components(graph, directed=False)
    degree = np.bincount(ids.ravel(), minlength=n)
    loops = []
    for k in range(label.max() + 1):
        members = label == k
        if np.all(degree[members] == 2):                # every vertex has two neighbours: closed
            loops.append(pts[first[members]])
    return loops


def find_circular_holes(mesh: trimesh.Trimesh, diameter: float, tol: float) -> list[np.ndarray]:
    """XY centres of closed circular loops of `diameter` (± tol) in the mid-height section."""
    lo, hi = mesh.bounds[:, 2]
    centres = []
    for pts in section_loops(mesh, (lo + hi) / 2.0):
        if len(pts) < 8:
            continue                                    # too coarse to be a tessellated circle
        c = pts.mean(axis=0)
        r = np.linalg.norm(pts - c, axis=1)
        if abs(2.0 * r.mean() - diameter) > tol or (r.max() - r.min()) > CIRCLE_RADIAL_TOL * r.mean():
            continue
        centres.append(c)
    return centres


def decimate(mesh: trimesh.Trimesh, faces: int) -> trimesh.Trimesh:
    ms = pymeshlab.MeshSet()
    ms.add_mesh(pymeshlab.Mesh(vertex_matrix=np.asarray(mesh.vertices, dtype=np.float64),
                               face_matrix=np.asarray(mesh.faces, dtype=np.int32)))
    ms.meshing_decimation_quadric_edge_collapse(
        targetfacenum=faces, preserveboundary=True, preservenormal=True,
        preservetopology=True, qualitythr=0.5)
    m = ms.current_mesh()
    return trimesh.Trimesh(m.vertex_matrix(), m.face_matrix(), process=True)


def stl_bytes(mesh: trimesh.Trimesh) -> int:
    return 84 + STL_BYTES_PER_FACE * len(mesh.faces)


# ---------------------------------------------------------------- per part

def process_part(pid: str, raw_stl: Path, spec: LineSpec, *, max_faces: int, bbox_tol_mm: float,
                 origin_tol_mm: float, hole_tol_mm: float, overlay_paths: set[str]) -> Result:
    res = Result(pid)
    part = spec.parts[pid]
    try:
        mesh = trimesh.load_mesh(raw_stl, file_type="stl", process=True)
    except Exception as exc:  # noqa: BLE001 — trimesh raises many types on a bad file
        res.fail(f"cannot load {raw_stl}: {exc}")
        return res
    faces_before = len(mesh.faces)
    extents_mm = mesh.extents
    res.info.update(src=str(raw_stl), faces_before=faces_before,
                    watertight=bool(mesh.is_watertight),
                    bbox_m=[round(float(e) / 1000.0, 6) for e in extents_mm])
    if not mesh.is_watertight:
        res.warn("mesh is not watertight")

    # 1. units
    biggest = float(extents_mm.max())
    if not MM_EXTENT_RANGE[0] <= biggest <= MM_EXTENT_RANGE[1]:
        res.fail(f"largest bbox side {biggest:g} is not plausible for millimetres "
                 f"({MM_EXTENT_RANGE[0]:g}..{MM_EXTENT_RANGE[1]:g}); exported in metres or inches?")
        return res

    # 4. envelope (before decimation: the raw CAD is the truth)
    env = list(part.envelope) if part.envelope else [None, None, None]
    status = []
    for i, ax in enumerate("xyz"):
        got = float(extents_mm[i]) / 1000.0
        if env[i] is None:
            res.fills[i] = round(got, 6)
            status.append("filled")
        elif abs(env[i] - got) * 1000.0 > bbox_tol_mm:
            status.append("mismatch")
            res.fail(f"envelope.{ax} spec {env[i] * 1000:.3f} mm != STL {got * 1000:.3f} mm "
                     f"(tol {bbox_tol_mm:g} mm): CAD and spec disagree")
        else:
            status.append("match")
        if env[i] is not None and f"parts.{pid}.envelope[{i}]" in overlay_paths:
            res.info.setdefault("placeholders", []).append(f"parts.{pid}.envelope[{i}]")
    res.info["envelope_status"] = status
    res.info["envelope_spec_m"] = env

    # 3. origin
    datums = part.datums
    rnd = datums.round if datums else None
    if rnd is None or None in rnd:
        res.warn("origin check skipped: spec has no parts.%s.datums.round" % pid)
    elif spec.cad is None:
        res.warn("origin check skipped: spec has no cad.datum_d")
    else:
        ph = [p for p in (f"parts.{pid}.datums.round[0]", f"parts.{pid}.datums.round[1]",
                          "cad.datum_d") if p in overlay_paths]
        if ph:
            res.info.setdefault("placeholders", []).extend(ph)
        want = np.array(rnd) * 1000.0
        holes = find_circular_holes(mesh, spec.cad.datum_d * 1000.0, hole_tol_mm)
        if not holes:
            res.fail(f"no Ø{spec.cad.datum_d * 1000:g} mm hole found at mid-thickness "
                     "(plate must lie in XY; is the round datum hole modelled?)")
        else:
            hole = min(holes, key=lambda c: float(np.linalg.norm(c - want)))
            err = float(np.linalg.norm(hole - want))
            res.info["origin"] = {"expected_m": [float(v) for v in rnd],
                                  "found_m": [round(float(v) / 1000.0, 6) for v in hole],
                                  "error_m": round(err / 1000.0, 7), "candidates": len(holes)}
            if err > origin_tol_mm:
                res.fail(f"round datum hole at ({hole[0]:.3f}, {hole[1]:.3f}) mm is {err:.3f} mm "
                         f"from the spec ({want[0]:.3f}, {want[1]:.3f}); tol {origin_tol_mm:g} mm. "
                         "Re-export with the origin at the hole")

    # 2. decimate
    out = mesh
    if faces_before > max_faces:
        out = decimate(mesh, max_faces)
    target = max_faces
    while stl_bytes(out) >= MAX_BYTES and target > 100:
        target = min(target, len(out.faces)) // 2
        out = decimate(mesh, target)
    if stl_bytes(out) >= MAX_BYTES:
        res.fail(f"cannot get below {MAX_BYTES} bytes")
        return res
    drift = float(np.abs(out.extents - extents_mm).max())
    if drift > bbox_tol_mm:
        res.fail(f"decimation changed the bbox by {drift:.3f} mm (tol {bbox_tol_mm:g} mm)")
    res.info.update(faces_after=len(out.faces), size_bytes=stl_bytes(out),
                    decimation_bbox_drift_mm=round(drift, 4))
    res.mesh = out
    return res


# ---------------------------------------------------------------- outputs

def write_mesh(mesh: trimesh.Trimesh, dest: Path) -> int:
    dest.parent.mkdir(parents=True, exist_ok=True)
    data = trimesh.exchange.stl.export_stl(mesh)
    dest.write_bytes(data)
    return len(data)


def write_envelopes(spec_path: Path, fills: dict[str, dict[int, float]]) -> None:
    """Fill null envelope components in the base spec, touching nothing else.

    ruamel's round trip keeps comments but re-spaces the hand-aligned flow mappings (~100 lines
    of churn in line_spec.yaml), so ruamel is used to *locate* each `null` (line/column of the
    envelope item) and only those characters are replaced in the original text.
    """
    text = spec_path.read_text()
    doc = YAML(typ="rt").load(text)
    lines = text.splitlines(keepends=True)
    edits = []                                           # (line, col, value)
    for pid, comps in fills.items():
        seq = doc["parts"][pid]["envelope"]
        for i, v in comps.items():
            line, col = seq.lc.item(i)
            if not lines[line].startswith("null", col):
                raise OSError(f"parts.{pid}.envelope[{i}] is not a literal `null` at "
                              f"{spec_path.name}:{line + 1}")
            edits.append((line, col, v))
    for line, col, v in sorted(edits, reverse=True):     # right-to-left keeps columns valid
        lines[line] = lines[line][:col] + repr(float(v)) + lines[line][col + 4:]
    with tempfile.NamedTemporaryFile("w", dir=spec_path.parent, delete=False,
                                     prefix=".spec.", suffix=".tmp") as tmp:
        tmp.write("".join(lines))
    Path(tmp.name).replace(spec_path)


def write_report(path: Path, doc: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n")


def find_raw(src: Path, mesh_rel: str) -> Path | None:
    rel = Path(mesh_rel)
    for cand in (src / rel.name, src / rel.parent.name / rel.name):
        if cand.is_file():
            return cand
    return None


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Raw STL (mm) -> decimated, checked Gazebo meshes")
    ap.add_argument("--src", type=Path, default=DEFAULT_SRC, help="folder of raw STL (mm)")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--max-faces", type=int, default=50000,
                    help="decimate meshes with more faces than this")
    ap.add_argument("--bbox-tol-mm", type=float, default=0.2)
    ap.add_argument("--origin-tol-mm", type=float, default=0.1)
    ap.add_argument("--hole-tol-mm", type=float, default=0.15,
                    help="tolerance on the datum hole diameter when identifying it")
    ap.add_argument("--out", type=Path, default=MESH_ROOT,
                    help="package root that spec mesh paths (meshes/<group>/x.stl) are under")
    add_dev_argument(ap)
    args = ap.parse_args(argv)

    report = report_path("meshes", args.dev)
    doc: dict = {"passed": False, "dev": args.dev, "src": str(args.src), "meshes": {},
                 "unclaimed": [], "spec_updated": False, "placeholders": [], "errors": []}

    def finish(code: int) -> int:
        doc["passed"] = code == 0
        write_report(report, doc)
        return code

    try:
        merged = load_merged(args.spec, args.dev)
        spec = parse_spec(merged.raw)
    except SpecError as exc:
        doc["errors"] = exc.errors
        print(f"mesh_pipeline: FAIL — invalid spec: {exc}", file=sys.stderr)
        return finish(1)
    if not args.src.is_dir():
        doc["errors"].append(f"source folder not found: {args.src}")
        print(f"mesh_pipeline: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return finish(1)

    overlay_paths = {p["path"] for p in merged.placeholders}
    results: list[Result] = []
    claimed: set[Path] = set()
    for pid, part in spec.parts.items():
        if part.source != "solidworks":
            continue
        mesh_rel = part.mesh
        if not mesh_rel or len(Path(mesh_rel).parts) != 3 or Path(mesh_rel).parts[0] != "meshes":
            r = Result(pid)
            r.fail(f"mesh '{mesh_rel}' must look like meshes/<group>/<name>.stl")
            results.append(r)
            continue
        raw = find_raw(args.src, mesh_rel)
        if raw is None:
            r = Result(pid)
            r.warn(f"no raw STL {Path(mesh_rel).name} under {args.src}; skipped")
            results.append(r)
            continue
        claimed.add(raw.resolve())
        r = process_part(pid, raw, spec, max_faces=args.max_faces, bbox_tol_mm=args.bbox_tol_mm,
                         origin_tol_mm=args.origin_tol_mm, hole_tol_mm=args.hole_tol_mm,
                         overlay_paths=overlay_paths)
        r.info["mesh"] = mesh_rel
        results.append(r)

    for stl in sorted(args.src.rglob("*.stl")):
        if stl.resolve() not in claimed:
            doc["unclaimed"].append(str(stl))
            print(f"mesh_pipeline: WARN {stl.name} is not the mesh of any solidworks part",
                  file=sys.stderr)

    failed = any(r.status == "FAIL" for r in results)
    for r in results:
        if r.mesh is not None and r.status != "FAIL":
            dest = args.out / r.info["mesh"]
            r.info["size_bytes"] = write_mesh(r.mesh, dest)
            r.info["out"] = str(dest)
        doc["meshes"][r.part] = {"status": r.status, "messages": r.messages, **r.info}
        doc["placeholders"] += r.info.get("placeholders", [])
        for m in r.messages:
            print(f"mesh_pipeline: {r.part}: {m}", file=sys.stderr)

    fills = {r.part: r.fills for r in results if r.fills and r.status != "FAIL"}
    if fills and not failed and not args.dev:
        try:
            write_envelopes(args.spec, fills)
            doc["spec_updated"] = True
        except OSError as exc:
            doc["errors"].append(f"cannot write {args.spec}: {exc}")
            print(f"mesh_pipeline: FAIL — {doc['errors'][-1]}", file=sys.stderr)
            return finish(1)
    elif fills and args.dev:
        print("mesh_pipeline: DEV MODE — spec not updated, envelopes not filled", file=sys.stderr)
    elif fills:
        print("mesh_pipeline: spec not updated because a part failed", file=sys.stderr)

    n = {s: sum(r.status == s for r in results) for s in ("PASS", "WARN", "FAIL")}
    print(f"mesh_pipeline: {'FAIL' if failed else 'OK'} — {n['PASS']} pass, {n['WARN']} warn, "
          f"{n['FAIL']} fail; {sum(len(f) for f in fills.values())} envelope value(s) "
          f"{'filled' if doc['spec_updated'] else 'not written'}")
    return finish(1 if failed else 0)


if __name__ == "__main__":
    sys.exit(main())
