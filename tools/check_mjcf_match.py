"""Check the generated MJCF against the placement code it came from (M1).

    tools/.venv/bin/python tools/check_mjcf_match.py [--dev] [--stations IDS [--name N]]
                                                     [--xml PATH] [--no-render] [--step M1.3]

The reference is never the exporter: it is spec.py's resolved poses, robot_geometry's kinematics
and arm capsules, and tools/cad/layout.py's `plan()` solids and `interference()` — what the layout
STEP and its checks use. The MJCF is loaded from disk, as a viewer or mujoco_ros2_control would.

Samples: every keyframe in the file — `folded` (as layout.py draws the line) and one
`parked_<scara>` per SCARA that parks (its station's gang heads in, working). No motion YAML exists
yet, so these are static poses, not times.

Poses (tolerance POS_TOL / ANG_TOL):
  * every spec-ID body (st_, scara_, gh_, fd_, buf_, rt_) vs its resolved spec pose (position and
    yaw), and its sites: <station>_in/_out at the stop unit's edges, <id>_mount at the pose,
    scara_<s>_mount at the mounting face (TCP top - the vendor's shaft_bottom);
  * at every sample, each SCARA's elbow and TCP sites vs robot_geometry.fk;
  * every layout solid's world AABB (the union of its geoms, visual meshes excluded) vs the
    reference: the layout shape's bounding box, or for an arm link the capsule robot_geometry
    checks with, at that sample's joints; a gang head at its working box when parked; a carrier's
    visual mesh vs the union of its layout solids.
Collisions, at every sample:
  * reference: layout.interference() on the layout solids at that sample (a parked arm is the
    vendor STEP posed by vendor.placed at its park_pose, the head moved to its working box) — the
    overlapping pairs; and every pair whose bounding boxes overlap by more than BBOX_EPS on each
    axis (layout.interference's own prefilter, with that tolerance so float noise on two touching
    faces does not count);
  * MuJoCo: per-geom world AABBs for the same bbox pair list, and mj_geomDistance on every geom
    pair of a non-allowed candidate (layout.allowed, the same ALLOWED_OVERLAPS): an overlap is a
    penetration deeper than PEN_TOL. mj_collision is run too, but it never tests two geoms that
    are both welded to the world (every static body here), so it only sees pairs with a moving
    arm or head; its contacts must be a subset of the geomDistance overlaps.
  Pairs inside one assembly (one SCARA's pedestal, housing and link capsules; one carrier's
  per-solid boxes) are not compared on either side: the convex proxies of one arm overlap at its
  joints and a carrier's boxes nest, where the vendor/carrier STEP solids do not. They are listed
  in the report. The MJCF excludes contact between one SCARA's bodies for the same reason.
  Both lists must be equal. Envelopes are visual-only and are left out of the pair lists on both
  sides in the overlap set (allowed with everything), kept in the bbox list.
Also renders verify/m1_<name>_{iso,top}.png (folded, guards and envelopes hidden, offscreen,
MUJOCO_GL=egl; skipped below 1 GB free).

Report: reports/mjcf_match[.<name>][.dev].json; last line `<step> PASS|FAIL <summary>`.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

TOOLS = Path(__file__).resolve().parent
sys.path[:0] = [str(TOOLS), str(TOOLS / "cad")]
import gen_mjcf  # noqa: E402  (names and output paths only)
import layout  # noqa: E402
import robot_geometry as rg  # noqa: E402
import vendor  # noqa: E402
from common import Values, pascal  # noqa: E402
from spec import (DEFAULT_SPEC, REPO, SpecError, add_dev_argument, load_merged,  # noqa: E402
                  parse_spec, parse_station_list, report_path, station_subset)

POS_TOL = 0.001                 # m
ANG_TOL = math.radians(0.1)     # rad
BBOX_EPS = 1e-6                 # m: bboxes overlapping less than this on an axis only touch
PEN_TOL = 1e-4                  # m: a penetration shallower than 0.1 mm is not an overlap
VERIFY = REPO / "verify"
TOP_FOVY = 4.0                  # deg


def yaw_of(xmat) -> float:
    m = np.asarray(xmat).reshape(3, 3)
    return math.atan2(m[1, 0], m[0, 0])


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------- reference side

def keyed(it: layout.Item) -> layout.Item:
    """A copy of the solid named by its MJCF geom key, so layout.interference reports keys."""
    return dataclasses.replace(it, name=gen_mjcf.item_key(it), shape=None)


def sample_items(items, line, v, cad, sample: str):
    """The layout solids at a sample: as planned, or with one arm parked and its heads in."""
    if not sample.startswith("parked_"):
        return [keyed(it) for it in items]
    sid = sample.removeprefix("parked_")
    j1, j2 = rg.park_joints(line, sid)
    name = f"Scara_{pascal(sid.removeprefix('scara_'))}"
    st = line.scaras[sid].station
    heads = {}
    for hid in line.gang_heads:
        hb = rg.head_box(line, hid, cad)
        if hb.station == st and hb.working is not None:
            heads[f"GangHead_{pascal(hid.removeprefix('gh_'))}"] = hb
    out = []
    for it in items:
        if it.subgroup == name and it.solid is not None:
            continue                                   # the folded vendor solids: re-posed below
        it = keyed(it)
        if it.name in heads:
            w = heads[it.name].working
            it.centre = (layout.Dim((w.x0 + w.x1) / 2.0), layout.Dim((w.y0 + w.y1) / 2.0),
                         it.centre[2])
        out.append(it)
    rm = line.robot_models[line.scaras[sid].model]
    model = vendor.load(rm.step)
    k = rg.kin(line, sid)
    mount = k.z_hi - model.shaft_bottom / 1000.0
    for role, solid in vendor.placed(model, j1, j2, k.base[0] * 1000.0, k.base[1] * 1000.0,
                                     mount * 1000.0, k.yaw):
        bb = solid.bounding_box()
        out.append(keyed(layout.Item(
            f"{name}_{role}", "robot", "Robots",
            tuple(layout.Dim(c / 1000.0) for c in tuple(bb.center())),
            tuple(layout.Dim(c / 1000.0) for c in tuple(bb.size)), subgroup=name, solid=solid)))
    return out


def reference_pairs(its) -> tuple[dict, set, set]:
    """(key -> AABB in m, bbox pairs, overlap pairs) from layout.py's own shapes and check."""
    for it in its:
        it.shape = layout.build_shape(it)
    raw = []                                      # mm, one optimal bbox per solid
    for it in its:
        bb = it.shape.bounding_box()
        raw.append((np.array([bb.min.X, bb.min.Y, bb.min.Z]),
                    np.array([bb.max.X, bb.max.Y, bb.max.Z])))
    boxes = {}
    for it, (lo, hi) in zip(its, raw):
        lo, hi = lo / 1000.0, hi / 1000.0
        if it.name in boxes:
            lo, hi = np.minimum(lo, boxes[it.name][0]), np.maximum(hi, boxes[it.name][1])
        boxes[it.name] = (lo, hi)
    bbox = set()
    for i in range(len(its)):
        for j in range(i + 1, len(its)):
            if its[i].name == its[j].name:
                continue
            lo = np.maximum(raw[i][0], raw[j][0])
            hi = np.minimum(raw[i][1], raw[j][1])
            if np.all(hi - lo > BBOX_EPS * 1000.0):
                bbox.add(frozenset((its[i].name, its[j].name)))
    res = layout.interference(its)
    overlaps = {frozenset((f["a"].removesuffix("_PH"), f["b"].removesuffix("_PH")))
                for f in res["failures"]}
    return boxes, bbox, overlaps


def capsule_boxes(line, cad, sid: str, j1: float, j2: float) -> dict:
    """The arm links' AABBs as robot_geometry's capsules (TCP at the top of its stroke)."""
    k = rg.kin(line, sid)
    caps = cad.vendor(line.scaras[sid].model)["capsules"]
    l1, _ = k.links
    a = k.yaw + j1
    j = (k.base[0], k.base[1])
    e = (k.base[0] + l1 * math.cos(a), k.base[1] + l1 * math.sin(a))
    t = rg.fk(k, j1, j2)
    name = f"Scara_{pascal(sid.removeprefix('scara_'))}"
    out = {}
    for key, p, q, c in ((f"{name}_Link1", j, e, caps["link1"]),
                         (f"{name}_Link2", e, t, caps["link2"]),
                         (f"{name}_Shaft", t, t, caps["shaft"])):
        r = c["radius_m"]
        out[key] = (np.array([min(p[0], q[0]) - r, min(p[1], q[1]) - r, k.z_hi + c["z_m"][0]]),
                    np.array([max(p[0], q[0]) + r, max(p[1], q[1]) + r, k.z_hi + c["z_m"][1]]))
    return out, e, t, k


# ---------------------------------------------------------------- MuJoCo side

def geom_aabb(m, d, g: int):
    """World AABB of geom g: exact for box, cylinder and mesh (its vertices)."""
    R = d.geom_xmat[g].reshape(3, 3)
    p = d.geom_xpos[g]
    t = m.geom_type[g]
    s = m.geom_size[g]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        h = np.abs(R) @ s
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        ax = R[:, 2]
        h = s[0] * np.sqrt(np.clip(1.0 - ax * ax, 0.0, 1.0)) + s[1] * np.abs(ax)
    elif t == mujoco.mjtGeom.mjGEOM_MESH:
        mid = m.geom_dataid[g]
        a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        w = m.mesh_vert[a:a + n] @ R.T + p
        return w.min(axis=0), w.max(axis=0)
    else:
        c, hh = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
        cw = p + R @ c
        h = np.abs(R) @ hh
        return cw - h, cw + h
    return p - h, p + h


def mj_state(m, d, keys: set):
    """key -> [geom ids], key -> AABB; visual meshes left out."""
    geoms: dict[str, list[int]] = {}
    for g in range(m.ngeom):
        name = m.geom(g).name
        if m.geom_group[g] == gen_mjcf.G_VISUAL:
            continue
        key = name.split("__")[0]
        geoms.setdefault(key, []).append(g)
    boxes = {}
    for key, gs in geoms.items():
        los, his = zip(*(geom_aabb(m, d, g) for g in gs))
        boxes[key] = (np.min(los, axis=0), np.max(his, axis=0))
    return geoms, boxes


def assembly_of(it: layout.Item) -> str:
    """One SCARA (pedestal, housing, links, shaft, envelope) or one carrier is one assembly."""
    key = gen_mjcf.item_key(it)
    if key.startswith("Scara_"):
        return "_".join(key.split("_")[:2])
    return it.subgroup if it.category == "carrier" and it.subgroup else key


def mj_pairs(m, d, geoms, boxes, cat, owner, assembly):
    keys = sorted(boxes)
    bbox, overlaps, depth, intra = set(), set(), {}, set()
    ft = np.zeros(6)
    for i, a in enumerate(keys):
        for b in keys[i + 1:]:
            lo = np.maximum(boxes[a][0], boxes[b][0])
            hi = np.minimum(boxes[a][1], boxes[b][1])
            if not np.all(hi - lo > BBOX_EPS):
                continue
            pair = frozenset((a, b))
            bbox.add(pair)
            if layout.allowed(cat[a], cat[b], owner.get(a), owner.get(b)):
                continue
            if assembly[a] == assembly[b]:
                intra.add(pair)                # proxies of one arm or carrier: not a clash
                continue
            dmin = min(mujoco.mj_geomDistance(m, d, ga, gb, 0.01, ft)
                       for ga in geoms[a] for gb in geoms[b])
            if dmin < -PEN_TOL:
                overlaps.add(pair)
                depth[pair] = dmin
    # mj_collision: only pairs with a moving body; its contacts must be geomDistance overlaps.
    mujoco.mj_collision(m, d)
    contacts = set()
    for c in d.contact[:d.ncon]:
        a, b = m.geom(c.geom1).name.split("__")[0], m.geom(c.geom2).name.split("__")[0]
        if a != b and c.dist < -PEN_TOL and assembly[a] != assembly[b] and \
                not layout.allowed(cat[a], cat[b], owner.get(a), owner.get(b)):
            contacts.add(frozenset((a, b)))
    return bbox, overlaps, depth, contacts, d.ncon, intra


def render(m, path_stem: Path, key: int) -> dict:
    """Folded line, guards and envelopes hidden: iso, and top (plan, +x right, +y up) through a
    TOP_FOVY camera from far overhead, close to orthographic."""
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, key)
    mujoco.mj_forward(m, d)
    opt = mujoco.MjvOption()
    for g in range(6):
        opt.geomgroup[g] = g in (gen_mjcf.G_PHYSICAL, gen_mjcf.G_REFERENCE, gen_mjcf.G_VISUAL)
    shown = [g for g in range(m.ngeom) if m.geom_group[g] in (gen_mjcf.G_PHYSICAL,
                                                                gen_mjcf.G_VISUAL)]
    lo = np.min([geom_aabb(m, d, g)[0] for g in shown], axis=0)
    hi = np.max([geom_aabb(m, d, g)[1] for g in shown], axis=0)
    centre, span = (lo + hi) / 2.0, float(np.max(hi - lo))
    out = {}
    fovy = float(m.vis.global_.fovy)
    for view in ("iso", "top"):
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = centre
        if view == "iso":
            m.vis.global_.fovy = fovy
            cam.azimuth, cam.elevation, cam.distance = 235.0, -32.0, span * 2.0
        else:                       # near-orthographic: a narrow view from far overhead
            m.vis.global_.fovy = TOP_FOVY
            extent = max(hi[1] - lo[1], (hi[0] - lo[0]) / 1.6) * 1.15
            cam.azimuth, cam.elevation = 90.0, -90.0
            cam.distance = extent / (2.0 * math.tan(math.radians(TOP_FOVY / 2.0))) + hi[2] - centre[2]
        with mujoco.Renderer(m, 1000, 1600) as r:
            r.update_scene(d, camera=cam, scene_option=opt)
            img = r.render()
        p = Path(f"{path_stem}_{view}.png")
        p.parent.mkdir(parents=True, exist_ok=True)
        _write_png(p, img)
        out[view] = str(p)
    m.vis.global_.fovy = fovy
    return out


def _write_png(path: Path, img) -> None:
    from PIL import Image
    Image.fromarray(img).save(path)


# ---------------------------------------------------------------- the check

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="MJCF vs placement code match check (M1)")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--stations", nargs="+", default=None)
    ap.add_argument("--name", default="batch")
    ap.add_argument("--xml", type=Path, default=None, help="default the exporter's output path")
    ap.add_argument("--cad-reports", type=Path, default=None)
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--step", default="M1.3", help="label of the closing PASS|FAIL line")
    add_dev_argument(ap)
    args = ap.parse_args(argv)
    t_start = time.monotonic()
    stations = parse_station_list(args.stations)
    name = args.name if stations else None
    xml = args.xml or gen_mjcf.MJCF_DIR / gen_mjcf.output_name(name, args.dev)
    report = report_path(f"mjcf_match.{name}" if name else "mjcf_match", args.dev)
    scope = name or "line"
    doc: dict = {"passed": False, "dev": args.dev, "checker": "tools/check_mjcf_match.py",
                 "mujoco": mujoco.__version__, "xml": str(xml), "stations": stations,
                 "tolerances": {"pos_m": POS_TOL, "ang_deg": math.degrees(ANG_TOL),
                                "bbox_eps_m": BBOX_EPS, "penetration_m": PEN_TOL},
                 "poses": {"checked": 0, "failures": []}, "samples": {}, "errors": [],
                 "warnings": []}

    def finish(code: int, summary: str) -> int:
        doc["passed"] = code == 0
        doc["elapsed_s"] = round(time.monotonic() - t_start, 1)
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(doc, indent=2, ensure_ascii=False, default=str) + "\n")
        for e in doc["errors"]:
            print(f"check_mjcf_match: FAIL — {e}", file=sys.stderr)
        print(f"{args.step} {'PASS' if code == 0 else 'FAIL'} {summary}")
        return code

    try:
        merged = load_merged(args.spec, args.dev)
        if stations:
            merged.raw = station_subset(merged.raw, stations)
        line = parse_spec(merged.raw)
    except SpecError as exc:
        doc["errors"] = list(exc.errors)
        return finish(1, "spec does not load")
    if not xml.is_file():
        doc["errors"].append(f"{xml} does not exist (run tools/gen_mjcf.py)")
        return finish(1, "no MJCF")
    if "GENERATED" not in xml.read_text()[:300]:
        doc["errors"].append(f"{xml} has no GENERATED header")
    m = mujoco.MjModel.from_xml_path(str(xml))
    d = mujoco.MjData(m)
    v = Values(merged.raw, {p["path"] for p in merged.placeholders})
    cad = rg.CadHeights(args.cad_reports, args.dev)
    items = layout.plan(v, line, cad, {"warnings": [], "vendor": {}})
    if items is None:
        doc["errors"].append(f"layout.plan: no value for {sorted(set(v.missing))}")
        return finish(1, "placement incomplete")
    cat = {gen_mjcf.item_key(it): it.category for it in items}
    owner = {gen_mjcf.item_key(it): it.owner for it in items}
    assembly = {gen_mjcf.item_key(it): assembly_of(it) for it in items}
    pf = doc["poses"]["failures"]

    def pos_check(what, got, want, ang=None):
        doc["poses"]["checked"] += 1
        err = float(np.linalg.norm(np.asarray(got) - np.asarray(want)))
        row = {"what": what, "err_mm": round(err * 1000.0, 4)}
        if ang is not None:
            row["err_deg"] = round(math.degrees(abs(wrap(ang[0] - ang[1]))), 4)
        if err > POS_TOL or (ang is not None and abs(wrap(ang[0] - ang[1])) > ANG_TOL):
            pf.append(row)
        return row

    # ---- spec-ID bodies and their sites (folded; they do not move)
    key_ids = {m.key(k).name: k for k in range(m.nkey)}
    mujoco.mj_resetDataKeyframe(m, d, key_ids["folded"])
    mujoco.mj_forward(m, d)
    stop = line.layout.stop_unit_length
    coll = {"station": line.stations, "feeder": line.feeders, "buffer": line.buffers,
            "return": line.return_paths}
    expected_bodies = []
    for kind, c in coll.items():
        for eid, ent in c.items():
            if ent.pose is None or ent.pose[0] is None:
                continue                         # a blow feeder rides its head: nothing placed
            p = list(ent.pose) + [None] * (4 - len(ent.pose))
            expected_bodies.append((eid, (p[0], p[1], p[2]), p[3] or 0.0, kind))
    for hid in line.gang_heads:
        hb = rg.head_box(line, hid, cad)
        p = line.gang_heads[hid].pose or line.stations[hb.station].pose
        expected_bodies.append((hid, tuple(p[:3]), (p[3] if len(p) > 3 else None) or 0.0,
                                "gang_head"))
    for sid in line.scaras:
        k = rg.kin(line, sid)
        expected_bodies.append((sid, k.base, k.yaw, "scara"))
    for eid, xyz, yaw, kind in expected_bodies:
        try:
            b = m.body(eid)
        except KeyError:
            pf.append({"what": f"body {eid}", "error": "missing from the MJCF"})
            continue
        pos_check(f"body {eid}", d.xpos[b.id], xyz, (yaw_of(d.xmat[b.id]), yaw))
        if kind == "station":
            c, s = math.cos(yaw), math.sin(yaw)
            for site, sign in (("in", -1.0), ("out", 1.0)):
                want = (xyz[0] + sign * stop / 2.0 * c, xyz[1] + sign * stop / 2.0 * s, xyz[2])
                pos_check(f"site {eid}_{site}", d.site(f"{eid}_{site}").xpos, want)
        elif kind == "scara":
            caps = cad.vendor(line.scaras[eid].model)
            k = rg.kin(line, eid)
            pos_check(f"site {eid}_mount", d.site(f"{eid}_mount").xpos,
                      (k.base[0], k.base[1], k.z_hi - caps["shaft_bottom_mm"] / 1000.0))
        else:
            pos_check(f"site {eid}_mount", d.site(f"{eid}_mount").xpos, xyz)

    # ---- samples: per keyframe, solids' AABBs, arm sites, and the collision lists
    ok_sets = True
    for kname, kid in key_ids.items():
        mujoco.mj_resetDataKeyframe(m, d, kid)
        mujoco.mj_forward(m, d)
        row: dict = {"aabb_checked": 0}
        its = sample_items(items, line, v, cad, kname)
        ref_boxes, ref_bbox, ref_all = reference_pairs(its)
        ref_over = {p for p in ref_all if len({assembly[k] for k in p}) == 2}
        geoms, boxes = mj_state(m, d, set(ref_boxes))
        # arm links: robot_geometry capsules at this sample's joints, and the arm sites
        for sid in line.scaras:
            j1 = float(d.qpos[m.jnt_qposadr[m.joint(f"{sid}_j1").id]])
            j2 = float(d.qpos[m.jnt_qposadr[m.joint(f"{sid}_j2").id]])
            cb, elbow, tcp, k = capsule_boxes(line, cad, sid, j1, j2)
            ref_boxes.update(cb)
            pos_check(f"{kname}: site {sid}_elbow", d.site(f"{sid}_elbow").xpos[:2], elbow)
            pos_check(f"{kname}: site {sid}_tcp", d.site(f"{sid}_tcp").xpos, (*tcp, k.z_hi))
        for key, (lo, hi) in ref_boxes.items():
            if key not in boxes:
                pf.append({"what": f"{kname}: solid {key}", "error": "no geom in the MJCF"})
                continue
            row["aabb_checked"] += 1
            err = max(float(np.max(np.abs(boxes[key][0] - lo))),
                      float(np.max(np.abs(boxes[key][1] - hi))))
            doc["poses"]["checked"] += 1
            if err > POS_TOL:
                pf.append({"what": f"{kname}: AABB {key}", "err_mm": round(err * 1000.0, 4),
                           "mjcf": [boxes[key][0].round(4).tolist(), boxes[key][1].round(4).tolist()],
                           "ref": [lo.round(4).tolist(), hi.round(4).tolist()]})
        extra = sorted(set(boxes) - set(ref_boxes))
        if extra:
            pf.append({"what": f"{kname}: geoms with no layout solid", "keys": extra})
        # carrier meshes vs their layout solids
        for g in range(m.ngeom):
            if m.geom_group[g] != gen_mjcf.G_VISUAL:
                continue
            sub = m.geom(g).name.removesuffix("_visual")
            parts = [ref_boxes[kk] for kk in ref_boxes if kk.startswith(sub + "_")]
            if parts:
                lo, hi = np.min([p[0] for p in parts], axis=0), np.max([p[1] for p in parts], axis=0)
                mlo, mhi = geom_aabb(m, d, g)
                err = max(float(np.max(np.abs(mlo - lo))), float(np.max(np.abs(mhi - hi))))
                doc["poses"]["checked"] += 1
                if err > POS_TOL:
                    pf.append({"what": f"{kname}: mesh {m.geom(g).name}",
                               "err_mm": round(err * 1000.0, 4)})
        mj_bbox, mj_over, depth, contacts, ncon, intra = mj_pairs(m, d, geoms, boxes, cat,
                                                                  owner, assembly)

        def names(pairs):
            return sorted(" ∩ ".join(sorted(p)) for p in pairs)
        row.update({
            "reference_overlaps": names(ref_over), "mjcf_overlaps": names(mj_over),
            "mjcf_depth_mm": {" ∩ ".join(sorted(p)): round(x * 1000.0, 3) for p, x in depth.items()},
            "reference_overlaps_within_assembly": names(ref_all - ref_over),
            "mjcf_pairs_within_assembly_not_compared": names(intra),
            "bbox_pairs": len(ref_bbox), "mjcf_bbox_pairs": len(mj_bbox),
            "bbox_only_reference": names(ref_bbox - mj_bbox),
            "bbox_only_mjcf": names(mj_bbox - ref_bbox),
            "mj_collision": {"ncon": ncon, "non_allowed_pairs": names(contacts),
                             "not_in_geomdistance": names(contacts - mj_over)}})
        same = ref_over == mj_over and ref_bbox == mj_bbox and contacts <= mj_over
        row["result"] = "PASS" if same else "FAIL"
        ok_sets &= same
        doc["samples"][kname] = row

    if not args.no_render:
        free = _free_mb()
        if free is not None and free < 1000:
            doc["warnings"].append(f"render skipped: {free} MB free")
        else:
            doc["renders"] = render(m, VERIFY / f"m1_{scope}", key_ids["folded"])

    np_ = len(pf)
    s = doc["samples"]
    summary = (f"{scope}: {m.nbody} bodies, {doc['poses']['checked']} pose/AABB checks, "
               f"{np_} off; samples " + ", ".join(
                   f"{k} {r['result']} ({len(r['reference_overlaps'])}/{len(r['mjcf_overlaps'])} "
                   f"overlaps, {r['bbox_pairs']}/{r['mjcf_bbox_pairs']} bbox pairs)"
                   for k, r in s.items()))
    if np_:
        doc["errors"].append(f"{np_} pose/AABB check(s) out of tolerance")
    if not ok_sets:
        doc["errors"].append("collision lists differ (see samples)")
    print(f"check_mjcf_match: {summary}", file=sys.stderr)
    return finish(1 if doc["errors"] else 0, summary)


def _free_mb() -> int | None:
    try:
        for ln in Path("/proc/meminfo").read_text().splitlines():
            if ln.startswith("MemAvailable:"):
                return int(ln.split()[1]) // 1024
    except OSError:
        return None
    return None


if __name__ == "__main__":
    sys.exit(main())
