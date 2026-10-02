"""Offscreen PNG renders of the layout solids (D-013), for a quick look without SolidWorks.

Called by tools/cad/layout.py with its built items; writes <out>/<stem>_top.png (parallel plan
view, +y up, labels on stations, SCARAs and sources) and <out>/<stem>_iso.png (perspective from
-x, -y, above). pyvista in tools/.venv, off screen: no display, no Gazebo. Line frame, mm.

Reference solids are drawn see-through so they cannot hide what they surround: the floor and the
aisle faint, guards at GUARD_OPACITY, reach/slide envelopes at ENVELOPE_OPACITY.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

TOL_MM = 1.0
ANGULAR = 0.4
ENVELOPE_OPACITY = 0.10
GUARD_OPACITY = 0.25
SIZE = (2000, 1200)


def _poly(verts, tris):
    import pyvista as pv
    pts = np.array([tuple(p) for p in verts], dtype=float)
    faces = np.hstack([np.full((len(tris), 1), 3), np.array(tris, dtype=np.int64)]).ravel()
    return pv.PolyData(pts, faces)


def _mesh(shape):
    """Tessellate the whole shape; a vendor STEP may hold a face OCCT cannot mesh, so fall back
    to face by face and leave out the ones that fail."""
    import pyvista as pv
    try:
        verts, tris = shape.tessellate(TOL_MM, ANGULAR)
        return _poly(verts, tris) if tris else None
    except AttributeError:
        pass
    parts = []
    for f in shape.faces():
        try:
            verts, tris = f.tessellate(TOL_MM, ANGULAR)
        except AttributeError:
            continue
        if tris:
            parts.append(_poly(verts, tris))
    return pv.merge(parts) if parts else None


def render(items, colour_of, out_dir: Path, stem: str) -> dict:
    """`items`: objects with .shape (build123d, mm), .category, .name, .label. Returns
    {"top": path, "iso": path} or {"error": message}."""
    try:
        import pyvista as pv
    except ImportError as exc:
        return {"error": f"pyvista not installed in tools/.venv ({exc})"}
    pv.OFF_SCREEN = True
    out_dir.mkdir(parents=True, exist_ok=True)
    meshes = []
    for it in items:
        m = _mesh(it.shape)
        if m is not None:
            meshes.append((it, m))
    # One label per thing: a SCARA's or carrier's leaves share their sub-assembly's name.
    labels, named = [], set()
    for it, m in meshes:
        if it.category not in ("station", "feeder", "buffer", "robot", "gang_head", "carrier"):
            continue
        name = getattr(it, "subgroup", None) or it.name
        if it.name.endswith("_Base") or name in named:
            continue
        named.add(name)
        labels.append((name, m))
    phys = [m for it, m in meshes if it.category not in ("floor", "aisle", "guard", "envelope")]
    lo = np.min([m.bounds[0::2] for m in phys], axis=0)
    hi = np.max([m.bounds[1::2] for m in phys], axis=0)
    pad = 0.15 * float(max(hi[0] - lo[0], hi[1] - lo[1]))
    fit = [lo[0] - pad, hi[0] + pad, lo[1] - pad, hi[1] + pad, lo[2], hi[2]]
    paths = {}
    for view in ("top", "iso"):
        p = pv.Plotter(off_screen=True, window_size=SIZE)
        p.set_background("white")
        for it, m in meshes:
            style, opacity = "surface", 1.0
            if it.category == "envelope":
                opacity = ENVELOPE_OPACITY
            elif it.category == "guard":
                style, opacity = ("wireframe", 0.6) if view == "top" else ("surface",
                                                                         GUARD_OPACITY)
            elif it.category in ("floor", "aisle"):
                opacity = 0.25
            p.add_mesh(m, color=colour_of(it.category), opacity=opacity, style=style,
                       show_edges=it.category in ("gang_head", "feeder", "buffer", "carrier"),
                       edge_color="#333333", line_width=1.0, smooth_shading=False)
        if labels:
            pts = np.array([(m.center[0], m.center[1], m.bounds[5] + 20.0) for _, m in labels])
            p.add_point_labels(pts, [n for n, _ in labels], font_size=14, point_size=1,
                               shape_opacity=0.6, always_visible=True)
        p.add_axes()
        if view == "top":
            p.view_xy()
            p.enable_parallel_projection()
            p.reset_camera(bounds=fit)
        else:
            c = [(fit[0] + fit[1]) / 2.0, (fit[2] + fit[3]) / 2.0, (fit[4] + fit[5]) / 2.0]
            d = 1.1 * max(fit[1] - fit[0], fit[3] - fit[2])
            p.camera_position = [(c[0] - 0.55 * d, c[1] - 1.0 * d, c[2] + 0.8 * d), c, (0, 0, 1)]
        path = out_dir / f"{stem}_{view}.png"
        p.screenshot(str(path))
        p.close()
        paths[view] = str(path)
    return paths
