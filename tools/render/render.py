"""Offscreen PNG renders of the layout solids (D-013), for a quick look without SolidWorks.

Called by tools/cad/layout.py with its built items; writes <out>/<stem>_top.png (parallel plan
view, +y up, labels on stations, SCARAs and sources) and <out>/<stem>_iso.png (perspective from
-x, -y, above). pyvista in tools/.venv, off screen: no display, no Gazebo. Line frame, mm.

Reference solids are drawn see-through so they cannot hide what they surround: the floor and the
aisle faint, guards at GUARD_OPACITY, reach/slide envelopes at ENVELOPE_OPACITY.

`render_panels()` (D-014) writes one PNG with the plan and the perspective side by side — the
per-station park close-ups, reports/render/<station>_park.png.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

TOL_MM = 1.0
ANGULAR = 0.4
ENVELOPE_OPACITY = 0.10
GUARD_OPACITY = 0.25
SIZE = (2000, 1200)
CLOSE_UP_ZOOM = 1.6      # the park close-ups' half-width panels


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


def _prepare(items, window=None):
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
    if window is not None:          # [x0, x1] mm: fit to that span of x only (a close-up)
        inside = [m for m in phys if m.bounds[0] >= window[0] and m.bounds[1] <= window[1]]
        lo = np.min([m.bounds[0::2] for m in inside], axis=0)
        hi = np.max([m.bounds[1::2] for m in inside], axis=0)
    pad = 0.15 * float(max(hi[0] - lo[0], hi[1] - lo[1]))
    fit = [lo[0] - pad, hi[0] + pad, lo[1] - pad, hi[1] + pad, lo[2], hi[2]]
    return meshes, labels, fit


def _draw(p, meshes, labels, colour_of, view: str, fit, font_size: int = 14,
          fit_iso: bool = False) -> None:
    for it, m in meshes:
        style, opacity = "surface", 1.0
        if it.category == "envelope":
            opacity = ENVELOPE_OPACITY
        elif it.category == "guard":
            style, opacity = ("wireframe", 0.6) if view == "top" else ("surface", GUARD_OPACITY)
        elif it.category in ("floor", "aisle"):
            opacity = 0.25
        p.add_mesh(m, color=colour_of(it.category), opacity=opacity, style=style,
                   show_edges=it.category in ("gang_head", "feeder", "buffer", "carrier",
                                              "slide_frame"),
                   edge_color="#333333", line_width=1.0, smooth_shading=False)
    if labels:
        pts = np.array([(m.center[0], m.center[1], m.bounds[5] + 20.0) for _, m in labels])
        p.add_point_labels(pts, [n for n, _ in labels], font_size=font_size, point_size=1,
                           shape_opacity=0.6, always_visible=True)
    p.add_axes()
    if view == "top":
        p.view_xy()
        p.enable_parallel_projection()
        p.reset_camera(bounds=fit)
        if fit_iso:
            p.camera.zoom(CLOSE_UP_ZOOM)
    elif fit_iso:                    # a close-up: look the same way, fitted to what is there
        p.view_vector((-0.55, -1.0, 0.8), viewup=(0, 0, 1))
        p.reset_camera(bounds=fit)
        p.camera.zoom(CLOSE_UP_ZOOM)
    else:
        c = [(fit[0] + fit[1]) / 2.0, (fit[2] + fit[3]) / 2.0, (fit[4] + fit[5]) / 2.0]
        d = 1.1 * max(fit[1] - fit[0], fit[3] - fit[2])
        p.camera_position = [(c[0] - 0.55 * d, c[1] - 1.0 * d, c[2] + 0.8 * d), c, (0, 0, 1)]


def render(items, colour_of, out_dir: Path, stem: str) -> dict:
    """`items`: objects with .shape (build123d, mm), .category, .name, .label. Returns
    {"top": path, "iso": path} or {"error": message}."""
    try:
        import pyvista as pv
    except ImportError as exc:
        return {"error": f"pyvista not installed in tools/.venv ({exc})"}
    pv.OFF_SCREEN = True
    out_dir.mkdir(parents=True, exist_ok=True)
    meshes, labels, fit = _prepare(items)
    paths = {}
    for view in ("top", "iso"):
        p = pv.Plotter(off_screen=True, window_size=SIZE)
        p.set_background("white")
        _draw(p, meshes, labels, colour_of, view, fit)
        path = out_dir / f"{stem}_{view}.png"
        p.screenshot(str(path))
        p.close()
        paths[view] = str(path)
    return paths


def render_panels(items, colour_of, path: Path, title: str, window=None) -> dict:
    """One PNG, plan (left) and perspective (right), titled, fitted to the solids that lie
    wholly inside `window` ([x0, x1] mm) when given. {"png": path} or {"error"}."""
    try:
        import pyvista as pv
    except ImportError as exc:
        return {"error": f"pyvista not installed in tools/.venv ({exc})"}
    pv.OFF_SCREEN = True
    path.parent.mkdir(parents=True, exist_ok=True)
    meshes, labels, fit = _prepare(items, window)
    p = pv.Plotter(off_screen=True, window_size=SIZE, shape=(1, 2))
    p.set_background("white")
    for col, view in enumerate(("top", "iso")):
        p.subplot(0, col)
        _draw(p, meshes, labels, colour_of, view, fit, font_size=11, fit_iso=True)
        p.add_text(f"{title} — {'plan' if view == 'top' else 'perspective'}", font_size=10,
                   color="black")
    p.screenshot(str(path))
    p.close()
    return {"png": str(path)}
