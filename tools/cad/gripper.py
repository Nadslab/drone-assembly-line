r"""The pick feature the gantry's carrier tool grips, and the space it must be left (plan P4 B3).

Every returnable fixture the gantry lifts — spider carrier, pod tray — presents the same feature:
a cylindrical **pick boss** standing proud of the fixture's top structure, which the tool's jaws
close on. Two things have to be true, and both are decided by `gantry.tools.<tool>.dims`, not by
the fixture:

    jaws close across the boss          jaws reach down the boss, not past it
      |<------- jaw ------->|             ___
      |  ___             ___ |           |   |  boss_h
      | |   | <- boss -> |   |           |___|  <- reach ends above the structure
      |_|   |___________|   |_|          =====  structure top (boss base)

  * the jaw opening must clear the boss diameter: `boss_d + 2*clearance <= min(jaw_x, jaw_y)`;
  * the boss must be taller than the jaws reach down: `reach + clearance <= boss_h`, or the jaw
    tips foul whatever the boss stands on.

`keep_out` is the volume the jaws sweep — the jaw footprint over the last `reach` of the boss,
minus the boss itself. Any other material there is an interference, so a generator intersects it
with the rest of the fixture and fails on a non-zero volume.

Geometry only: every dimension arrives in millimetres and nothing here reads the spec. The boss
stands on z = 0, so a generator places it with `Pos(x, y, base_z) * boss(dims)`.
"""

from __future__ import annotations

from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Part, Pos

TOP = (Align.CENTER, Align.CENTER, Align.MIN)   # solid stands above z = 0


@dataclass(frozen=True)
class GripDims:
    """One pick boss and the tool that grips it, in millimetres."""

    boss_d: float       # fabrication choice, local to the fixture
    boss_h: float       # fabrication choice: how far the boss stands above the structure
    jaw_x: float        # gantry.tools.<tool>.dims[0]
    jaw_y: float        # gantry.tools.<tool>.dims[1]
    reach: float        # gantry.tools.<tool>.dims[2]: how far the jaws come down
    clearance: float    # gap the jaws need, per side and at the tips


def problems(d: GripDims, tool: str) -> list[str]:
    """Why this tool cannot grip this boss — empty when it can."""
    out = []
    jaw = min(d.jaw_x, d.jaw_y)
    if d.boss_d + 2.0 * d.clearance > jaw:
        out.append(f"gantry.tools.{tool}.dims: jaws open {jaw:.1f} mm but the pick boss is "
                   f"Ø{d.boss_d:.1f} mm and needs {d.clearance:g} mm per side")
    if d.reach + d.clearance > d.boss_h:
        out.append(f"gantry.tools.{tool}.dims[2]: jaws reach {d.reach:.1f} mm down but the pick "
                   f"boss is only {d.boss_h:.1f} mm tall, so the jaw tips would foul the "
                   f"structure it stands on")
    return out


def boss(d: GripDims) -> Part:
    """The boss itself, standing on z = 0."""
    part = Cylinder(d.boss_d / 2.0, d.boss_h, align=TOP)
    part.label = "PickBoss"
    return part


def keep_out(d: GripDims) -> Part:
    """The volume the jaws sweep around the boss, in the same frame: nothing else may be here."""
    swept = Box(d.jaw_x, d.jaw_y, d.reach, align=(Align.CENTER, Align.CENTER, Align.MAX))
    part = Pos(0, 0, d.boss_h) * swept          # hangs from the top of the boss
    part -= Cylinder(d.boss_d / 2.0, d.boss_h, align=TOP)
    part.label = "GripKeepOut"
    return part
