r"""Round and diamond locating pins for the main-line pallet (plan P4 B1).

Every plate in the stack is located through two dedicated Ø`cad.datum_d` tooling holes on the
longest diagonal (summary §7.2): the **round** pin takes position, the **diamond** (relieved) pin
takes rotation only, so the tolerance between the two holes can never jam the stack. The relief
flats therefore face *along* the line joining the two pins — the direction the diamond pin must
stay free in — and the crowns that are left bear across it.

    round pin              diamond pin (looking down the pin axis)
       ___                        _
      /   \                     /   \      crowns bear across the datum line
     |  +  |                   ( -+- )     flats face along it (angle from the round pin)
      \___/                      \_/

This module is geometry only: every dimension arrives as an argument in millimetres (build123d's
unit) and nothing here reads the spec. `tools/cad/pallet.py` resolves the spec values, builds the
pins into the pallet and is the entry point.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from build123d import Align, Axis, Box, Cylinder, GeomType, Part, Rot, chamfer

BOTTOM = (Align.CENTER, Align.CENTER, Align.MAX)   # solid hangs below z = 0
TOP = (Align.CENTER, Align.CENTER, Align.MIN)      # solid stands above z = 0

# Crown width / locating diameter. A third of the diameter is the usual diamond-pin proportion:
# enough bearing area to take the rotation, narrow enough that the pin cannot bind along the
# datum line.
CROWN_RATIO = 1.0 / 3.0


@dataclass(frozen=True)
class PinDims:
    """One pin, in millimetres, with z = 0 at the pallet rest plane (the plate's underside).

    The shank is pressed into the pallet body (below z = 0); the locating portion is what the
    plates slide over (above z = 0).
    """

    locating_d: float      # fits the reamed Ø datum_d hole with a locational clearance
    free_length: float     # above the rest plane: full plate stack + lead-in
    shank_d: float         # pressed into the pallet bore
    press_depth: float     # shank length, below the rest plane
    tip_chamfer: float     # 45° lead-in at the tip
    crown_w: float         # diamond pin only: width left between the relief flats

    @property
    def slenderness(self) -> float:
        """Free length / locating diameter — how easily the pin bends when a plate lands on it."""
        return self.free_length / self.locating_d


def dims(*, hole_d: float, stack_h: float, fit_clearance: float, lead_in: float,
         shank_step: float, press_depth: float, tip_chamfer: float) -> PinDims:
    """Derive both pins' dimensions from the hole they locate in and the stack they clear."""
    locating_d = hole_d - fit_clearance
    return PinDims(locating_d=locating_d,
                   free_length=stack_h + lead_in,
                   shank_d=locating_d + 2.0 * shank_step,
                   press_depth=press_depth,
                   tip_chamfer=tip_chamfer,
                   crown_w=CROWN_RATIO * locating_d)


def relief_angle_deg(round_xy: tuple[float, float], diamond_xy: tuple[float, float]) -> float:
    """Direction of the datum line, round pin -> diamond pin. The diamond pin's flats face it."""
    dx, dy = diamond_xy[0] - round_xy[0], diamond_xy[1] - round_xy[1]
    if math.isclose(math.hypot(dx, dy), 0.0, abs_tol=1e-9):
        raise ValueError("round and diamond datums are at the same point: no datum line")
    return math.degrees(math.atan2(dy, dx))


def _shank(d: PinDims) -> Part:
    return Cylinder(d.shank_d / 2.0, d.press_depth, align=BOTTOM)


def _locating(d: PinDims) -> Part:
    """The full-diameter locating portion, chamfered at the tip so a plate cannot catch on it."""
    body = Cylinder(d.locating_d / 2.0, d.free_length, align=TOP)
    tip = body.edges().filter_by(GeomType.CIRCLE).group_by(Axis.Z)[-1]
    return chamfer(tip, length=d.tip_chamfer)


def round_pin(d: PinDims) -> Part:
    """Takes position: full diameter, so it constrains X and Y."""
    pin = _shank(d) + _locating(d)
    pin.label = "Pin_Round"
    return pin


def diamond_pin(d: PinDims, relief_deg: float) -> Part:
    """Takes rotation only: relieved to `crown_w` across the datum line at `relief_deg`."""
    big = 2.0 * max(d.locating_d, d.free_length)
    slab = Rot(Z=relief_deg) * Box(d.crown_w, big, big)     # normal = the datum line
    pin = _shank(d) + (_locating(d) & slab)
    pin.label = "Pin_Diamond"
    return pin
