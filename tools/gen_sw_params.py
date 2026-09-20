"""Write cad_params.txt (SolidWorks global variables) from line_spec.yaml.

    tools/.venv/bin/python tools/gen_sw_params.py [--out PATH] [--spec PATH]

The file is linked from stack_skeleton.SLDPRT (Tools > Equations > Link to external file), one
line per variable in millimetres with 4 decimals:  "datum_dx" = 62.5000

A variable whose spec value is null (or absent) is skipped with a WARN naming the spec key, so
the report is the to-do list for the CAD side. Exits non-zero only on an unreadable/invalid
spec or an unwritable output. Writes reports/sw_params.json.

--dev merges the placeholder overlay; SolidWorks parameters are a deliverable, so a dev run
never writes the CAD folder: output is reports/cad_params.dev.txt and --out is refused.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from spec import (DEFAULT_SPEC, REPO, LineSpec, Merged, SpecError, add_dev_argument,  # noqa: E402
                  load_merged, parse_spec, report_path)

DEFAULT_OUT = Path("/mnt/c/Users/Admin/Documents/drone-line-cad/10_skeleton/cad_params.txt")
DEV_OUT = REPO / "reports" / "cad_params.dev.txt"
# Plate frame reference: the plate-stack check keeps datum XY identical across bottom/mid/top.
DATUM_PLATE = "bottom_plate"
LINE_RE = re.compile(r'^"(?P<name>[A-Za-z_][A-Za-z0-9_]*)"\s*=\s*(?P<value>-?\d+(?:\.\d+)?)\s*$')


@dataclass
class Var:
    name: str
    keys: list[str]  # spec paths the value is derived from (all must be known)
    value_m: float | None
    missing: list[str] = field(default_factory=list)  # the keys that are null/absent


def collect(spec: LineSpec) -> list[Var]:
    """The variables named in docs/11_cad_start_guide.md §2.1, in that order."""
    p = f"parts.{DATUM_PLATE}.datums"
    d = spec.parts[DATUM_PLATE].datums
    rnd = d.round if d and d.round else (None, None)
    dia = d.diamond if d and d.diamond else (None, None)
    cad = spec.cad
    mid, top = spec.parts.get("mid_plate"), spec.parts.get("top_plate")
    slot = (mid.lead_slot if mid and mid.lead_slot else (None, None))

    def var(name: str, *pairs: tuple[str, float | None], combine=lambda v: v[0]) -> Var:
        keys = [k for k, _ in pairs]
        missing = [k for k, v in pairs if v is None]
        return Var(name, keys, None if missing else combine([v for _, v in pairs]), missing)

    def diff(axis: int, name: str) -> Var:
        return var(name, (f"{p}.diamond[{axis}]", dia[axis]), (f"{p}.round[{axis}]", rnd[axis]),
                   combine=lambda v: v[0] - v[1])

    return [
        var("datum_d", ("cad.datum_d", cad.datum_d if cad else None)),
        diff(0, "datum_dx"),
        diff(1, "datum_dy"),
        var("press_nut_hole_d", ("cad.press_nut_hole_d", cad.press_nut_hole_d if cad else None)),
        var("lead_slot_w", ("parts.mid_plate.lead_slot[0]", slot[0])),
        var("lead_slot_l", ("parts.mid_plate.lead_slot[1]", slot[1])),
        var("z_mid", ("parts.mid_plate.z_in_stack", mid.z_in_stack if mid else None)),
        var("z_top", ("parts.top_plate.z_in_stack", top.z_in_stack if top else None)),
    ]


def format_mm(value_m: float) -> str:
    mm = round(value_m * 1000.0, 4)
    return f"{(mm if mm != 0 else 0.0):.4f}"  # no "-0.0000"


def render(variables: list[Var]) -> tuple[str, list[Var], list[Var]]:
    written = [v for v in variables if v.value_m is not None]
    skipped = [v for v in variables if v.value_m is None]
    text = "".join(f'"{v.name}" = {format_mm(v.value_m)}\n' for v in written)
    return text, written, skipped


def parse_params(text: str) -> dict[str, float]:
    """Inverse of render(): {name: value in mm}. Raises ValueError on a malformed line."""
    out: dict[str, float] = {}
    for n, line in enumerate(text.splitlines(), 1):
        if not line.strip():
            continue
        m = LINE_RE.match(line)
        if not m:
            raise ValueError(f"line {n}: not a global-variable line: {line!r}")
        if m["name"] in out:
            raise ValueError(f"line {n}: duplicate variable {m['name']}")
        out[m["name"]] = float(m["value"])
    return out


def write_report(path: Path, doc: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Write SolidWorks global variables from the spec")
    ap.add_argument("--spec", type=Path, default=DEFAULT_SPEC)
    ap.add_argument("--out", type=Path, default=None,
                    help=f"output file (default {DEFAULT_OUT}); not allowed with --dev")
    add_dev_argument(ap)
    args = ap.parse_args(argv)
    report = report_path("sw_params", args.dev)

    if args.dev and args.out is not None:
        print("gen_sw_params: --out is refused with --dev: placeholder values must not reach "
              f"the CAD folder (dev output goes to {DEV_OUT})", file=sys.stderr)
        return 2
    out = DEV_OUT if args.dev else (args.out or DEFAULT_OUT)

    doc: dict = {"passed": False, "dev": args.dev, "out": str(out), "written": {},
                 "skipped": [], "placeholders": [], "errors": []}
    try:
        merged: Merged = load_merged(args.spec, args.dev)
        variables = collect(parse_spec(merged.raw))
    except SpecError as exc:
        doc["errors"] = exc.errors
        write_report(report, doc)
        print(f"gen_sw_params: FAIL — invalid spec: {exc}", file=sys.stderr)
        return 1

    text, written, skipped = render(variables)
    overlay_paths = {p["path"] for p in merged.placeholders}
    for v in written:
        used = [k for k in v.keys if k in overlay_paths]
        doc["written"][v.name] = {"value_mm": float(format_mm(v.value_m)), "from": v.keys}
        doc["placeholders"] += used
    for v in skipped:
        doc["skipped"].append({"name": v.name, "keys": v.missing, "reason": "null in spec"})
        print(f"gen_sw_params: WARN skipped {v.name}: null or absent: {', '.join(v.missing)}",
              file=sys.stderr)

    try:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text)
    except OSError as exc:
        doc["errors"].append(f"cannot write {out}: {exc}")
        write_report(report, doc)
        print(f"gen_sw_params: FAIL — {doc['errors'][-1]}", file=sys.stderr)
        return 1

    doc["passed"] = True
    write_report(report, doc)
    if args.dev:
        print(f"gen_sw_params: DEV MODE — {len(doc['placeholders'])} placeholder value(s) "
              "written; not a deliverable")
    print(f"gen_sw_params: wrote {len(written)} variable(s), skipped {len(skipped)} -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
