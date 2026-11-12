"""Teammate / AI hand-off: out/handoff/autonav_robot.obj (plain-text mesh, mm, one named object per part)
and out/handoff/autonav_robot.md (frame, key dimensions, per-part bounding boxes).
Run after robot.py:  python make_handoff.py"""
import base64, glob, json, os, zipfile
import numpy as np, trimesh
import robot as R

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
DST = os.path.join(OUT, "handoff")
FRAME = "X right, Y forward, Z up, millimetres. Origin = floor directly under the rear-axle centre."


def parts():
    """[(name, group, 'printed'|'bought', world-space mesh)] from robot.glb (vertices are already mm, world pose)."""
    s = trimesh.load(os.path.join(OUT, "robot.glb"))
    out = []
    for node in s.graph.nodes_geometry:
        T, g = s.graph[node]
        grp, kind, name = node.split("__", 2)
        m = s.geometry[g].copy()
        m.apply_transform(T)
        out.append((name, grp, "printed" if kind == "P" else "bought", m))
    return sorted(out, key=lambda p: (p[2], p[1], p[0]))


def write_obj(path, ps):
    with open(path, "w") as f:
        f.write(f"# Autonav Ackermann robot v4.8, full assembly in driving pose\n# {FRAME}\n"
                "# One 'o' object per part, named <part>; 'g' = <subassembly>_<printed|bought>\n")
        base = 1
        for name, grp, kind, m in ps:
            f.write(f"o {name}\ng {grp}_{kind}\n")
            f.writelines(f"v {x:.3f} {y:.3f} {z:.3f}\n" for x, y, z in m.vertices)
            f.writelines(f"f {a + base} {b + base} {c + base}\n" for a, b, c in m.faces)
            base += len(m.vertices)


def fmt(v):
    return ", ".join(f"{a:.1f}" for a in v)


def write_md(path, ps, rep):
    env = rep["envelope_mm"]
    mass = rep["parts_printed_g"]
    lo = np.min([p[3].bounds[0] for p in ps], axis=0)
    hi = np.max([p[3].bounds[1] for p in ps], axis=0)
    key = [
        ("Overall size, wheels straight (as modelled) L x W x H",
         f"{hi[1] - lo[1]:.1f} x {hi[0] - lo[0]:.1f} x {hi[2] - lo[2]:.1f} mm; x {lo[0]:.1f}..{hi[0]:.1f}, y {lo[1]:.1f}..{hi[1]:.1f}, z {lo[2]:.1f}..{hi[2]:.1f}"),
        ("Envelope incl. full steering lock L x W x H", f"{env['L']} x {env['W']} x {env['H']} mm (rule: 300 mm cube)"),
        ("Mass printed / bought / total", f"{rep['printed_mass_g']} / {rep['bought_mass_g']} / {rep['total_mass_g']} g (rule: 1500 g)"),
        ("Wheelbase", f"{R.WB} mm (rear axle y=0, front axle y={R.WB})"),
        ("Track (wheel centre to centre)", f"rear {2 * R.REAR_X} mm, front {2 * R.FRONT_X} mm"),
        ("Wheel", f"Ø{2 * R.WHEEL_R} mm incl. {R.TIRE_T} mm TPU tyre, {R.WHEEL_W} mm wide; axle height z={R.AX_Z}"),
        ("Ground clearance (min)", f"{rep['ground_clearance_min']} mm"),
        ("Steering", f"Ackermann, MG996R; lock inner/outer {rep['lock_deg_inner_outer'][0]}° / {rep['lock_deg_inner_outer'][1]}°; "
                     f"min turn radius {rep['min_turn_radius_mm']} mm; kingpins at x=±{R.KP_X}, scrub {rep['scrub_radius_mm']} mm"),
        ("Steering servo output shaft", f"x={R.SERVO_SHAFT[0]:.1f}, y={R.SERVO_SHAFT[1]:.1f} (shaft points down)"),
        ("Drive", f"JGA25-370 encoder motor -> printed crown/bevel diff, ratio {rep['drive_ratio']}; RWD"),
        ("LIDAR (RPLIDAR C1)", f"centre x=0, y={R.LIDAR_Y}; base z={R.LIDAR_BASE_Z}; scan plane z={rep['lidar_scan_plane_z']} mm"),
        ("Camera pan servo (SG90) shaft", f"x={R.SG90_SHAFT[0]}, y={R.SG90_SHAFT[1]} on the mid deck"),
        ("Bottom plate", f"z {R.PL_Z0}–{R.PL_Z1} mm"),
        ("Mid deck", f"z {R.MID_Z0}–{R.MID_Z1} mm"),
        ("Rear top deck (half length)", f"z {R.TOP_Z0}–{R.TOP_Z1} mm"),
        ("Lower standoffs (plate -> mid deck), x,y", "; ".join(f"({x}, {y})" for x, y in R.STANDOFF_LOW)),
        ("Upper standoffs (mid -> top deck), x,y", "; ".join(f"({x}, {y})" for x, y in R.STANDOFF_HIGH)),
        ("IR line sensors (under plate), centre x,y", "; ".join(f"{n} ({x}, {y})" for n, ((x, y), _) in R.IR_MODULES.items())),
        ("Raspberry Pi 4B on mid deck (taped)", f"board corner ({R.PI_X0}, {R.PI_Y0}), 56 x 85 board, USB/Ethernet to the rear; "
                                                 "its hole centres (reference only, no screws) " + "; ".join(f"({x}, {y})" for x, y in R.PI_HOLES)),
        ("Fasteners", "none bought: printed snap pins/pegs; electronics glued or taped"),
    ]
    L = [f"# Autonav Ackermann robot v4.8: dimensions for teammates and AI tools\n",
         f"Model file: `autonav_robot.obj` (plain-text Wavefront OBJ, same folder). Every part is its own `o <name>` object, "
         f"already in its assembled position, so a bounding box of an object's `v` lines gives that part's position directly.\n",
         f"**Coordinate frame:** {FRAME}  \n**Units:** all numbers in mm, angles in degrees.\n",
         "Front of the robot = +Y (lidar end). Right = +X. Parts ending `_L` are on the left (−X), `_R` on the right (+X).\n",
         "## Key dimensions\n", "| What | Value |", "|---|---|"]
    L += [f"| {k} | {v} |" for k, v in key]
    for kind in ("printed", "bought"):
        rows = [p for p in ps if p[2] == kind]
        L += [f"\n## {kind.capitalize()} parts ({len(rows)})\n",
              "Bounding box in the assembly frame (min corner, max corner) and size along X, Y, Z."
              + (" Bought parts are simplified stand-ins (outer envelope only)." if kind == "bought" else "") + "\n",
              "| Part | Subassembly | Min x, y, z | Max x, y, z | Size X x Y x Z | " + ("Mass g |" if kind == "printed" else ""),
              "|---|---|---|---|---|" + ("---|" if kind == "printed" else "")]
        for name, grp, _, m in rows:
            lo, hi = m.bounds
            L.append(f"| {name} | {grp} | {fmt(lo)} | {fmt(hi)} | {' x '.join(f'{a:.1f}' for a in hi - lo)} |"
                     + (f" {mass.get(name, '')} |" if kind == "printed" else ""))
    L += ["\n## Notes\n",
          "- Source of truth is `robot.py` (parametric). Regenerate: `python robot.py && python make_handoff.py`.",
          "- Per-part print files: `out/stl/<part>.stl` (assembly pose; lay flat in the slicer) or `out/autonav_plates.3mf`.",
          f"- Printed clearances: FIT {R.FIT} mm radial on pins/bushings; tune after printing `00_FIT_TEST_COUPON_print_first.stl`.",
          "- Not modelled: wiring, switches, glue/tape. Bought-part shapes are approximate envelopes; check real datasheets before cutting anything."]
    open(path, "w", encoding="utf-8").write("\n".join(L) + "\n")


def main():
    os.makedirs(DST, exist_ok=True)
    ps = parts()
    rep = json.load(open(os.path.join(OUT, "report.json")))
    write_obj(os.path.join(DST, "autonav_robot.obj"), ps)
    write_md(os.path.join(DST, "autonav_robot.md"), ps, rep)
    # self-check: the OBJ round-trips to the report envelope (report W also covers full steering lock, so only <=)
    back = trimesh.load(os.path.join(DST, "autonav_robot.obj"), force="mesh")
    ext = back.bounds[1] - back.bounds[0]
    env = rep["envelope_mm"]
    assert abs(ext[1] - env["L"]) < 0.5 and abs(ext[2] - env["H"]) < 0.5 and ext[0] <= env["W"] + 0.5, ext
    print(f"wrote {DST}: {len(ps)} parts, envelope {ext.round(1)}")
    # one zip for the shared viewer's download button (artifact downloads allow zip/md, not 3mf/obj)
    here = os.path.dirname(os.path.abspath(__file__))
    with zipfile.ZipFile(os.path.join(DST, "autonav_robot_files.zip"), "w", zipfile.ZIP_DEFLATED) as z:
        for src, arc in [(os.path.join(OUT, "autonav_plates.3mf"), "autonav_plates_BambuStudio.3mf"),
                         (os.path.join(here, "PRINT_PLATES.md"), "PRINT_PLATES.md"),
                         (os.path.join(DST, "autonav_robot.obj"), "autonav_robot.obj"),
                         (os.path.join(DST, "autonav_robot.md"), "autonav_robot.md")]:
            z.write(src, arc)
        for f in sorted(glob.glob(os.path.join(OUT, "stl", "*.stl"))):
            z.write(f, "stl/" + os.path.basename(f))
    # artifacts serve text but not .zip: the viewer fetches this, decodes it and offers it via the downloads capability
    with open(os.path.join(DST, "autonav_robot_files.zip"), "rb") as f, open(os.path.join(DST, "autonav_robot_files.zip.b64.txt"), "w") as o:
        o.write(base64.b64encode(f.read()).decode())


if __name__ == "__main__":
    main()
