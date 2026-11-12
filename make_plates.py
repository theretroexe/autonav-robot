"""Lay every out/stl part flat, pack them per PRINT_PLATES.md, write out/autonav_plates.3mf
(Bambu Studio project, one plate per row of the plate list, no printer settings -> uses yours).
Run after robot.py:  python make_plates.py"""
import fnmatch, glob, math, os, uuid, zipfile
import numpy as np, trimesh

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
BED, MARGIN, GAP = 256.0, 6.0, 4.0
STRIDE, COLS = BED * 1.2, 3          # Bambu lays plates out on a grid, gap = 1/5 bed

PLATES = [  # (name, filament, part globs)
    ("0 PETG fit coupon - print first", "PETG", ["00_FIT_TEST_COUPON*"]),
    ("1 PLA bottom plate", "PLA", ["bottom_plate_*", "tie_rod_*", "axle_cap_*"]),
    ("2 PLA mid deck", "PLA", ["mid_deck_*", "knuckle_*"]),
    ("3 PLA brackets", "PLA", ["top_deck", "front_axle", "lidar_cradle", "rear_housing", "motor_mount", "cam_cradle"]),
    ("4 PLA wheels", "PLA", ["wheel_*"]),
    ("5 PETG gears 0.12mm layers", "PETG", ["diff_case_crown", "diff_cover", "side_gear_*", "spider_gear_*", "motor_pinion", "horn_adapter"]),
    ("6 PETG pins standoffs shafts", "PETG", ["standoff_*", "snap_pin_*", "kingpin_*", "pivot_pin_*", "spider_cross", "axle_clip_*", "half_shaft_*"]),
    ("7 TPU tyres", "TPU", ["tire_*"]),
]
UPRIGHT = ["half_shaft_*"]           # print standing on an end (round middle can't lie flat)


def rot_to_down(n):
    """Rotation matrix taking unit normal n to -Z."""
    return trimesh.geometry.align_vectors(n, [0, 0, -1])


def lay_flat(m, upright=False):
    """Rotate so the largest outermost flat face sits on the bed."""
    keys = np.round(m.face_normals, 3)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    area = np.bincount(inv.ravel(), m.area_faces)
    cand = uniq[np.argsort(area)[::-1][:40]]
    if upright:                                   # only the two ends of the long axis
        ax = np.eye(3)[np.argmax(m.extents)]
        cand = [ax, -ax]
    best, best_a = None, -1
    for n in cand:
        n = n / np.linalg.norm(n)
        d = m.vertices @ n
        on = (np.abs(m.triangles_center @ n - d.max()) < 0.05) & (m.face_normals @ n > 0.999)
        a = m.area_faces[on].sum()
        if a > best_a:
            best, best_a = n, a
    m = m.copy()
    m.apply_transform(rot_to_down(best))
    # spin about Z for the smallest footprint
    pts = m.vertices[:, :2]
    ang = min(np.radians(np.arange(0, 90, 3)),
              key=lambda t: np.prod(np.ptp(pts @ [[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]], axis=0)))
    m.apply_transform(trimesh.transformations.rotation_matrix(ang, [0, 0, 1]))
    m.apply_translation(-m.bounds[0])             # min corner at origin, on the bed
    return m, best_a


def pack(meshes):
    """Shelf pack (tallest-in-Y first) inside the bed; returns local xy offsets."""
    order = sorted(range(len(meshes)), key=lambda i: -meshes[i].extents[1])
    x = y = MARGIN
    row_h, pos = 0.0, {}
    for i in order:
        w, h = meshes[i].extents[:2]
        if x + w > BED - MARGIN:
            x, y, row_h = MARGIN, y + row_h + GAP, 0.0
        assert y + h <= BED - MARGIN, f"plate overflow at {i}"
        pos[i] = (x, y)
        x += w + GAP
        row_h = max(row_h, h)
    return [pos[i] for i in range(len(meshes))]


def main():
    files = {os.path.splitext(os.path.basename(f))[0]: f for f in glob.glob(os.path.join(OUT, "stl", "*.stl"))}
    used, objs, plates = set(), [], []
    for pi, (pname, fil, globs) in enumerate(PLATES):
        names = sorted(n for n in files if any(fnmatch.fnmatch(n, g) for g in globs))
        assert names and not used & set(names), pname
        used |= set(names)
        meshes, low = [], []
        for n in names:
            m, a = lay_flat(trimesh.load(files[n]), any(fnmatch.fnmatch(n, g) for g in UPRIGHT))
            meshes.append(m)
            if a < 15:
                low.append(f"{n} ({a:.0f} mm2)")
        ox, oy = (pi % COLS) * STRIDE, -(pi // COLS) * STRIDE
        ids = []
        for n, m, (x, y) in zip(names, meshes, pack(meshes)):
            objs.append((n, m, (ox + x, oy + y)))
            ids.append(len(objs))
        plates.append((pname, ids))
        print(f"plate {pname}: {len(names)} parts" + (f"  small contact: {', '.join(low)}" if low else ""))
    assert used == set(files), f"unassigned STLs: {sorted(set(files) - used)}"
    write_3mf(os.path.join(OUT, "autonav_plates.3mf"), objs, plates)


def write_3mf(path, objs, plates):
    res, build, cfg = [], [], ['<?xml version="1.0" encoding="UTF-8"?>\n<config>']
    for oid, (n, m, (x, y)) in enumerate(objs, 1):
        v = "".join(f'<vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in m.vertices)
        t = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in m.faces)
        res.append(f'<object id="{oid}" p:UUID="{uuid.uuid4()}" type="model"><mesh><vertices>{v}</vertices>'
                   f'<triangles>{t}</triangles></mesh></object>')
        build.append(f'<item objectid="{oid}" p:UUID="{uuid.uuid4()}" transform="1 0 0 0 1 0 0 0 1 {x:.4f} {y:.4f} 0" printable="1"/>')
        cfg.append(f'<object id="{oid}"><metadata key="name" value="{n}"/><metadata key="extruder" value="1"/>'
                   f'<part id="{oid}" subtype="normal_part"><metadata key="name" value="{n}"/>'
                   f'<metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/></part></object>')
    for pid, (pname, ids) in enumerate(plates, 1):
        cfg.append(f'<plate><metadata key="plater_id" value="{pid}"/><metadata key="plater_name" value="{pname}"/>'
                   '<metadata key="locked" value="false"/>')
        cfg += [f'<model_instance><metadata key="object_id" value="{i}"/><metadata key="instance_id" value="0"/>'
                f'<metadata key="identify_id" value="{100 + i}"/></model_instance>' for i in ids]
        cfg.append('</plate>')
    cfg.append('<assemble></assemble></config>')
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
             'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
             'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p">'
             '<metadata name="Application">BambuStudio-02.09.01.70</metadata>'
             '<metadata name="BambuStudio:3mfVersion">1</metadata>'
             f'<resources>{"".join(res)}</resources><build p:UUID="{uuid.uuid4()}">{"".join(build)}</build></model>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?>\n<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
                   '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
                   '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                   '<Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr("3D/3dmodel.model", model)
        z.writestr("Metadata/model_settings.config", "\n".join(cfg))
    print("wrote", path)


if __name__ == "__main__":
    main()
