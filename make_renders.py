"""Studio renders of out/robot.glb -> media/*.webp (transparent background, soft shadow), for the README and site.
Run after robot.py:
  "C:/Program Files/Blender Foundation/Blender 5.1/blender.exe" -b --factory-startup -P make_renders.py
Optional: append `-- name1 name2` to render only some views."""
import math, os, sys
import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
GLB, MEDIA = os.path.join(HERE, "out", "robot.glb"), os.path.join(HERE, "media")
EXPLODE = {"chassis": 0.0, "mid": 0.07, "cam": 0.07, "top": 0.13}      # layer lift (m) for the exploded view
DIFF = {"diff_cover": -0.05, "side_gear_L": -0.026, "spider": 0.0, "side_gear_R": 0.026, "diff_case_crown": 0.05}

# side view is orthographic with a fixed mapping so the site can draw dimension lines on it:
# image 1600 x 1000 px = 0.36 m x 0.225 m, centred on y 0.11375, z 0.095 (see index.html)
SIDE = dict(scale=0.36, cy=0.11375, cz=0.095)

VIEWS = {   # name: (camera position, look-at, lens mm or None for ortho, resolution, mode)
    "hero":    ((0.50, 0.66, 0.30), (0.0, 0.115, 0.075), 60, (1800, 1100), None),
    "rear34":  ((-0.48, -0.42, 0.26), (0.0, 0.10, 0.07), 60, (1600, 1000), None),
    "side":    ((1.0, SIDE["cy"], SIDE["cz"]), (0.0, SIDE["cy"], SIDE["cz"]), None, (1600, 1000), None),
    "exploded": ((0.55, 0.60, 0.42), (0.0, 0.11, 0.13), 55, (1600, 1200), "explode"),
    "diff":    ((-0.12, -0.28, 0.14), (0.0, 0.0, 0.034), 62, (1600, 1000), "diff"),
}


def setup():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=GLB)
    root = bpy.data.objects.new("robot", None)          # glTF import gives (x, -z, y) in mm: rotate back, mm -> m
    bpy.context.scene.collection.objects.link(root)
    root.rotation_euler, root.scale = (math.radians(-90), 0, 0), (0.001,) * 3
    parts = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    for o in parts:
        o.parent = root
        name = o.name.split("__")[-1]
        for slot in o.material_slots:
            m = slot.material
            bsdf = m and m.node_tree and m.node_tree.nodes.get("Principled BSDF")
            if not bsdf:
                continue
            bsdf.inputs["Roughness"].default_value = 0.85 if "tire" in name else 0.42
            if "JGA25" in name:
                bsdf.inputs["Metallic"].default_value, bsdf.inputs["Roughness"].default_value = 0.8, 0.3
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    try:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        for dev in ("OPTIX", "CUDA", "HIP", "ONEAPI"):
            try:
                prefs.compute_device_type = dev
                prefs.get_devices()
                if any(d.type == dev for d in prefs.devices):
                    for d in prefs.devices:
                        d.use = True
                    sc.cycles.device = "GPU"
                    print("cycles device", dev)
                    break
            except TypeError:
                continue
    except Exception as e:
        print("GPU setup skipped:", e)
    sc.cycles.samples, sc.cycles.use_denoising = 128, True
    sc.render.film_transparent = True
    sc.render.image_settings.file_format, sc.render.image_settings.color_mode = "WEBP", "RGBA"
    sc.render.image_settings.quality = 90
    try:
        sc.view_settings.view_transform, sc.view_settings.look = "AgX", "AgX - Punchy"
    except TypeError:
        pass
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.8, 0.83, 0.87, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.12
    sc.world = world
    bpy.ops.mesh.primitive_plane_add(size=6, location=(0, 0.1, 0))
    bpy.context.object.is_shadow_catcher = True
    for name, loc, power, size in (("key", (-0.7, 0.55, 1.0), 32, 1.2), ("fill", (0.9, 0.3, 0.45), 8, 1.0),
                                   ("rim", (0.1, -0.9, 0.6), 22, 0.8)):
        L = bpy.data.lights.new(name, "AREA")
        L.energy, L.size = power, size
        ob = bpy.data.objects.new(name, L)
        sc.collection.objects.link(ob)
        ob.location = loc
        ob.rotation_euler = (Vector((0, 0.11, 0.08)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    sc.collection.objects.link(cam)
    sc.camera = cam
    return parts, cam


def pose(parts, mode):
    """Child offsets are in the parent's frame: mm, local (x, -z, y) of the robot frame."""
    bpy.data.objects["Plane"].hide_render = mode == "diff"      # floating diff parts: no floor shadows
    for o in parts:
        grp, name = o.name.split("__")[0], o.name.split("__")[-1]
        o.location = (0, 0, 0)
        o.hide_render = False
        if mode == "explode":
            o.location = (0, -EXPLODE.get(grp, 0) * 1000, 0)
        elif mode == "diff":
            hit = next((k for k in DIFF if name.startswith(k)), None)
            o.hide_render = hit is None
            o.location = (DIFF.get(hit, 0) * 1000, 0, 0)


def shoot(parts, cam, name):
    pos, look, lens, (w, h), mode = VIEWS[name]
    pose(parts, mode)
    sc = bpy.context.scene
    sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage = w, h, 100
    cam.location = pos
    cam.rotation_euler = (Vector(look) - Vector(pos)).to_track_quat("-Z", "Y").to_euler()
    if lens:
        cam.data.type, cam.data.lens = "PERSP", lens
    else:
        cam.data.type, cam.data.ortho_scale = "ORTHO", SIDE["scale"]
    cam.data.clip_start, cam.data.clip_end = 0.01, 10
    sc.render.filepath = os.path.join(MEDIA, f"{name}.webp")
    bpy.ops.render.render(write_still=True)
    print("rendered", sc.render.filepath)


if __name__ == "__main__":
    os.makedirs(MEDIA, exist_ok=True)
    want = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else list(VIEWS)
    parts, cam = setup()
    for v in want:
        shoot(parts, cam, v)
