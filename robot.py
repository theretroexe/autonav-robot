r"""Autonomous Navigation Challenge robot - parametric assembly (manifold3d), v2.

Ackermann front steering (MG996R), rear-wheel drive from one JGA25-370 encoder motor through a
printed crown-gear + bevel differential, RPLIDAR C1 hung low in the nose, three stacked decks on
solid printed standoffs, Logitech Brio 100 on an SG90 pan servo.

v4: NO screws at all (nothing to buy): printed 4 mm snap pins through brackets + plate, snap-peg standoffs,
snap-ring motor cradle, snap hooks on both servos, printed splined servo horns; glue/tape for electronics.
v3: two decks only (camera on the mid deck), IR line modules under the plate front + rear,
switches left off the model (hot-glued by the team).
v2: kingpins inside the wheel (scrub 9 mm), light spoked 20 mm wheels, lattice decks, hollowed
brackets, and ONE bought fastener size: M3x8 into M3 heat-set inserts. Every other joint is a
printed snap pin, snap clip, wheel clip or trapped part.

Frame: X right, Y forward, Z up, mm. Origin = floor under the rear-axle centre.

Run:  python robot.py      -> out/stl/<part>.stl (printed parts, assembly pose),
                              out/robot.glb (everything, coloured), out/report.json + checks
"""
import functools
import json
import math
import os
import numpy as np
import manifold3d as m3d
import trimesh

M, CS = m3d.Manifold, m3d.CrossSection
RND = m3d.JoinType.Round
SEG = 48
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

# ---- tweakables (calibration knobs) --------------------------------------------------------------
FIT = 0.25            # radial clearance on printed bushings and pins (print the fit-test coupon first)
M3_CLR = 3.4          # grid holes on the mid deck (zip ties, wiring); no screws are used anywhere
PIN_R, PIN_HEAD_R = 2.0, 3.5   # printed 4 mm snap pins replace every screw
PIN_HOLE = PIN_R + FIT
SPLINE_FIT = 0.05     # printed servo-horn splines: + if the horn won't press on, - if it slips
BACKLASH = 0.15       # printed gear backlash (mm on pitch circle)
CROWN_BACKLASH = 0.15 # extra crown-gap width (the pinion is also thinned by BACKLASH): 0.3 mm total
JOIN_FIT = 0.10       # per-side gap on the glued dovetails of the split decks
BED = 250.0           # Bambu P2S bed is 256 mm; keep 3 mm margin
SNAP = 0.6            # snap-lip overhang on clips and pins
LIP_L = 4.0           # length of the cone on every snap lip (longer = gentler push; the hold comes from the shoulder)
LIP_TIP = 0.45        # cone tip radius as a fraction of the pin radius (pointed, so it finds the hole by itself)
HORN_PEG_R = 7.0      # radius of the 2 pegs that key the horn adapter into the MG996R round horn holes

# ---- layout -------------------------------------------------------------------------------------
WHEEL_R, TIRE_T, WHEEL_W = 34.0, 3.0, 20.0
RIM_R = WHEEL_R - TIRE_T                 # 30.5
HUB_L = 12.0                             # hub sits at the OUTER face, leaving the wheel hollow inside
AX_Z = WHEEL_R
WB = 150.0                               # wheelbase
REAR_X, FRONT_X = 70.0, 72.0             # wheel centre |x|
KP_X = 63.0                              # kingpin |x|: 1 mm inside the front wheel's inner face -> scrub 9 mm
PL_Z0, PL_Z1 = 17.5, 21.5                # bottom plate
MID_Z0, MID_Z1 = 86.0, 89.0              # mid deck
TOP_Z0, TOP_Z1 = 130.0, 133.0            # half-length rear deck
LIDAR_Y, LIDAR_BASE_Z = 223.0, 12.0      # as far forward as the 30 cm cube allows (4.5 mm margin)
LIDAR_SCAN = LIDAR_BASE_Z + 29.8
# drivetrain
DIFF_M, DIFF_Z = 1.25, 12                # spider/side miter gears (zv = 17: no undercut)
CROWN_M, CROWN_Z, PIN_Z = 1.25, 36, 11
CROWN_ROOT_X = 15.0
CROWN_PITCH_X = CROWN_ROOT_X + 1.25 * CROWN_M
MOTOR_X = CROWN_PITCH_X + PIN_Z * CROWN_M / 2      # pinion/motor axis x
MOTOR_FACE_Y = 31.0                                 # gearbox face
CROWN_R_IN, CROWN_R_OUT = 20.0, 25.5                # 5.5 mm working face (inside r20 the generated teeth undercut to slivers)
DIFF_SPLIT_X = 0.0                                  # cover | crown-half split plane, through the spider centres
                                                    # (at -5 the spiders couldn't pass the cavity lip on assembly)
# steering (tuned numerically by tune_steering(), see report "ackermann")
ARM_LEN = 20.0
ARM_AIM_Y = -150.0
HORN_LEN = 16.0
HORN_PIV = 5.0                                      # horn pivots at x = +-HORN_PIV
HORN_MAX = 32.0                                     # servo travel used each way
TIE_Z = (27.6, 30.4)
ARM_Z = (24.6, 27.4)
LOWER_ARM_Z = (15.0, 24.4)
BRIDGE_Z = (46.9, 50.9)
KP_R = 2.5                                          # printed kingpin radius
PIV_R = 2.0                                         # printed pivot pin radius


def arm_end():
    u = np.array([-KP_X, ARM_AIM_Y - WB]) / math.hypot(KP_X, ARM_AIM_Y - WB)
    return np.array([KP_X, WB]) + ARM_LEN * u


ARM_END = arm_end()                                 # right-side arm pivot
SERVO_SHAFT = np.array([0.0, ARM_END[1] + HORN_LEN])  # MG996R output, shaft pointing down
STANDOFF_LOW = [(-44, -22), (44, -22), (-46, 75), (46, 75), (-40, 172), (40, 172)]
STANDOFF_HIGH = [(-46, -8), (46, -8), (-46, 60), (46, 60)]   # mid deck -> rear top deck
SG90_SHAFT = (0.0, 212.0)                           # camera pan servo, front of the mid deck
# IR line modules (32 x 14 PCB, sensor at one end facing down), glued/screwed UNDER the bottom plate
IR_W, IR_L = 14.0, 32.0
IR_MODULES = {"IR_front": ((40.0, 237.0), 1), "IR_rear": ((32.0, -14.0), -1)}   # centre, sensor end (+y/-y)
PI_X0, PI_Y0 = -28.0, 35.0                         # Pi 4B corner (56 x 85, USB/Eth to the rear)
PI_HOLES = [(PI_X0 + 3.5 + dx, PI_Y0 + 3.5 + dy) for dx in (0, 49) for dy in (0, 58)]
ESP_X0, ESP_Y0 = -35.0, 6.5                         # ESP32-S3 DevKitC lies across the rear, on its headers


# ---- primitives ---------------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return M.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(r, z0, z1, x=0.0, y=0.0, seg=SEG):
    return M.cylinder(z1 - z0, r, r, seg).translate([x, y, z0])


def cylx(r, x0, x1, y=0.0, z=0.0, seg=SEG):
    return M.cylinder(x1 - x0, r, r, seg).rotate([0, 90, 0]).translate([x0, y, z])


def cyly(r, y0, y1, x=0.0, z=0.0, seg=SEG):
    return M.cylinder(y1 - y0, r, r, seg).rotate([-90, 0, 0]).translate([x, y0, z])


def hexz(af, z0, z1, x=0.0, y=0.0):
    return M.cylinder(z1 - z0, af / math.sqrt(3), af / math.sqrt(3), 6).translate([x, y, z0])


def outline_cs(right, r):
    """Right-half outline listed front->back (x>0), mirrored about x=0, corners rounded by r."""
    pts = (list(right) + [(-x, y) for x, y in reversed(right)])[::-1]
    return CS([np.asarray(pts, float)]).offset(-r, RND).offset(r, RND, circular_segments=24)


def union(parts):
    return M.batch_boolean([p for p in parts if p is not None], m3d.OpType.Add)


def cs_union(parts):
    return CS.batch_boolean(parts, m3d.OpType.Add) if parts else CS()


def rsq(x, y, w, r=3.0):
    return CS.square([w, w]).translate([x - w / 2, y - w / 2]).offset(-r, RND).offset(r, RND, circular_segments=12)


def lattice(outline, keep, margin=4.5, pitch=20.0, win=15.5, node_holes=False):
    """Window pattern (2D) for a plate: rounded windows on a grid, skipping anything near `keep`.
    Tries 4 grid offsets and keeps the one that removes the most material.
    node_holes adds an M3 clearance hole at every grid node that sits on solid bar."""
    inset = outline.offset(-margin, RND)
    (bx0, by0), (bx1, by1) = np.array(outline.bounds()).reshape(2, 2)
    best = CS()
    for ox, oy in ((0, 0), (pitch / 2, 0), (0, pitch / 2), (pitch / 2, pitch / 2)):
        cuts = []
        for x in np.arange(ox + math.floor((bx0 - ox) / pitch) * pitch, bx1 + pitch, pitch):
            for y in np.arange(oy + math.floor((by0 - oy) / pitch) * pitch, by1 + pitch, pitch):
                w = rsq(x + pitch / 2, y + pitch / 2, win)
                if (w - inset).area() < 0.01 and (w ^ keep).area() < 0.01:
                    cuts.append(w)
                if node_holes:
                    h = CS.circle(M3_CLR / 2, 16).translate([x, y])
                    if (h - inset).area() < 0.01 and (h ^ keep).area() < 0.01:
                        cuts.append(h)
        c = cs_union(cuts)
        if c.area() > best.area():
            best = c
    return best


def keep_circles(pts, r):
    return [CS.circle(r, 24).translate([x, y]) for x, y in pts]


def keep_box(x0, x1, y0, y1):
    return CS.square([x1 - x0, y1 - y0]).translate([x0, y0])


# ---- printed fasteners --------------------------------------------------------------------------
def snap_lip(x, y, z_shoulder, dirn, r, snap=SNAP, length=LIP_L, seg=24):
    """Pointed cone snap lip: full r+snap at the flat shoulder (catches the far face), tapering to a near
    point so it drops into the hole and the slotted prongs squeeze closed gradually as it's pushed in.
    dirn = -1 when the tip points down, +1 when it points up."""
    r_tip = r * LIP_TIP
    if dirn < 0:
        return M.cylinder(length, r_tip, r + snap, seg).translate([x, y, z_shoulder - length])
    return M.cylinder(length, r + snap, r_tip, seg).translate([x, y, z_shoulder])


def snap_pin(x, y, z_head, z_tip, r=PIN_R, head_r=PIN_HEAD_R, down=True):
    """Printed snap pin. down=True: head sits on the surface at z_head, slotted lip snaps under z_tip.
    down=False: head under z_head, lip above z_tip. One flat side so it prints lying down (prongs flex
    parallel to the bed). Print in PETG."""
    lo, hi = sorted((z_head, z_tip))
    shaft = cyl(r, lo, hi, x, y, 24)
    sl = min(7.0, hi - lo - 1.5)
    if down:
        head = cyl(head_r, z_head, z_head + 2, x, y, 24)
        lip = snap_lip(x, y, z_tip, -1, r)
        slot = box(x - 0.6, x + 0.6, y - r - 2, y + r + 2, z_tip - LIP_L - 0.1, z_tip + sl)
        zz = (z_tip - LIP_L - 0.5, z_head + 3)
    else:
        head = cyl(head_r, z_head - 2, z_head, x, y, 24)
        lip = snap_lip(x, y, z_tip, 1, r)
        slot = box(x - 0.6, x + 0.6, y - r - 2, y + r + 2, z_tip - sl, z_tip + LIP_L + 0.1)
        zz = (z_head - 3, z_tip + LIP_L + 0.5)
    flat = box(x - head_r - 1, x + head_r + 1, y - head_r - 1, y - (r - 0.4), *zz)
    return shaft + head + lip - slot - flat


def axle_clip():
    """C-clip for the 8 mm wheel axles (rear half-shafts and front stubs), lies in the YZ plane at x=0."""
    c = cyl(6.5, 0, 1.2, seg=40) - cyl(3.3, -1, 2.2, seg=40) - box(-2.5, 2.5, 1.5, 8, -1, 2.2)
    return c.rotate([0, 90, 0])


# ---- gears --------------------------------------------------------------------------------------
def gear2d(m, z, pa=20.0, backlash=BACKLASH, n=7, add=1.0):
    pa = math.radians(pa)
    rp, rb, ra, rf = m * z / 2, m * z / 2 * math.cos(pa), m * z / 2 + add * m, m * z / 2 - 1.25 * m
    inv = lambda a: math.tan(a) - a
    half = math.pi / (2 * z) - backlash / (2 * rp)
    th = lambda r: half + inv(pa) - inv(math.acos(min(1.0, rb / r)))
    rs = np.linspace(max(rb, rf), ra, n)
    pts = []
    for i in range(z):
        c = 2 * math.pi * i / z
        flank = [(r, c - th(r)) for r in rs] + [(r, c + th(r)) for r in rs[::-1]]
        if rf < rb:
            flank = [(rf, c - th(rb))] + flank + [(rf, c + th(rb))]
        pts += flank
    return CS([np.array([(r * math.cos(a), r * math.sin(a)) for r, a in pts])])


def spur(m, z, z0, z1):
    return gear2d(m, z).extrude(z1 - z0).translate([0, 0, z0])


def bevel(m, z, face, hub_r, hub_len, bore_hex=None, bore_r=None):
    """Straight miter gear, heel at z=0, teeth taper towards the apex at z=+rp. Hub on -z side."""
    rp = m * z / 2
    s = (rp - face) / rp
    g = gear2d(m, z).extrude(face, 4, 0, (s, s)) + cyl(hub_r, -hub_len, 0.01)
    if bore_hex:
        g -= hexz(bore_hex + 2 * FIT, -hub_len - 1, face + 1)
    if bore_r:
        g -= cyl(bore_r, -hub_len - 1, face + 1)
    return g


@functools.lru_cache(maxsize=None)
def crown_teeth(m, z, r_in, r_out, x_root):
    """Face (crown) teeth on the +x face, GENERATED: a slightly fat copy of the pinion is rolled through the
    blank exactly as the real pinion will roll, so the teeth are conjugate across the whole face width
    (inner undercut and outer tip thinning come out automatically). Crown centred on the x axis at z=0."""
    zp, rpp = PIN_Z, m * PIN_Z / 2
    blank = cylx(r_out, x_root - 0.01, x_root + 2.25 * m, seg=4 * z) - cylx(r_in, x_root - 1, x_root + 3 * m, seg=4 * z)
    px = x_root + 1.25 * m + rpp                                                   # pinion axis (= MOTOR_X)
    cutter = gear2d(m, zp, backlash=-CROWN_BACKLASH, add=1.25).extrude(r_out - r_in + 7).translate([0, 0, r_in - 4])
    cutter = cutter.rotate([-90, 0, 0])                                            # pinion axis along +y
    poses = []
    for phi in np.arange(-40.0, 40.01, 0.5):                                       # crown angle; pinion turns z/zp as much
        poses.append(cutter.rotate([0, phi * z / zp, 0]).translate([px, 0, 0]).rotate([-phi, 0, 0]))
    swept = union(poses)
    pitch = 360.0 / z
    a = np.radians(np.linspace(-1.0, pitch + 1.0, 14))                              # one gap + 1 deg overlap each side
    pts = [(x, 0.0, 0.0) for x in (x_root - 2, x_root + 4 * m)]
    pts += [(x, (r_out + 3) * math.cos(t), (r_out + 3) * math.sin(t)) for x in (x_root - 2, x_root + 4 * m) for t in a]
    gap = swept ^ M.hull_points(np.array(pts))
    return blank - union([gap.rotate([k * pitch, 0, 0]) for k in range(z)])


# ---- purchased parts (placeholders, for fit + looks) ---------------------------------------------
def rplidar_c1():
    z0 = LIDAR_BASE_Z
    base = box(-27.8, 27.8, LIDAR_Y - 27.8, LIDAR_Y + 27.8, z0, z0 + 23.1)
    return base + cyl(23.5, z0 + 23.1, z0 + 41.3, 0, LIDAR_Y, 64)


SERVO_TOP = BRIDGE_Z[1] - 11.4           # MG996R spline face (shaft down)


def mg996r():
    sx, sy = SERVO_SHAFT
    cx = sx + 10.35                       # case centre (shaft sits 10.35 from one end)
    case = box(cx - 20.35, cx + 20.35, sy - 9.85, sy + 9.85, SERVO_TOP, SERVO_TOP + 42.9)
    fl = box(cx - 27, cx + 27, sy - 9.85, sy + 9.85, BRIDGE_Z[1], BRIDGE_Z[1] + 2.5)
    fl -= union([cyl(2.25, 49, 55, cx + dx, sy + dy) for dx in (-24.75, 24.75) for dy in (-5, 5)])
    spline = cyl(2.95, SERVO_TOP - 4, SERVO_TOP, sx, sy)            # 25T output spline
    return case + fl + spline


def sg90():
    sx, sy = SG90_SHAFT
    cy = sy - 5.5
    bot = MID_Z1 - 15.9
    body = box(-6.1, 6.1, cy - 11.4, cy + 11.4, bot, bot + 22.7)
    fl = box(-6.1, 6.1, cy - 16.1, cy + 16.1, MID_Z1, MID_Z1 + 2.5)
    return body + fl + cyl(2.4, bot + 22.7, bot + 26.7, sx, sy)


def motor25ga():
    x, z = MOTOR_X, AX_Z
    return (cyly(12.5, MOTOR_FACE_Y, MOTOR_FACE_Y + 21, x, z) + cyly(12.2, MOTOR_FACE_Y + 21, MOTOR_FACE_Y + 52, x, z)
            + cyly(12.5, MOTOR_FACE_Y + 52, MOTOR_FACE_Y + 66, x, z) + cyly(3.5, MOTOR_FACE_Y - 2, MOTOR_FACE_Y, x, z)
            + cyly(2.0, MOTOR_FACE_Y - 9.5, MOTOR_FACE_Y, x, z))


def brio100(cam_z0):
    sx, sy = SG90_SHAFT
    body = box(-36.5, 36.5, sy - 8, sy + 17, cam_z0, cam_z0 + 32)
    lens = cyly(7, sy + 17, sy + 19, 0, cam_z0 + 16)
    clip = box(-15, 15, sy - 14, sy - 3, cam_z0 - 22, cam_z0)      # Brio's own folding clip, hooked on the lip
    return body + lens + clip


def board(x0, y0, w, h, z, t=1.6, comp_h=10.0):
    return box(x0, x0 + w, y0, y0 + h, z, z + t) + box(x0 + 4, x0 + w - 4, y0 + 4, y0 + h - 4, z + t, z + t + comp_h)


def pi4(z):
    pcb = box(PI_X0, PI_X0 + 56, PI_Y0, PI_Y0 + 85, z, z + 1.4)
    usb = box(PI_X0 + 2, PI_X0 + 54, PI_Y0 - 2, PI_Y0 + 20, z + 1.4, z + 17)
    soc = box(PI_X0 + 15, PI_X0 + 40, PI_Y0 + 35, PI_Y0 + 60, z + 1.4, z + 12)       # + heatsink
    return pcb + usb + soc


# ---- bottom plate -------------------------------------------------------------------------------
PLATE_RIGHT = [(30, 260), (36, 255), (48, 255), (48, 219), (36, 215), (36, 207), (42, 178), (42, 166), (38, 160),
               (38, 112), (52, 100), (52, -32)]   # front ear carries IR_front; IR_rear sits inside the rear edge


def plate_solid(z0=PL_Z0, z1=PL_Z1):
    return outline_cs(PLATE_RIGHT, 3).extrude(z1 - z0).translate([0, 0, z0])




def motor_mount_screws():
    return [(MOTOR_X - 18, 44), (MOTOR_X + 18, 44), (MOTOR_X - 18, 86), (MOTOR_X + 18, 86)]


def front_axle_screws():
    return [(-35.5, 159.5), (35.5, 159.5), (-6, 158), (6, 158)]


def servo_flange_pts():
    cx, sy = SERVO_SHAFT[0] + 10.35, SERVO_SHAFT[1]
    return [(cx + s * 24.75, sy + dy) for s in (-1, 1) for dy in (-5, 5)]


def pin_joints():
    """(x, y, z_head, z_tip, down) for every printed snap pin."""
    J = [(x, y, AX_Z + 8, PL_Z0, True) for x, y in cap_screws()]                     # caps + housing + plate
    J += [(x, y, PL_Z1 + 6.5, PL_Z0, True) for x, y in motor_mount_screws() + lidar_cradle_screws()]
    J += [(s * 35.5, 159.5, BRIDGE_Z[1], PL_Z0, True) for s in (-1, 1)]               # front axle walls
    J += [(s * 6, 158, PL_Z0, PL_Z1 + 6.5, False) for s in (-1, 1)]                   # front axle foot (from below)
    return J                                                                           # MG996R: snap hooks, no pins


def pin_holes():
    lr = PIN_R + SNAP + 0.4                                    # pocket so the snap lip has room past the tip
    return union([cyl(PIN_HOLE, 0, 70, x, y, 24) for x, y, *_ in pin_joints()]
                 + [cyl(lr, zt - LIP_L - 0.4, zt, x, y, 24) if dn else cyl(lr, zt, zt + LIP_L + 0.4, x, y, 24)
                    for x, y, _, zt, dn in pin_joints()])


def lidar_cradle_screws():
    return [(-27, LIDAR_Y - 34.3), (27, LIDAR_Y - 34.3), (-27, LIDAR_Y + 34), (27, LIDAR_Y + 34)]


def plate_screws():
    return STANDOFF_LOW + [(x, y) for x, y, *_ in pin_joints()]


PLATE_SPLIT = (140.0, (-22, 0, 22))         # y of the dovetail joint, tab x positions
MID_SPLIT = (128.0, (-36, -12, 12, 36))      # staggered from the plate joint so the standoffs brace both


def dovetail_split(shape, ys, xs, neck=7.0, head=11.0, depth=6.0):
    """Cut a 2D deck at y=ys into (rear, front) joined by glued dovetail tabs (rear part carries the tabs)."""
    tabs = cs_union([CS([np.array([(x - neck / 2, ys - 0.01), (x + neck / 2, ys - 0.01),
                                    (x + head / 2, ys + depth), (x - head / 2, ys + depth)])]) for x in xs])
    rear = (shape ^ keep_box(-300, 300, -300, ys)) + tabs
    front = (shape ^ keep_box(-300, 300, ys, 400)) - tabs.offset(JOIN_FIT, m3d.JoinType.Miter)
    return rear, front


def split_band(split):
    return keep_box(-300, 300, split[0] - 9, split[0] + 9)


def bottom_plate():
    out = outline_cs(PLATE_RIGHT, 3)
    # solid only where something sits: pin pads, the front-axle walls/foot, the lidar flange ring, strap slots
    keep = cs_union([keep_box(-41, -33, 150, 163), keep_box(33, 41, 150, 163), keep_box(-13, 13, 153, 163),
                     keep_box(-40, 40, 160, LIDAR_Y + 42), keep_box(-19, 21, -28, 28)]
                    + keep_circles(plate_screws(), 6.5)
                    + [keep_box(-46, -38, y - 12, y + 12) for y in (50, 100)] + [keep_box(-14, -6, y - 12, y + 12) for y in (50, 100)]
                    + [ir_footprint(c, 2) for c, _ in IR_MODULES.values()] + [split_band(PLATE_SPLIT)])
    cut2d = lattice(out, keep, margin=4.5)
    cut2d += keep_box(-18, 20, -27, 27) + keep_box(-31.4, 31.4, LIDAR_Y - 31.4, LIDAR_Y + 31.4)   # diff + lidar cup
    cut2d += cs_union(keep_circles(plate_screws(), PIN_HOLE))
    for y in (50, 100):                                                                        # battery strap slots
        cut2d += keep_box(-43.5, -40.5, y - 10, y + 10) + keep_box(-11.5, -8.5, y - 10, y + 10)
    rear, front = dovetail_split(out - cut2d, *PLATE_SPLIT)                  # IR modules: double-sided tape / glue
    return [h.extrude(PL_Z1 - PL_Z0).translate([0, 0, PL_Z0]) for h in (rear, front)]


def ir_footprint(c, grow=0.0):
    x, y = c
    return keep_box(x - IR_W / 2 - grow, x + IR_W / 2 + grow, y - IR_L / 2 - grow, y + IR_L / 2 + grow)


def ir_module(c, end):
    """Generic IR line/obstacle module: PCB under the plate, emitter+receiver at one end pointing down."""
    x, y = c
    z1 = PL_Z0
    pcb = box(x - IR_W / 2, x + IR_W / 2, y - IR_L / 2, y + IR_L / 2, z1 - 1.6, z1)
    ys = y + end * (IR_L / 2 - 5)
    sensor = box(x - 5, x + 5, ys - 3, ys + 3, z1 - 1.6 - 7, z1 - 1.6)          # face ~9 mm above the floor
    pot = box(x - 3.5, x + 3.5, y - end * 4 - 3.5, y - end * 4 + 3.5, z1 - 1.6 - 5, z1 - 1.6)
    header = box(x - 5, x + 5, y - end * (IR_L / 2 - 2) - 1.5, y - end * (IR_L / 2 - 2) + 1.5, z1 - 1.6 - 8.5, z1 - 1.6)
    return pcb + sensor + pot + header - cyl(1.6, 0, 30, x, y)


# ---- rear drive ---------------------------------------------------------------------------------
def cap_screws():
    return [(sx * xc, dy) for sx in (-1, 1) for xc, ys in ((22, 8.5), (48, 7.5)) for dy in (-ys, ys)]


def rear_housing():
    zb0, zb1 = PL_Z1, PL_Z1 + 4
    base = box(-54, -19, -14, 14, zb0, zb1) + box(20, 54, -14, 14, zb0, zb1)            # two halves, no bridge
    base -= box(-41, -29, -8, 8, zb0 - 1, zb1 + 1) + box(29, 41, -8, 8, zb0 - 1, zb1 + 1)    # lightening
    ups = [box(-25, -19, -11, 11, zb0, AX_Z), box(19, 25, -11, 11, zb0, AX_Z),
           box(-52, -44, -11, 11, zb0, AX_Z), box(44, 52, -11, 11, zb0, AX_Z)]
    h = base + union(ups)
    bores = [cylx(6 + FIT, -26, -18, 0, AX_Z), cylx(6 + FIT, 18, 26, 0, AX_Z),
             cylx(4 + FIT, -53, -43, 0, AX_Z), cylx(4 + FIT, 43, 53, 0, AX_Z)]
    return h - union(bores) - pin_holes()


def axle_cap(side):
    """Held down by two snap pins per upright that run through cap, upright, housing and plate."""
    c = box(19, 52, -11, 11, AX_Z, AX_Z + 8)
    c -= box(25.5, 43.5, -11.5, -5, AX_Z - 1, AX_Z + 10) + box(25.5, 43.5, 5, 11.5, AX_Z - 1, AX_Z + 10)   # waist
    c -= cylx(6 + FIT, 18, 26, 0, AX_Z) + cylx(4 + FIT, 24, 53, 0, AX_Z)
    c = c if side > 0 else c.mirror([1, 0, 0])
    return c - pin_holes()


def diff_carrier_halves():
    """Crown half carries the 3-arm spider cross; the cover only holds the left side gear. Both are trapped
    axially between the housing's inner uprights (collars), so no bolts hold the case together."""
    body_r = 20.0
    sx = DIFF_SPLIT_X
    o = np.array([0, 0, AX_Z])
    cav = M.sphere(CAV_R, 64).translate(o)
    cover = cylx(body_r, -12, sx, 0, AX_Z, 64) + cylx(6, -25.5, -12, 0, AX_Z) + cylx(8, -18.8, -12, 0, AX_Z)
    crown = cylx(body_r, sx, 12, 0, AX_Z, 64) + cylx(25.5, 12, CROWN_ROOT_X, 0, AX_Z, 96)
    crown += crown_teeth(CROWN_M, CROWN_Z, CROWN_R_IN, CROWN_R_OUT, CROWN_ROOT_X).translate(o)
    crown += cylx(6, CROWN_ROOT_X, 25.5, 0, AX_Z) + cylx(8, CROWN_ROOT_X + 2.8, 18.8, 0, AX_Z)
    peg_yz = [(15 * math.cos(math.radians(a + 60)), 15 * math.sin(math.radians(a + 60))) for a in SPIDER_ANG]   # between arms
    crown += union([cylx(1.5, sx - 3, sx + 0.01, y, AX_Z + z) for y, z in peg_yz])
    cut = [cav, cylx(SIDE_HUB_R + FIT, -11.5, 11.5, 0, AX_Z), cylx(4 + FIT, -30, 30, 0, AX_Z)]
    slot = cyl(2.0 + 0.1, 0, 19) + box(sx - 1, 0, -2.1, 2.1, 0, 19)              # arm hole + drop-in slot from the split face
    cross = union([slot.rotate([a - 90, 0, 0]).translate(o) for a in SPIDER_ANG])
    cover -= union(cut + [cross] + [cylx(1.5 + FIT, sx - 3.5, sx + 1, y, AX_Z + z) for y, z in peg_yz])   # arms clamp between halves
    crown -= union(cut + [cross])
    return cover, crown


CAV_R = 12.2
DIFF_FACE = 5.0
SIDE_HUB_R, SPIDER_HUB_R = 6.5, 4.0
SPIDER_ANG = (90.0, 210.0, 330.0)        # 3 spiders: at 120 deg their tip cones can't touch (4 at 90 deg clash)
SPIDER_PHASE = (15.0, 15.0, 15.0)        # half a tooth: centre of the zero-overlap band (checked 14..16 deg)


def spider_cross():
    """Printed 3-arm 'Y' carrying the spider gears. Flat on one face so it prints lying down (PETG)."""
    arm = cyl(2.0, 0, 18.5)
    c = union([arm.rotate([a - 90, 0, 0]) for a in SPIDER_ANG]).translate([0, 0, AX_Z])
    return c - box(-5, -1.6, -20, 20, AX_Z - 20, AX_Z + 20)


def diff_gears():
    """Two side gears + three spiders. Side gears are driven through a hex in their HUB only, so every tooth
    has solid material under its root. Hubs are long enough that each gear seats ~0.3 mm from its thrust
    face, so bevel thrust can't push a gear out of mesh."""
    rp = DIFF_M * DIFF_Z / 2
    side_hub = 11.5 - rp - 0.3                                                       # hub end meets the bore step
    side = bevel(DIFF_M, DIFF_Z, DIFF_FACE, SIDE_HUB_R, side_hub) - hexz(7.0 + 2 * FIT, -side_hub - 1, 0.0)
    spider_hub = math.sqrt(CAV_R ** 2 - SPIDER_HUB_R ** 2) - rp - 0.3               # hub end meets the cavity sphere
    spider = bevel(DIFF_M, DIFF_Z, DIFF_FACE, SPIDER_HUB_R, spider_hub, bore_r=2.0 + FIT)
    G = {"side_gear_R": side.rotate([0, -90, 0]).translate([rp, 0, AX_Z]),
         "side_gear_L": side.rotate([0, 90, 0]).translate([-rp, 0, AX_Z])}
    for tag, a, ph in zip("ABC", SPIDER_ANG, SPIDER_PHASE):
        d = np.array([0, math.cos(math.radians(a)), math.sin(math.radians(a))])
        G[f"spider_gear_{tag}"] = spider.rotate([0, 0, ph]).rotate([a + 90, 0, 0]).translate(rp * d + [0, 0, AX_Z])
    return G


def spider_axis(tag):
    a = math.radians(SPIDER_ANG["ABC".index(tag)])
    return np.array([0, math.cos(a), math.sin(a)])


def wheel_x(rear):
    """(inner face, hub start, outer face) |x| for a wheel."""
    c = REAR_X if rear else FRONT_X
    return c - WHEEL_W / 2, c + WHEEL_W / 2 - HUB_L, c + WHEEL_W / 2


def clip_groove(x_out):
    """Groove for the axle clip just outside the hub face x_out (on an 8 mm round tip)."""
    return cylx(4.5, x_out + 0.2, x_out + 1.6) - cylx(3.2, x_out, x_out + 2)


def half_shaft(side):
    _, hub0, x_out = wheel_x(True)
    sh = hexz(7.0, DIFF_M * DIFF_Z / 2, 12) + cyl(4, 12, 52.2) + cyl(5, 52.2, hub0) + hexz(7.0, hub0, x_out) + cyl(4, x_out, x_out + 3.5)
    sh = sh.rotate([0, 90, 0]) - clip_groove(x_out)
    sh = sh.translate([0, 0, AX_Z])
    return sh if side > 0 else sh.mirror([1, 0, 0])


def motor_pinion():
    p = spur(CROWN_M, PIN_Z, 0, 8) + cyl(5.5, 8, 8.01)
    p -= cyl(2 + 0.05, -1, 9) - box(1.5, 5, -5, 5, -2, 10)      # 4 mm D-bore, press fit (no grub screw)
    return p.rotate([90, 0, 0]).translate([MOTOR_X, MOTOR_FACE_Y - 4.5, AX_Z])


def motor_mount():
    """Face plate (axial stop) + two snap-in ring cradles; the motor clicks in from above. A dab of hot glue
    on the rings stops it turning."""
    x, z = MOTOR_X, AX_Z
    fy0 = MOTOR_FACE_Y - 3
    face = box(x - 16, x + 16, fy0, MOTOR_FACE_Y, PL_Z1, z + 13)
    rings = union([cyly(15, y0, y0 + 6, x, z) for y0 in (50, 80)])
    rings -= cyly(12.6, 40, 100, x, z) + box(x - 10, x + 10, 40, 100, z + 6, z + 20) + box(x - 20, x + 20, 40, 100, 0, PL_Z1)
    rails = box(x - 16, x - 10, fy0, 90, PL_Z1, PL_Z1 + 4) + box(x + 10, x + 16, fy0, 90, PL_Z1, PL_Z1 + 4)
    tabs = union([box(min(px, x + s * 13), max(px, x + s * 13), py - 4, py + 4, PL_Z1, PL_Z1 + 4)
                  for (px, py), s in zip(motor_mount_screws(), (-1, 1, -1, 1))])
    bosses = union([cyl(4.5, PL_Z1, PL_Z1 + 6.5, px, py) for px, py in motor_mount_screws()])
    m = face + rings + rails + tabs + bosses
    m -= cyly(4.5, fy0 - 1, MOTOR_FACE_Y + 1, x, z) + cyly(12.6, MOTOR_FACE_Y, 100, x, z)
    return m - pin_holes()


# ---- front axle + steering ----------------------------------------------------------------------
def front_axle():
    """Two C-hubs (arms reach into the hollow wheels) joined by the servo bridge."""
    bz0, bz1 = BRIDGE_Z
    sx, sy = SERVO_SHAFT
    cx = sx + 10.35
    parts = [box(-40, 40, 130, 162, bz0, bz1)]
    for s in (-1, 1):
        xa, xb = sorted((s * 34, s * 40))
        parts += [M.hull(cyl(6, bz0, bz1, s * KP_X, WB) + box(xa, xb, 143, 157, bz0, bz1)),                    # upper arm
                  M.hull(cyl(6, *LOWER_ARM_Z, s * KP_X, WB) + box(xa, xb, 150, 162, *LOWER_ARM_Z)),            # lower arm
                  box(xa, xb, 152, 162, PL_Z1, bz1)]                                                           # wall
        end = cx + s * 27                                                                                      # MG996R flange end
        x0, x1 = sorted((end, end + s * 2.2))
        parts += [box(x0, x1, sy - 5, sy + 5, bz1, bz1 + 2.5 + 0.2 + 1.4)]                                     # snap hook post
        l0, l1 = sorted((end, end - s * 1.5))
        parts += [box(l0, l1, sy - 5, sy + 5, bz1 + 2.7, bz1 + 4.1)]                                           # hook lip over flange
    parts += [box(-12, 12, 160, 162, PL_Z1, bz1), box(-12, 12, 154, 162, PL_Z1, PL_Z1 + 6.5)]                 # front web + foot
    cut = [box(cx - 20.75, cx + 20.75, sy - 10.25, sy + 10.25, 40, 60), plate_solid(PL_Z0 - 3, PL_Z1)]
    cut += [box(-36, -24, 133, 158, bz0 - 1, bz1 + 1)]                                                         # lightening
    cut += [cyl(KP_R + FIT, 0, 60, s * KP_X, WB) for s in (-1, 1)]                                             # printed kingpins
    return union(parts) - union(cut) - pin_holes()


def kingpin(side):
    return snap_pin(side * KP_X, WB, BRIDGE_Z[1], LOWER_ARM_Z[0], KP_R, 5.0, down=True)


def knuckle(side):
    kx, ky = KP_X, WB
    _, hub0, x_out = wheel_x(False)
    k = cyl(5.5, ARM_Z[0], BRIDGE_Z[0] - 0.5, kx, ky)
    k += cylx(4, kx, x_out + 3.5, ky, AX_Z) + cylx(6, kx, hub0 - 0.3, ky, AX_Z)
    k += M.hull(cyl(5.5, *ARM_Z, kx, ky) + cyl(4, *ARM_Z, ARM_END[0], ARM_END[1]))
    k -= cyl(KP_R + FIT, 0, 60, kx, ky) + cyl(PIV_R + FIT, 0, 40, ARM_END[0], ARM_END[1])
    k -= clip_groove(x_out).translate([0, ky, AX_Z])
    return k if side > 0 else k.mirror([1, 0, 0])


def tie_rod(a_end, h_end):
    a, h = np.asarray(a_end, float), np.asarray(h_end, float)
    r = M.hull(cyl(4, *TIE_Z, *a) + cyl(4, *TIE_Z, *h))
    return r - cyl(PIV_R + FIT, 0, 50, *a) - cyl(PIV_R + FIT, 0, 50, *h)


def spline_bore(teeth, r_major, r_minor, z0, z1, x, y):
    """Internal spline for a printed servo horn (sawtooth approximation of the servo's serrations)."""
    pts = [((r_major if k % 2 == 0 else r_minor) + SPLINE_FIT) * np.array([math.cos(a), math.sin(a)])
           for k, a in enumerate(np.linspace(0, 2 * math.pi, 2 * teeth, endpoint=False))]
    return CS([np.array(pts)]).extrude(z1 - z0).translate([x, y, z0])


def horn_adapter():
    """Printed MG996R steering horn: 25T spline pressed onto the output shaft (no stock horn, no screw).
    Superglue it once the steering is centred, or use the servo's own horn screw."""
    sx, sy = SERVO_SHAFT
    z0 = TIE_Z[1] + 0.2
    arm = M.hull(cyl(6, z0, SERVO_TOP - 6.5, sx, sy)
                 + box(-HORN_PIV - 4, HORN_PIV + 4, ARM_END[1] - 4, ARM_END[1] + 4, z0, SERVO_TOP - 6.5))
    a = arm + cyl(6.5, z0, SERVO_TOP - 0.5, sx, sy)
    a -= spline_bore(25, 3.0, 2.65, SERVO_TOP - 4.6, SERVO_TOP, sx, sy)
    a -= cyl(1.3, 0, 60, sx, sy)
    a -= union([cyl(PIV_R + FIT, 0, 60, s * HORN_PIV, ARM_END[1]) for s in (-1, 1)])
    return a


def pivot_pins():
    """Two at the steering arms (head under the arm), two at the horn (head under the tie rod)."""
    pins = {}
    for s, tag in ((-1, "L"), (1, "R")):
        ax, ay = s * ARM_END[0], ARM_END[1]
        pins[f"pivot_pin_arm_{tag}"] = snap_pin(ax, ay, ARM_Z[0] - 0.2, TIE_Z[1] + 0.2, PIV_R, 4.0, down=False)
        pins[f"pivot_pin_horn_{tag}"] = snap_pin(s * HORN_PIV, ay, TIE_Z[0] - 0.2, SERVO_TOP - 6.5 + 0.2, PIV_R, 3.5, down=False)
    return pins


# ---- wheels -------------------------------------------------------------------------------------
def wheel(rear, side):
    """Spoked rim, hub at the OUTER face so the inside is hollow (front: kingpin sits in there)."""
    w = WHEEL_W
    rim = cyl(RIM_R, 0, w) - cyl(RIM_R - 1.8, -1, w + 1)
    lip = cyl(RIM_R + 1.2, 0, 1.2) - cyl(RIM_R - 1.8, -1, 2.2)
    rim += lip + lip.translate([0, 0, w - 1.2])                                     # bead lips (rings)
    hub = cyl(6.5, w - HUB_L, w)
    spokes = union([box(-1.6, 1.6, 0, RIM_R - 1, w - 3, w).rotate([0, 0, a]) for a in range(0, 360, 72)])
    wh = rim + hub + spokes
    wh -= hexz(7.0 + 2 * FIT, -1, w + 1) if rear else cyl(4 + FIT, -1, w + 1)
    x_in, _, _ = wheel_x(rear)
    wh = wh.rotate([0, 90, 0]).translate([x_in, 0.0 if rear else WB, AX_Z])
    return wh if side > 0 else wh.mirror([1, 0, 0])


def tire(rear, side):
    w = WHEEL_W
    t = cyl(WHEEL_R, 0, w, seg=96) - cyl(RIM_R, -1, w + 1, seg=96)
    for z in (5, 10, 15):
        t -= cyl(WHEEL_R + 1, z - 0.8, z + 0.8, seg=96) - cyl(WHEEL_R - 1, z - 2, z + 2, seg=96)
    t -= cyl(RIM_R + 1.3, -1, 1.2, seg=96) + cyl(RIM_R + 1.3, w - 1.2, w + 1, seg=96)
    x_in, _, _ = wheel_x(rear)
    t = t.rotate([0, 90, 0]).translate([x_in, 0 if rear else WB, AX_Z])
    return t if side > 0 else t.mirror([1, 0, 0])


def wheel_clip(rear, side):
    _, _, x_out = wheel_x(rear)
    c = axle_clip().translate([x_out + 0.3, 0 if rear else WB, AX_Z])
    return c if side > 0 else c.mirror([1, 0, 0])


# ---- lidar --------------------------------------------------------------------------------------
def lidar_cradle():
    """Cup with 4 corner snap hooks over the C1's base (no screws). Walls stay under the scan.
    Snap pins through its 4 bosses hold it to the plate."""
    y, z0 = LIDAR_Y, LIDAR_BASE_Z - 2.4
    wall_top = LIDAR_SCAN - 10
    hook_top = LIDAR_BASE_Z + 23.1 + 0.2 + 1.4                    # 5 mm below the scan plane
    cup = box(-30.5, 30.5, y - 30.5, y + 30.5, z0, wall_top)
    cup -= box(-28.1, 28.1, y - 28.1, y + 28.1, LIDAR_BASE_Z, wall_top + 1)
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * 26.5, y + sy * 26.5
            post = box(cx - 4, cx + 4, cy - 4, cy + 4, wall_top - 1, hook_top) - box(-28.1, 28.1, y - 28.1, y + 28.1, 0, LIDAR_BASE_Z + 23.3)
            cup += post ^ box(-30.5, 30.5, y - 30.5, y + 30.5, 0, 60)
    fl = box(-33, 33, y - 35.5, y + 37, PL_Z1, PL_Z1 + 2.5) - box(-30, 30, y - 30, y + 30, 0, 50)
    fl += union([cyl(4.5, PL_Z1, PL_Z1 + 6.5, sx_, sy_) for sx_, sy_ in lidar_cradle_screws()])
    cut = [box(-18, 18, y - 18, y + 18, 0, LIDAR_BASE_Z + 1)]                                   # floor window
    cut += [box(-7, 7, y - 32, y - 27, LIDAR_BASE_Z + 4, wall_top + 1)]                        # cable exit (rear)
    cut += [box(-14, 14, y - 31, y + 31, LIDAR_BASE_Z + 6, wall_top - 2)]                      # side windows (y)
    cut += [box(-31, 31, y - 14, y + 14, LIDAR_BASE_Z + 6, wall_top - 2)]                      # side windows (x)
    return (cup + fl) - union(cut) - pin_holes()


# ---- decks --------------------------------------------------------------------------------------
def standoff(x, y, lo=(PL_Z0, PL_Z1), hi=(MID_Z0, MID_Z1)):
    """Solid printed hex standoff between deck `lo` and deck `hi` (each (z_bottom, z_top)), with a split
    snap peg at each end: down through `lo`, up through `hi`. Print lying on a hex flat (PETG)."""
    z0, z1 = lo[1], hi[0]
    s = hexz(8.0, z0, z1, x, y)
    s += cyl(PIN_R, lo[0], z0, x, y, 24) + snap_lip(x, y, lo[0], -1, PIN_R)
    s += cyl(PIN_R, z1, hi[1], x, y, 24) + snap_lip(x, y, hi[1], 1, PIN_R)
    s -= (box(x - 0.6, x + 0.6, y - 6, y + 6, lo[0] - LIP_L - 0.1, z0 + 5)
          + box(x - 0.6, x + 0.6, y - 6, y + 6, z1 - 5, hi[1] + LIP_L + 0.1))
    return s


MID_RIGHT = [(40, 236), (55, 222), (55, -26), (50, -31)]
TOP_RIGHT = [(44, 100), (50, 94), (50, -26), (45, -31)]      # half-length rear deck, over the Pi + ESP32


def top_deck():
    """Spare rear deck for whatever needs a home later (e-stop, start button, wiring): lattice with an
    M3-size hole at every node for zip ties."""
    out = outline_cs(TOP_RIGHT, 4)
    keep = cs_union(keep_circles(STANDOFF_HIGH, 7))
    cut2d = lattice(out, keep, margin=5, node_holes=True) + cs_union(keep_circles(STANDOFF_HIGH, PIN_HOLE))
    return (out - cut2d).extrude(TOP_Z1 - TOP_Z0).translate([0, 0, TOP_Z0])


def mid_deck():
    """Top layer: electronics + the camera's SG90 pan servo (snap hooks over its flanges) at the front."""
    out = outline_cs(MID_RIGHT, 4)
    cy = SG90_SHAFT[1] - 5.5
    keep = cs_union(keep_circles(STANDOFF_LOW + STANDOFF_HIGH, 7) + keep_circles(PI_HOLES, 5.5)
                    + [keep_box(ESP_X0 - 3.5, ESP_X0 - 0.5, ESP_Y0 - 1, ESP_Y0 + 26.4),
                       keep_box(ESP_X0 + 69.5, ESP_X0 + 72.5, ESP_Y0 - 1, ESP_Y0 + 26.4), keep_box(-22, 22, -10, 7),
                       keep_box(-12, 12, cy - 22, cy + 22), split_band(MID_SPLIT)])
    cut2d = lattice(out, keep, margin=5, node_holes=True)
    cut2d += cs_union(keep_circles(STANDOFF_LOW + STANDOFF_HIGH, PIN_HOLE)) + keep_box(-20, 20, -8, 5)   # + cable pass-through
    cut2d += keep_box(-6.4, 6.4, cy - 11.7, cy + 11.7)                                         # SG90 pocket
    halves = [h.extrude(MID_Z1 - MID_Z0).translate([0, 0, MID_Z0]) for h in dovetail_split(out - cut2d, *MID_SPLIT)]
    d = M()   # Pi + ESP32: double-sided tape on the solid pads the lattice leaves (keep list above), no mounts
    for s in (-1, 1):                                                                          # SG90 snap hooks
        yy = cy + s * 16.1
        y0, y1 = sorted((yy, yy + s * 2.2))
        d += box(-4, 4, y0, y1, MID_Z1, MID_Z1 + 2.5 + 0.2 + 1.4)
        l0, l1 = sorted((yy, yy - s * 1.5))
        d += box(-4, 4, l0, l1, MID_Z1 + 2.7, MID_Z1 + 4.1)
    ys = MID_SPLIT[0]                                                # every feature sits wholly on one side
    return halves[0] + (d ^ box(-300, 300, -300, ys, 0, 300)), halves[1] + (d ^ box(-300, 300, ys, 400, 0, 300))


def cam_cradle():
    """Printed SG90 horn + 'monitor edge' the Brio's own clip grips. 21T spline presses onto the SG90
    output (superglue or the SG90's own horn screw keeps it on)."""
    sx, sy = SG90_SHAFT
    spl_top = MID_Z1 - 15.9 + 26.7                 # SG90 output spline top
    z0 = spl_top + 0.5
    hub = cyl(5.5, spl_top - 3.5, z0, sx, sy)
    base = cyl(13, z0, z0 + 3, sx, sy)
    lip = box(-30, 30, sy - 3, sy + 3, z0 + 3, z0 + 40)
    lip -= box(-24, -6, sy - 4, sy + 4, z0 + 8, z0 + 32) + box(6, 24, sy - 4, sy + 4, z0 + 8, z0 + 32)   # windows
    rib = M.hull(box(-2, 2, sy - 12, sy + 12, z0, z0 + 3) + box(-2, 2, sy - 3, sy + 3, z0 + 3, z0 + 25))
    c = hub + base + lip + rib
    c -= spline_bore(21, 2.45, 2.15, spl_top - 3.6, spl_top + 0.01, sx, sy) + cyl(1.0, spl_top - 4, z0 + 3.1, sx, sy)
    return c, z0 + 40


# ---- steering kinematics ------------------------------------------------------------------------
def rot(v, deg):
    a = math.radians(deg)
    return np.array([v[0] * math.cos(a) - v[1] * math.sin(a), v[0] * math.sin(a) + v[1] * math.cos(a)])


def steer_solution(phi):
    """Servo horn angle phi (deg, +CCW from above) -> (left, right) wheel angles (deg, +CCW)."""
    S = SERVO_SHAFT
    out = {}
    for s in (-1, 1):
        K = np.array([s * KP_X, WB])
        A0 = np.array([s * ARM_END[0], ARM_END[1]])
        H0 = np.array([s * HORN_PIV, ARM_END[1]])
        L = np.linalg.norm(A0 - H0)
        H = S + rot(H0 - S, phi)
        f = lambda d: np.linalg.norm(K + rot(A0 - K, d) - H) - L
        lo, hi = -60.0, 60.0          # |A-H| is monotone in d over +-60 deg for this linkage -> bisection
        flo = f(lo)
        for _ in range(80):
            mid = (lo + hi) / 2
            fm = f(mid)
            if (fm > 0) == (flo > 0):
                lo, flo = mid, fm
            else:
                hi = mid
        out[s] = (lo + hi) / 2
    return out[-1], out[1]


def ackermann_report():
    rows = []
    for phi in np.arange(0, HORN_MAX + 0.1, 4.0):
        dl, dr = steer_solution(phi)
        inner, outer = max(abs(dl), abs(dr)), min(abs(dl), abs(dr))
        ideal = math.degrees(math.atan(1 / (1 / math.tan(math.radians(inner)) + 2 * KP_X / WB))) if inner > 0.01 else 0
        rows.append(dict(horn=float(phi), left=round(dl, 2), right=round(dr, 2), outer_ideal=round(ideal, 2),
                         err=round(outer - ideal, 2)))
    return rows


def steered(part, side, deg):
    k = np.array([side * KP_X, WB, 0])
    return part.translate(-k).rotate([0, 0, deg]).translate(k)


# ---- assembly -----------------------------------------------------------------------------------
PC = {"plate": (60, 64, 72), "bracket": (40, 120, 220), "gear": (245, 130, 30), "wheel": (230, 230, 230),
      "tire": (25, 25, 25), "standoff": (235, 235, 235), "link": (90, 160, 240), "pin": (250, 200, 60)}
BC = {"lidar": (20, 20, 22), "servo": (25, 50, 140), "motor": (190, 190, 195), "pcb": (20, 110, 60),
      "battery": (240, 200, 40), "camera": (35, 35, 38), "estop": (210, 30, 30), "button": (40, 170, 70),
      "led": (255, 80, 40)}


def build(steer=0.0):
    P = {}   # name -> (manifold, colour, printed, group)

    def add(n, m, c, pr, g):
        P[n] = (m, c, pr, g)

    pr_, pf_ = bottom_plate()
    add("bottom_plate_rear", pr_, PC["plate"], True, "chassis")
    add("bottom_plate_front", pf_, PC["plate"], True, "chassis")
    add("rear_housing", rear_housing(), PC["bracket"], True, "chassis")
    add("axle_cap_R", axle_cap(1), PC["bracket"], True, "chassis")
    add("axle_cap_L", axle_cap(-1), PC["bracket"], True, "chassis")
    cover, crown = diff_carrier_halves()
    add("diff_cover", cover, PC["gear"], True, "chassis")
    add("diff_case_crown", crown, PC["gear"], True, "chassis")
    add("spider_cross", spider_cross(), PC["pin"], True, "chassis")
    for n, g in diff_gears().items():
        add(n, g, PC["gear"], True, "chassis")
    add("motor_pinion", motor_pinion(), PC["gear"], True, "chassis")
    add("half_shaft_R", half_shaft(1), PC["link"], True, "chassis")
    add("half_shaft_L", half_shaft(-1), PC["link"], True, "chassis")
    add("motor_mount", motor_mount(), PC["bracket"], True, "chassis")
    add("front_axle", front_axle(), PC["bracket"], True, "chassis")
    dl, dr = steer_solution(steer) if steer else (0.0, 0.0)
    pins = pivot_pins()
    S = np.array([*SERVO_SHAFT, 0])
    for s, d in ((-1, dl), (1, dr)):
        tag = "L" if s < 0 else "R"
        add(f"kingpin_{tag}", kingpin(s), PC["pin"], True, "chassis")
        add(f"knuckle_{tag}", steered(knuckle(s), s, d), PC["link"], True, "chassis")
        add(f"wheel_front_{tag}", steered(wheel(False, s), s, d), PC["wheel"], True, "chassis")
        add(f"tire_front_{tag}", steered(tire(False, s), s, d), PC["tire"], True, "chassis")
        add(f"axle_clip_front_{tag}", steered(wheel_clip(False, s), s, d), PC["pin"], True, "chassis")
        add(f"wheel_rear_{tag}", wheel(True, s), PC["wheel"], True, "chassis")
        add(f"tire_rear_{tag}", tire(True, s), PC["tire"], True, "chassis")
        add(f"axle_clip_rear_{tag}", wheel_clip(True, s), PC["pin"], True, "chassis")
        add(f"pivot_pin_arm_{tag}", steered(pins[f"pivot_pin_arm_{tag}"], s, d), PC["pin"], True, "chassis")
        add(f"pivot_pin_horn_{tag}", pins[f"pivot_pin_horn_{tag}"].translate(-S).rotate([0, 0, steer]).translate(S),
            PC["pin"], True, "chassis")
        K = np.array([s * KP_X, WB])
        a_end = K + rot(np.array([s * ARM_END[0], ARM_END[1]]) - K, d)
        h_end = SERVO_SHAFT + rot(np.array([s * HORN_PIV, ARM_END[1]]) - SERVO_SHAFT, steer)
        add(f"tie_rod_{tag}", tie_rod(a_end, h_end), PC["link"], True, "chassis")
    add("horn_adapter", horn_adapter().translate(-S).rotate([0, 0, steer]).translate(S), PC["link"], True, "chassis")
    add("lidar_cradle", lidar_cradle(), PC["bracket"], True, "chassis")
    for i, (x, y) in enumerate(STANDOFF_LOW):
        add(f"standoff_low_{i}", standoff(x, y), PC["standoff"], True, "chassis")
    mr_, mf_ = mid_deck()
    add("mid_deck_rear", mr_, PC["plate"], True, "mid")
    add("mid_deck_front", mf_, PC["plate"], True, "mid")
    for i, (x, y) in enumerate(STANDOFF_HIGH):
        add(f"standoff_high_{i}", standoff(x, y, (MID_Z0, MID_Z1), (TOP_Z0, TOP_Z1)), PC["standoff"], True, "mid")
    add("top_deck", top_deck(), PC["plate"], True, "top")
    cc, lip_top = cam_cradle()
    add("cam_cradle", cc, PC["bracket"], True, "cam")
    # bought parts
    add("RPLIDAR_C1", rplidar_c1(), BC["lidar"], False, "chassis")
    add("MG996R", mg996r(), BC["servo"], False, "chassis")
    add("JGA25_370_encoder_motor", motor25ga(), BC["motor"], False, "chassis")
    add("LiPo_3S_2200", box(-38.5, -13.5, 16, 122, PL_Z1, PL_Z1 + 35), BC["battery"], False, "chassis")   # on its side
    add("Raspberry_Pi_4B", pi4(MID_Z1 + 1.2), BC["pcb"], False, "mid")        # on 1 mm foam tape
    add("ESP32_S3_DevKitC", board(ESP_X0, ESP_Y0, 69, 25.4, MID_Z1 + 8.5, comp_h=3), BC["pcb"], False, "mid")
    # IBT-2 (BTS7960) at its largest listed size, 50 x 50 x 42 mm with the heatsink, on 3 mm spacers
    add("IBT2_BTS7960", board(-33, 121, 50, 50, MID_Z1 + 3, comp_h=0) + box(-33, 17, 121, 171, MID_Z1 + 4.6, MID_Z1 + 45),
        BC["pcb"], False, "mid")
    add("buck_5V_Pi", board(29, 80, 26, 51, MID_Z1 + 5, comp_h=10), BC["pcb"], False, "mid")
    for i, (x, y, zh, zt, down) in enumerate(pin_joints()):
        add(f"snap_pin_{i:02d}", snap_pin(x, y, zh, zt, down=down), PC["pin"], True, "chassis")
    add("buck_6V_servo", board(-10, -27, 43, 21, MID_Z1 + 5, comp_h=10), BC["pcb"], False, "mid")
    add("lidar_usb_adapter", board(-55, 80, 24, 18, MID_Z1 + 5, comp_h=4), BC["pcb"], False, "mid")
    add("IMU_BNO085", board(-36, -28, 20, 25, MID_Z1 + 5, comp_h=2), BC["pcb"], False, "mid")
    add("SG90", sg90(), BC["servo"], False, "mid")
    for n, (c, end) in IR_MODULES.items():
        add(n, ir_module(c, end), BC["pcb"], False, "chassis")
    add("Logitech_Brio_100", brio100(lip_top + 2), BC["camera"], False, "cam")
    return P


# ---- fasteners ----------------------------------------------------------------------------------
def fastener_bom():
    """No bought fasteners: printed snap pins, pegs, clips and hooks; glue/tape for the electronics."""
    J = pin_joints()
    groups = {"axle caps + rear housing": len(cap_screws()),
              "motor mount": len(motor_mount_screws()), "front axle": len(front_axle_screws()),
              "lidar cradle": len(lidar_cradle_screws())}
    lengths = sorted({round(abs(zh - zt) + 2 + LIP_L, 1) for _, _, zh, zt, _ in J})
    return dict(bought="none", snap_pins=len(J), pin_lengths_mm=lengths, joints=groups,
                printed_fasteners=["standoff snap pegs x20", "kingpin x2", "pivot pin x4", "axle clip x4",
                                   "spider cross x1", "MG996R + SG90 splined horns", "motor snap rings",
                                   "lidar snap hooks", "SG90 snap hooks"],
                glue_or_tape=["Raspberry Pi 4B", "ESP32-S3", "IR modules", "IBT-2", "buck converters", "IMU", "lidar USB adapter", "LiPo",
                              "e-stop + start button"])


# ---- checks -------------------------------------------------------------------------------------
BOUGHT_MASS = {"RPLIDAR_C1": 110, "MG996R": 55, "SG90": 9, "JGA25_370_encoder_motor": 95, "LiPo_3S_2200": 185,
               "Raspberry_Pi_4B": 46, "ESP32_S3_DevKitC": 10, "IBT2_BTS7960": 66, "buck_5V_Pi": 15, "buck_6V_servo": 15,
               "lidar_usb_adapter": 5, "IMU_BNO085": 3, "Logitech_Brio_100": 105, "IR_front": 4, "IR_rear": 4,
               "estop_and_start_button_hot_glued": 40, "screws_inserts_wires_glue": 100}
V1_PRINTED_G = 433                  # v1 estimate, same method, for the before/after
INTENDED_TOUCH = {frozenset(p) for p in [
    ("RPLIDAR_C1", "lidar_cradle"), ("JGA25_370_encoder_motor", "motor_mount"), ("JGA25_370_encoder_motor", "motor_pinion"),
    ("MG996R", "front_axle"), ("MG996R", "horn_adapter"), ("SG90", "mid_deck_front"), ("SG90", "cam_cradle"),
    ("Logitech_Brio_100", "cam_cradle"),
    ("bottom_plate_rear", "bottom_plate_front"), ("mid_deck_rear", "mid_deck_front")]}   # pinion/crown: mesh_check()


def interference(P, tol=0.01):
    items = list(P.items())
    bbs = {n: np.array(v[0].bounding_box()) for n, v in items}
    hits = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = items[i][0], items[j][0]
            A, B = bbs[a], bbs[b]
            if np.any(A[:3] > B[3:]) or np.any(B[:3] > A[3:]) or frozenset((a, b)) in INTENDED_TOUCH:
                continue
            v = (items[i][1][0] ^ items[j][1][0]).volume()
            if v > tol:
                hits.append((a, b, round(v, 1)))
    return hits


def printed_grams(m):
    v = m.volume()
    return round(v / 1000 * 1.24 * (0.55 if v > 15000 else 0.85), 1)


def to_trimesh(m, color):
    mesh = m.to_mesh()
    t = trimesh.Trimesh(mesh.vert_properties[:, :3], mesh.tri_verts, process=False)
    t.visual.face_colors = [*color, 255]
    return t


MOVING = ("knuckle", "wheel_front", "tire_front", "axle_clip_front", "tie_rod", "horn", "pivot_pin")
MIN_RUN_GAP = 0.15        # anything that moves relative to a neighbour must clear it by this much
CONTACT_OK = {frozenset(p) for p in [            # deliberate contact: press fits, splines, parts sitting on parts
    ("motor_pinion", "JGA25_370_encoder_motor"), ("motor_mount", "JGA25_370_encoder_motor"),
    ("bottom_plate_rear", "JGA25_370_encoder_motor"), ("horn_adapter", "MG996R"),
    ("knuckle_L", "axle_clip_front_L"), ("knuckle_R", "axle_clip_front_R"),   # clip seated in its groove
    ("diff_case_crown", "motor_pinion")] + [(f"side_gear_{a}", f"spider_gear_{b}") for a in "LR" for b in "ABC"]}
# ^ gear pairs sit 0.06-0.09 apart at rest; mesh_check() rotates them through a tooth pitch instead


def rigid_body(n):
    """Which rigid body a part rides with, for the running-clearance audit."""
    if n.startswith("spider_gear"):
        return n                                       # each spider spins on the cross
    if n.startswith(("diff_cover", "diff_case_crown", "spider_cross")):
        return "carrier"
    if n.startswith(("side_gear", "half_shaft", "wheel_rear", "tire_rear", "axle_clip_rear")):
        return "rear_" + n[-1]
    if n.startswith(("wheel_front", "tire_front", "axle_clip_front")):
        return "front_wheel_" + n[-1]
    for k in ("knuckle", "tie_rod", "pivot_pin_arm", "pivot_pin_horn", "motor_pinion", "horn_adapter", "JGA25"):
        if n.startswith(k):
            return k + n[-1]
    return "static"


def running_clearance(P):
    """Min gap between every pair of chassis parts on different rigid bodies (overlap counts as -volume)."""
    names = [n for n in P if P[n][3] == "chassis"]
    bbs = {n: np.array(P[n][0].bounding_box()) for n in names}
    rows, bad = [], []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            A, B = bbs[a], bbs[b]
            if rigid_body(a) == rigid_body(b) or np.any(A[:3] > B[3:] + 1) or np.any(B[:3] > A[3:] + 1):
                continue
            ov = (P[a][0] ^ P[b][0]).volume()
            g = -ov if ov > 1e-3 else P[a][0].min_gap(P[b][0], 1.0)
            if g < 1.0:
                rows.append((a, b, round(g, 3)))
                if g < MIN_RUN_GAP and frozenset((a, b)) not in CONTACT_OK:
                    bad.append((a, b, round(g, 3)))
    return sorted(rows, key=lambda r: r[2]), bad


def rot_axis(m_, axis, deg, centre):
    """Rotate a manifold by deg about a line through centre (Rodrigues; Euler angles can't do tilted axes)."""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    a = math.radians(deg)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    Rm = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
    c = np.asarray(centre, float)
    return m_.transform(np.hstack([Rm, (c - Rm @ c)[:, None]]))


def mesh_check(P):
    """Gear pairs are allowed to sit close, but must never overlap anywhere over one tooth pitch of rotation."""
    c = np.array([0, 0, AX_Z])
    ax = np.array([MOTOR_X, 0, AX_Z])
    pin, cr = P["motor_pinion"][0], P["diff_case_crown"][0]
    worst = {"pinion_crown": 0.0, "side_spider": 0.0}
    for k in range(12):
        th = k * (360 / PIN_Z) / 12
        p = pin.translate(-ax).rotate([0, th, 0]).translate(ax)
        q = cr.translate(-c).rotate([th * PIN_Z / CROWN_Z, 0, 0]).translate(c)
        worst["pinion_crown"] = max(worst["pinion_crown"], (p ^ q).volume())
    # differential action: right side gear +th, left -th, every spider spins about its own axis (1:1 miters).
    # Each spider's spin direction is taken from its mesh with the right gear, then must also clear the left.
    turn = lambda m_, rot: m_.translate(-c).rotate(rot).translate(c)
    spin = {t: spider_axis(t) for t in "ABC"}
    sR, sL = P["side_gear_R"][0], P["side_gear_L"][0]
    for tag, axis in spin.items():
        sp = P[f"spider_gear_{tag}"][0]
        runs = []
        for sgn in (1, -1):
            w = 0.0
            for k in range(12):
                th = k * (360 / DIFF_Z) / 12
                s_ = rot_axis(sp, axis, sgn * th, c)
                w = max(w, (turn(sR, [th, 0, 0]) ^ s_).volume(), (turn(sL, [-th, 0, 0]) ^ s_).volume())
            runs.append(w)
        worst["side_spider"] = max(worst["side_spider"], min(runs))
    return {k: round(v, 3) for k, v in worst.items()}


def diff_assembly_paths(P):
    """Slide each diff piece in along the axle the way it is assembled; any overlap on the way (mm3) means it
    can't go together. Order matches the viewer's diff steps."""
    g = lambda n: P[n][0]
    unit = g("spider_cross") + g("spider_gear_A") + g("spider_gear_B") + g("spider_gear_C")
    moves = {"side_gear_R into crown half": (g("side_gear_R"), g("diff_case_crown"), -1, 40),
             "cross + spiders into crown half": (unit, g("diff_case_crown") + g("side_gear_R"), -1, 40),
             "side_gear_L into cover": (g("side_gear_L"), g("diff_cover"), 1, 40),
             "cover onto crown half": (g("diff_cover") + g("side_gear_L"), g("diff_case_crown") + g("side_gear_R") + unit, -1, 40)}
    case = g("diff_case_crown") + g("diff_cover") + g("side_gear_R") + g("side_gear_L") + unit
    moves |= {f"half_shaft_{s} into side gear": (g(f"half_shaft_{s}"), case, d, 100) for s, d in (("R", 1), ("L", -1))}
    return {k: round(max((mv.translate([0.5 * i * d, 0, 0]) ^ fx).volume() for i in range(1, int(dist / 0.5) + 1)), 3)
            for k, (mv, fx, d, dist) in moves.items()}


def gear_float(P):
    """How far each diff gear can slide outward along its own axis before it seats (bevel thrust pushes
    them out; more than ~0.5 mm and the teeth ride up onto their tips)."""
    case = P["diff_case_crown"][0] + P["diff_cover"][0]
    axes = {"side_gear_R": [1, 0, 0], "side_gear_L": [-1, 0, 0], **{f"spider_gear_{t}": spider_axis(t) for t in "ABC"}}
    out = {}
    for n, a in axes.items():
        d = 0.0
        while d < 3 and (P[n][0].translate(np.array(a) * (d + 0.05)) ^ case).volume() < 0.01:
            d += 0.05
        out[n] = round(d, 2)
    return out


# strength: loads are capped by tyre grip (the tyres slip before the drivetrain can be loaded harder)
MU_TYRE = 1.0              # TPU on a smooth floor, generous
REAR_LOAD = 0.55           # share of the weight on the driven rear wheels
SHOCK = 2.0                # hard starts / bumps on top of the grip limit
PETG_MPA = {"in_layer": 30.0, "across_layers": 20.0}   # conservative printed-PETG bending strength
N_SPIDERS, SPIDER_SHARE = 3, 1.5                        # printed gears never share load evenly: worst spider takes 1.5x
LEWIS_Y = [(10, .201), (11, .226), (12, .245), (13, .261), (14, .277), (15, .290), (17, .303), (20, .322), (24, .337),
           (30, .359), (40, .389), (60, .422), (100, .447), (300, .472), (1e9, .485)]


def lewis_y(z):
    zs, ys = zip(*LEWIS_Y)
    return float(np.interp(z, zs, ys))


def strength_report(total_mass_g):
    """Lewis tooth bending + simple beam/torsion checks at the grip-limited axle torque x SHOCK.
    Also: the motor stall torque that would break the weakest part if a wheel is physically jammed."""
    T = MU_TYRE * total_mass_g / 1000 * 9.81 * REAR_LOAD * WHEEL_R / 1000 * SHOCK      # N*m at the crown/axle
    rows = {}
    b = CROWN_R_OUT - CROWN_R_IN
    F = T / (CROWN_M * CROWN_Z / 2 / 1000)                                              # N at crown pitch
    rows["pinion teeth (z11, in layer)"] = (F / (b * CROWN_M * lewis_y(PIN_Z)), "in_layer")
    rows["crown teeth (teeth up = across layers)"] = (F / (b * CROWN_M * lewis_y(1e9)), "across_layers")
    rp, face = DIFF_M * DIFF_Z / 2, DIFF_FACE
    rm = rp - face / 2 * math.sin(math.radians(45))                                     # mean pitch radius
    Fb = (T / 2) / N_SPIDERS / (rm / 1000) * SPIDER_SHARE
    rows["side/spider bevel teeth (in layer)"] = (Fb / (face * DIFF_M * rm / rp * lewis_y(DIFF_Z / math.cos(math.radians(45)))), "in_layer")
    Fa = T / N_SPIDERS / (rm / 1000)                                                    # each cross arm, at r = rm
    Mmax = Fa * rm * (CAV_R - rm) / CAV_R                   # by symmetry the hub carries no moment: arm = simply supported
    rows["spider cross arm bending"] = (32 * Mmax / (math.pi * 4.0 ** 3), "in_layer")
    rows["half shaft torsion (7 mm hex, printed upright)"] = (16 * T / 2 * 1000 / (math.pi * 7.0 ** 3) * 2, "across_layers")  # shear x2 ~ tensile
    Tp = T / (CROWN_Z / PIN_Z) * 1000                                                    # N*mm at the motor shaft
    rows["pinion D-flat bearing on motor shaft"] = (Tp / 1.5 / (8 * 2.65), "across_layers")
    out = {k: dict(MPa=round(s, 1), allow=PETG_MPA[w], SF=round(PETG_MPA[w] / s, 1)) for k, (s, w) in rows.items()}
    weakest = min(out, key=lambda k: out[k]["SF"])
    T_break = T * out[weakest]["SF"]                                                    # axle torque at SF = 1
    return dict(load_case=f"grip-limited axle torque x{SHOCK} = {T:.2f} N*m", parts=out, weakest=weakest,
                min_SF=out[weakest]["SF"],
                jammed_wheel_motor_stall_limit_kgcm=round(T_break / (CROWN_Z / PIN_Z) * 10.197, 1))


def bed_fit(P):
    """Printed parts whose two longest sides don't fit the bed (any part can be laid on its smallest side)."""
    out = {}
    for n, (m, c, pr, g) in P.items():
        if pr:
            bb = np.array(m.bounding_box())
            d = sorted(bb[3:] - bb[:3])
            if d[1] > BED or d[2] > BED:
                out[n] = [round(v, 1) for v in d]
    return out


def fit_coupon():
    """Print this first (PETG, same settings as the real parts): every fit in the robot in one 70 x 22 bar.
    Holes, left to right: snap-pin hole, 8 mm wheel/stub bore, 12 mm diff collar bore, 7 mm hex (rear wheel),
    4 mm D-bore (motor shaft). Pegs: MG996R 25T and SG90 21T splines. Tune FIT / SPLINE_FIT from it."""
    bar = box(0, 70, 0, 22, 0, 6)
    holes = [cyl(PIN_HOLE, -1, 7, 7, 11), cyl(4 + FIT, -1, 7, 18, 11), cyl(6 + FIT, -1, 7, 33, 11),
             hexz(7.0 + 2 * FIT, -1, 7, 48, 11), cyl(2 + 0.05, -1, 7, 61, 11) - box(62.5, 66, 6, 16, -2, 8)]
    bar -= union(holes)
    hubs = [cyl(6.5, 0, 6, 15, 32) - spline_bore(25, 3.0, 2.65, 1.4, 6.01, 15, 32),
            cyl(5.5, 0, 6, 35, 32) - spline_bore(21, 2.45, 2.15, 2.4, 6.01, 35, 32)]
    # male gauges for the round/hex holes, standing on one bar: 4 mm pin, 8 mm stub, 12 mm collar, 7 mm hex
    gauge = (box(0, 56, 41, 55, 0, 4) + cyl(PIN_R, 4, 14, 7, 48) + cyl(4, 4, 14, 18, 48) + cyl(6, 4, 14, 33, 48)
             + hexz(7.0, 4, 14, 48, 48))
    return bar + union(hubs) + gauge


def main():
    os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
    for f in os.listdir(os.path.join(OUT, "stl")):          # drop STLs of parts that no longer exist
        os.remove(os.path.join(OUT, "stl", f))
    P = build()
    rep = {}
    lo, hi = np.full(3, 1e9), np.full(3, -1e9)
    sweep = []
    for st in (-HORN_MAX, -20, -10, 0, 10, 20, HORN_MAX):
        Q = build(st) if st else P
        for n, (m, *_r) in Q.items():
            bb = np.array(m.bounding_box())
            lo, hi = np.minimum(lo, bb[:3]), np.maximum(hi, bb[3:])
        if st:
            sweep += [(st, a, b, v) for a, b, v in interference(Q) if a.startswith(MOVING) or b.startswith(MOVING)]
    rep["envelope_mm"] = dict(L=round(hi[1] - lo[1], 1), W=round(hi[0] - lo[0], 1), H=round(hi[2] - lo[2], 1))
    rep["lidar_scan_plane_z"] = LIDAR_SCAN
    rep["scrub_radius_mm"] = FRONT_X - KP_X
    rep["ground_clearance_min"] = round(min(np.array(m.bounding_box())[2] for n, (m, c, pr, g) in P.items()
                                            if not n.startswith(("tire", "wheel"))), 1)
    pm = {n: printed_grams(m) for n, (m, c, pr, g) in P.items() if pr}
    rep["printed_mass_g"] = round(sum(pm.values()))
    rep["printed_mass_v1_g"] = V1_PRINTED_G
    rep["bought_mass_g"] = sum(BOUGHT_MASS.values())
    rep["total_mass_g"] = rep["printed_mass_g"] + rep["bought_mass_g"]
    rep["drive_ratio"] = f"{CROWN_Z}:{PIN_Z} = {CROWN_Z / PIN_Z:.2f}"
    rep["ackermann"] = ackermann_report()
    lock = steer_solution(HORN_MAX)
    rep["lock_deg_inner_outer"] = [round(max(map(abs, lock)), 1), round(min(map(abs, lock)), 1)]
    rep["min_turn_radius_mm"] = round(WB / math.tan(math.radians(min(map(abs, lock)))) + FRONT_X)
    rep["fasteners"] = fastener_bom()
    rep["interference_static"] = interference(P)
    rep["interference_steering"] = sweep
    rows, bad = running_clearance(P)
    rep["running_clearance"] = dict(min_gap_mm=MIN_RUN_GAP, FIT=FIT, tight=bad, closest=rows[:40])
    rep["gear_mesh_overlap_mm3"] = mesh_check(P)
    rep["gear_float_mm"] = gear_float(P)
    rep["diff_assembly_overlap_mm3"] = diff_assembly_paths(P)
    rep["strength"] = strength_report(rep["total_mass_g"])
    rep["bed"] = dict(limit_mm=BED, too_big=bed_fit(P),
                      largest={n: v for n, v in sorted(((n, sorted(np.round(np.array(m.bounding_box())[3:]
                               - np.array(m.bounding_box())[:3], 1))) for n, (m, c, pr, g) in P.items() if pr),
                               key=lambda r: -r[1][2])[:6]})
    rep["parts_printed_g"] = pm
    with open(os.path.join(OUT, "report.json"), "w") as f:
        json.dump(rep, f, indent=1)
    scene = trimesh.Scene()
    for n, (m, c, pr, g) in P.items():
        t = to_trimesh(m, c)
        if pr:
            t.export(os.path.join(OUT, "stl", f"{n}.stl"))
        scene.add_geometry(t, node_name=f"{g}__{'P' if pr else 'B'}__{n}", geom_name=n)
    scene.export(os.path.join(OUT, "robot.glb"))
    to_trimesh(fit_coupon(), PC["pin"]).export(os.path.join(OUT, "stl", "00_FIT_TEST_COUPON_print_first.stl"))
    print(json.dumps({k: v for k, v in rep.items() if k not in ("parts_printed_g", "ackermann")}, indent=1))
    e = rep["envelope_mm"]
    assert max(e.values()) <= 300, "outside the 30 cm cube"
    assert rep["total_mass_g"] <= 1500, "over the 1.5 kg limit"
    assert LIDAR_SCAN < 70, "scan plane must sit well under the 100 mm walls/obstacles"
    assert all(abs(r["err"]) < 3 for r in rep["ackermann"]), "Ackermann error > 3 deg"
    assert not rep["interference_static"] and not sweep, "parts collide"
    assert not bad, f"running clearance under {MIN_RUN_GAP} mm: {bad}"
    assert max(rep["gear_mesh_overlap_mm3"].values()) < 0.01, "gear teeth overlap during rotation"
    assert max(rep["gear_float_mm"].values()) <= 0.5, f"a diff gear can slide out of mesh: {rep['gear_float_mm']}"
    assert max(rep["diff_assembly_overlap_mm3"].values()) < 0.01, f"diff can't be assembled: {rep['diff_assembly_overlap_mm3']}"
    assert rep["strength"]["min_SF"] >= 2.0, f"weak part: {rep['strength']['weakest']}"
    assert not rep["bed"]["too_big"], f"won't fit the P2S bed: {rep['bed']['too_big']}"


if __name__ == "__main__":
    main()
