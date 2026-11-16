"""
boxing_robot_chassis.py  -  parametric FDM chassis for a two-armed boxing robot.

Blender 4.x. Run from a terminal, in the folder you want the STL files in:
    blender --background --python boxing_robot_chassis.py

Output:
  ./stl/<part>.stl         one watertight STL per printable part (print-oriented, base at z=0)
  collection 'assembly_preview'  all parts in assembled position, plus purchased parts
                                 (motors, wheels, gears, rod) - NOT exported.
All units are millimetres. Parts are built in assembled position, then exported.
"""

import bpy, bmesh, math, os
from mathutils import Matrix

# =====================================================================
# PARAMETERS
# =====================================================================

# --- Printer (FDM, PLA) ------------------------------------------------
PRINTER_X = 220            # build volume X
PRINTER_Y = 220            # build volume Y
PRINTER_Z = 250            # build volume Z
STL_DIR   = os.path.join(os.getcwd(), "stl")

# --- General print rules -----------------------------------------------
WALL_T      = 3.0          # structural wall thickness
WALL_MIN    = 2.0          # minimum (non-structural) wall
FEATURE_MIN = 1.2          # minimum printable feature
FILLET_R    = 1.0          # outer edge fillet
ELEPHANT_CH = 0.5          # bottom-edge chamfer against elephant foot
TORSO_CORNER = 4.0         # torso vertical corner radius

# --- Fits ------------------------------------------------------------------
FIT_LOOSE = 0.4            # loose clearance (pockets for bodies)
FIT_SLIDE = 0.25           # sliding fit (axle in bushing)
FIT_PRESS = 0.1            # press fit (bushing in block, collar on rod)

# --- Fasteners ---------------------------------------------------------------
M3_HOLE     = 3.2
M2_HOLE     = 2.2
INSERT_HOLE = 4.0          # heat-set insert hole
NUT_AF      = 5.7          # M3 nut across flats
NUT_DEPTH   = 2.6

# --- Electronics (MEASURE: values marked are guesses) ------------------------
BOARD_L = 70               # ESP32-S3 dev board, long side (along Y in this design)
BOARD_W = 28               # short side (along X); USB-C ports sit on the rear short edge
BOARD_H = 15               # incl. headers (MEASURE)
BOARD_HOLE_D  = 3.2        # board mounting hole (MEASURE)
BOARD_HOLE_DX = 12         # hole pattern half-spacing X (MEASURE)
BOARD_HOLE_DY = 31         # hole pattern half-spacing Y (MEASURE)
BOARD_STANDOFF_H = 6       # standoff height above shelf 2
PERF_L = 70                # perfboard (along Y)
PERF_W = 50                # perfboard (along X)
PERF_T = 1.6               # perfboard thickness (MEASURE)
PERF_HOLE_D = 2.0          # perfboard mounting hole (MEASURE)
PERF_HOLE_INSET = 3.0      # hole inset from board corner (MEASURE)
BATT_L = 62                # 4xAA holder, along X
BATT_W = 58                # 4xAA holder, along Y
BATT_H = 16                # 4xAA holder height (MEASURE)
USB_W = 10                 # opening in back panel for USB-C
USB_H = 6
SHELF_T = WALL_T           # internal shelves thickness
SHELF1_OFFSET = 4          # perfboard shelf: distance above battery top
SHELF2_OFFSET = 35         # ESP32 shelf: distance above perfboard shelf

# --- DC motors (small brushed, round can) -----------------------------------
MOTOR_D    = 24.4
MOTOR_L    = 25
SHAFT_D    = 2.0
SHAFT_L    = 10
CLAMP_WALL = 2.5           # wall of clamp halves around motor
CLAMP_LUG_W = 6            # lug width for M3 screws
CLAMP_LUG_T = WALL_MIN

# --- SG90 micro servo (datasheet values) -----------------------------------
SERVO_BODY_L   = 22.5
SERVO_BODY_W   = 12.2
SERVO_BODY_H   = 22.7
SERVO_TAB_W    = 32.2
SERVO_TAB_HOLE = 27.5
SERVO_TAB_T    = 2.5
SERVO_SHAFT_OFF = 5.9
SERVO_HORN_D   = 8.0       # MEASURE

# --- Axle, wheels, gears, buttons (purchased kit) ----------------------------
ROD_D      = 3.0           # steel axle (MEASURE)
ROD_MAX_L  = 100           # longest axle available (MEASURE)
WHEEL_D    = 40
WHEEL_W    = 8
WHEEL_BORE = 3.0
GEAR_SMALL_TEETH = 10
GEAR_LARGE_TEETH = 30
GEAR_MODULE      = 0.5
GEAR_BORE_SMALL  = 2.0
GEAR_BORE_LARGE  = 3.0
GEAR_THICKNESS   = 4.0
GEAR_BACKLASH    = 0.1
CENTRE_DIST = GEAR_MODULE * (GEAR_SMALL_TEETH + GEAR_LARGE_TEETH) / 2 + GEAR_BACKLASH
BTN_SIZE = 6.0             # push button body (MEASURE)
BTN_H    = 5.0

# --- Base and drive ----------------------------------------------------------
BASE_W = 76                # base plate width (X)
BASE_D = 110               # base plate depth (Y)
BASE_T = 4                 # base plate thickness
BASE_Z0 = 6                # plate underside height above ground (clears the skid)
AXLE_Z = WHEEL_D / 2       # axle height above ground
WHEEL_OFF = BASE_W / 2 + WHEEL_W / 2   # wheel centre X
AXLE_GEAR_X = WHEEL_OFF + WHEEL_W / 2  # axle gear sits just outside the wheel
BLOCK_X = 22               # bearing block centre X
BLOCK_W = 16               # bearing block length along X (bushing length)
BLOCK_Y = 16               # bearing block width along Y
BLOCK_TOP = AXLE_Z + 8     # bearing block top
BUSHING_OD = 7.0
BUSHING_ID = ROD_D + FIT_SLIDE * 2
BORE_D = BUSHING_OD + FIT_PRESS
COLLAR_OD = 10
COLLAR_L = 6
COLLAR_X = BLOCK_X + BLOCK_W / 2 + COLLAR_L / 2 + 1   # collar centre X
COLLAR_GRUB = M3_HOLE
POST_W = 6                 # torso support posts
POST_X = 28
POST_Y = 30
SKID_R = 7
SKID_T = BASE_Z0
SKID_Y = 45

# --- Torso and head ----------------------------------------------------------
TORSO_W = 70
TORSO_D = 80
TORSO_H = 130
TORSO_Z0 = 32              # torso underside above ground (sits on the base posts)
TORSO_BOT = TORSO_Z0
TORSO_TOP = TORSO_Z0 + TORSO_H
SHELF1_Z = TORSO_BOT + WALL_T + BATT_H + SHELF1_OFFSET
SHELF2_Z = SHELF1_Z + SHELF2_OFFSET
CHEST_BTN_Z = TORSO_BOT + 80
CHEST_BTN_X = 14
HEAD_R = 32
HEAD_PLATE_T = 3
EYE_R = 6
ANTENNA_D = 2.0

# --- Arms --------------------------------------------------------------------
ARM_X = TORSO_W / 2 + 40   # |X| of each arm
SHOULDER_Z = TORSO_BOT + 120
HOUSING_R = 14
HOUSING_L = 28
UPPER_ARM_L = 50
UPPER_ARM_W = 20
UPPER_ARM_D = 12
FOREARM_L = 46
FOREARM_W = 20
FOREARM_D = 12
FIST_R = 16
ROD_OFF = 4                # right-arm mechanism sits this far in +X from the arm centre
CRANK_DISC_R = 10
CRANK_DISC_T = 4
CRANK_R = 6                # pin offset from axle (<= disc radius)
CRANK_PIN_D = 3.0
ROD_LEN = 62
ROD_W = 4
ROD_T = 6
LEVER_ARM = 4              # rod-to-elbow offset
ELBOW_Z = SHOULDER_Z - CRANK_R - ROD_LEN - LEVER_ARM
DRIVE_MOTOR_Z = AXLE_Z + CENTRE_DIST
PUNCH_MOTOR_X = TORSO_W / 2 + 1 + MOTOR_L / 2   # motor body starts 1 mm off torso wall
MOTOR_BOX_MARGIN = 1

# =====================================================================
# SMALL HELPERS
# =====================================================================

PREVIEW = None
PRINT_LIST = []


def coll(name):
    c = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(c)
    return c


def reset_scene():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    for m in list(bpy.data.meshes):
        bpy.data.meshes.remove(m)
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)


def select_only(o):
    """Blender 3.0-compatible: make o the only selected and active object."""
    bpy.ops.object.mode_set(mode='OBJECT') if bpy.context.object and bpy.context.object.mode != 'OBJECT' else None
    bpy.ops.object.select_all(action='DESELECT')
    o.select_set(True)
    bpy.context.view_layer.objects.active = o


def bake(o):
    """Apply location/rotation/scale so the mesh lives in global coordinates."""
    select_only(o)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    return o


def apply_mod(o, m):
    select_only(o)
    bpy.ops.object.modifier_apply(modifier=m.name)


def box(size, loc):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.active_object
    o.scale = size
    return bake(o)


def rbox(size, loc, r=FILLET_R):
    o = box(size, loc)
    m = o.modifiers.new("fillet", 'BEVEL')
    m.width = min(r, min(size) / 2 - 0.05)
    m.segments = 3
    apply_mod(o, m)
    return o


def cyl(r, h, loc, axis='Z', seg=48):
    rot = {'Z': (0, 0, 0), 'X': (0, math.pi / 2, 0), 'Y': (math.pi / 2, 0, 0)}[axis]
    bpy.ops.mesh.primitive_cylinder_add(vertices=seg, radius=r, depth=h, location=loc, rotation=rot)
    return bake(bpy.context.active_object)


def sph(r, loc):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=r, location=loc, segments=48, ring_count=24)
    return bake(bpy.context.active_object)


def bool_op(target, cutter, op):
    m = target.modifiers.new("bool", 'BOOLEAN')
    m.object = cutter
    m.operation = op
    m.solver = 'EXACT'
    apply_mod(target, m)
    bpy.data.objects.remove(cutter, do_unlink=True)
    return target


def cut(target, *cutters):
    for c in cutters:
        bool_op(target, c, 'DIFFERENCE')
    return target


def union(target, *others):
    for o in others:
        bool_op(target, o, 'UNION')
    return target


def finalize(o, name, print_rot=None):
    """Move object into the preview collection; register it for export if printable."""
    o.name = name
    for c in o.users_collection:
        c.objects.unlink(o)
    PREVIEW.objects.link(o)
    if print_rot is not None:
        PRINT_LIST.append((name, o, print_rot))
    return o


def prism(pts, h, name):
    """Extrude a closed 2D polygon (list of (x,y)) from z=0 to z=h."""
    n = len(pts)
    verts = [(x, y, 0.0) for x, y in pts] + [(x, y, h) for x, y in pts]
    faces = [list(range(n))[::-1], list(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    me = bpy.data.meshes.new(name + "_mesh")
    me.from_pydata(verts, [], faces)
    me.update()
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    return o


def gear_pts(teeth, module):
    """Simple spur-gear outline (trapezoidal teeth). Good enough for printing/preview."""
    rp = module * teeth / 2
    ra = rp + module
    rd = rp - 1.25 * module
    pitch = 2 * math.pi / teeth
    pts = []
    for k in range(teeth):
        a0 = k * pitch
        for frac, r in ((0.0, rd), (0.15, ra), (0.35, ra), (0.5, rd)):
            a = a0 + frac * pitch
            pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def gear_on_x(name, teeth, bore, x, y, z):
    """Gear whose axis is along X, centred at (x, y, z) with thickness GEAR_THICKNESS."""
    o = prism(gear_pts(teeth, GEAR_MODULE), GEAR_THICKNESS, name)
    o.rotation_euler = (0, math.pi / 2, 0)
    o.location = (x - GEAR_THICKNESS / 2, y, z)
    bake(o)
    cut(o, cyl(bore / 2, GEAR_THICKNESS + 2, (x, y, z), 'X'))
    return o


def check_mesh(name, me):
    bm = bmesh.new()
    bm.from_mesh(me)
    bad = sum(1 for e in bm.edges if len(e.link_faces) != 2)
    bm.free()
    if bad:
        print(f"WARNING {name}: {bad} edges are not shared by 2 faces (not watertight)")


def repair_mesh(me):
    """Fast-solver booleans can leave doubled verts and flipped faces; clean them up."""
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=0.0005)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    open_edges = [e for e in bm.edges if len(e.link_faces) == 1]   # open boundary loops
    if open_edges:
        bmesh.ops.holes_fill(bm, edges=open_edges, sides=0)
    bm.to_mesh(me)
    bm.free()


def elephant_foot(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    zmin = min(v.co.z for v in bm.verts)
    edges = [e for e in bm.edges if all(abs(v.co.z - zmin) < 1e-4 for v in e.verts)]
    if edges:
        bmesh.ops.bevel(bm, geom=edges, offset=ELEPHANT_CH, offset_type='OFFSET',
                        segments=1, affect='EDGES')
    bm.to_mesh(me)
    bm.free()


# =====================================================================
# PRINTABLE PARTS (built in assembled position)
# =====================================================================

def build_base():
    plate = rbox((BASE_W, BASE_D, BASE_T), (0, 0, BASE_Z0 + BASE_T / 2))
    posts = []
    post_h = TORSO_BOT - (BASE_Z0 + BASE_T)
    for sx in (-1, 1):
        for sy in (-1, 1):
            posts.append(box((POST_W, POST_W, post_h),
                             (sx * POST_X, sy * POST_Y, BASE_Z0 + BASE_T + post_h / 2)))
    blocks = []
    for sx in (-1, 1):
        blocks.append(box((BLOCK_W, BLOCK_Y, BLOCK_TOP - BASE_Z0),
                          (sx * BLOCK_X, 0, (BASE_Z0 + BLOCK_TOP) / 2)))
    union(plate, *posts, *blocks)
    holes = [cyl(INSERT_HOLE / 2, 60, (sx * POST_X, sy * POST_Y, TORSO_BOT - 10))
             for sx in (-1, 1) for sy in (-1, 1)]
    bores = cyl(BORE_D / 2, BASE_W + 20, (0, 0, AXLE_Z), 'X')
    cut(plate, *holes, bores)
    return finalize(plate, "base_plate", print_rot=(0, 0, 0))


def build_skid():
    s = cyl(SKID_R, SKID_T, (0, SKID_Y, SKID_T / 2))
    boss = box((14, 14, BASE_Z0 - SKID_T + 4), (0, SKID_Y, (SKID_T + BASE_Z0 + 4) / 2))
    union(s, boss)
    return finalize(s, "skid_rear", print_rot=(0, 0, 0))


def build_bushing(side):
    x = side * BLOCK_X
    b = cyl(BUSHING_OD / 2, BLOCK_W, (x, 0, AXLE_Z), 'X')
    cut(b, cyl(BUSHING_ID / 2, BLOCK_W + 2, (x, 0, AXLE_Z), 'X'))
    return finalize(b, f"axle_bushing_{'L' if side < 0 else 'R'}", print_rot=(0, 90, 0))


def build_collar(side):
    x = side * COLLAR_X
    c = cyl(COLLAR_OD / 2, COLLAR_L, (x, 0, AXLE_Z), 'X')
    cut(c, cyl((ROD_D + FIT_PRESS) / 2, COLLAR_L + 2, (x, 0, AXLE_Z), 'X'))
    cut(c, cyl(COLLAR_GRUB / 2, COLLAR_OD + 2, (x, 0, AXLE_Z), 'Y'))
    return finalize(c, f"shaft_collar_{'L' if side < 0 else 'R'}", print_rot=(0, 90, 0))





def build_torso():
    o = rbox((TORSO_W, TORSO_D, TORSO_H), (0, 0, TORSO_BOT + TORSO_H / 2), r=TORSO_CORNER)
    inner = box((TORSO_W - 2 * WALL_T, TORSO_D - 2 * WALL_T, TORSO_H - 2 * WALL_T),
                (0, 0, TORSO_BOT + TORSO_H / 2))
    rear = box((TORSO_W - 2 * WALL_T, WALL_T + 2, TORSO_H - 2 * WALL_T),
               (0, TORSO_D / 2 - WALL_T / 2, TORSO_BOT + TORSO_H / 2))
    cutters = [inner, rear]
    # chest button holes (through front wall) and wire channels beside them
    for sx in (-1, 1):
        cutters.append(box((BTN_SIZE + FIT_SLIDE, WALL_T + 4, BTN_SIZE + FIT_SLIDE),
                           (sx * CHEST_BTN_X, -TORSO_D / 2, CHEST_BTN_Z)))
        cutters.append(box((4, 2, 30),
                           (sx * (CHEST_BTN_X + 5), -TORSO_D / 2 + WALL_T, CHEST_BTN_Z - 15)))
    cut(o, *cutters)
    shelf_w = TORSO_W - 2 * WALL_T + 1
    shelf_d = TORSO_D - 2 * WALL_T + 1
    shelves = [box((shelf_w, shelf_d, SHELF_T), (0, 0, z + SHELF_T / 2))
               for z in (SHELF1_Z, SHELF2_Z)]
    esp_posts = [box((POST_W, POST_W, BOARD_STANDOFF_H + SHELF_T),
                     (sx * BOARD_HOLE_DX, sy * BOARD_HOLE_DY, SHELF2_Z + SHELF_T + BOARD_STANDOFF_H / 2 - SHELF_T / 2))
                 for sx in (-1, 1) for sy in (-1, 1)]
    union(o, *shelves, *esp_posts)
    holes = [cyl(INSERT_HOLE / 2, 20, (sx * BOARD_HOLE_DX, sy * BOARD_HOLE_DY,
                                       SHELF2_Z + SHELF_T + BOARD_STANDOFF_H), 'Z')
             for sx in (-1, 1) for sy in (-1, 1)]
    cut(o, *holes)
    return finalize(o, "torso_shell", print_rot=(0, 0, 0))


def build_back_panel():
    panel_w = TORSO_W - 2 * WALL_T - FIT_LOOSE
    panel_h = TORSO_H - 2 * WALL_T - FIT_LOOSE
    t = WALL_MIN + FIT_LOOSE
    p = box((panel_w, t, panel_h), (0, TORSO_D / 2 - t / 2 - 0.2, TORSO_BOT + TORSO_H / 2))
    usb_z = SHELF2_Z + SHELF_T + BOARD_STANDOFF_H + 4
    cutters = [box((USB_W, t + 4, USB_H), (0, TORSO_D / 2, usb_z))]
    for sx in (-1, 1):
        for z in (TORSO_BOT + 8, TORSO_TOP - 8):
            cutters.append(cyl(M3_HOLE / 2, t + 4, (sx * (panel_w / 2 - 6), TORSO_D / 2, z), 'Y'))
    cut(p, *cutters)
    return finalize(p, "torso_back_panel", print_rot=(90, 0, 0))


def build_motor_box():
    cx = PUNCH_MOTOR_X
    o = rbox((MOTOR_L + 2 * MOTOR_BOX_MARGIN, MOTOR_D + 2 * CLAMP_WALL + 4,
              MOTOR_D + 2 * CLAMP_WALL + 4), (cx, 0, SHOULDER_Z))
    cut(o, cyl(MOTOR_D / 2 + FIT_LOOSE / 2, MOTOR_L + 2 * MOTOR_BOX_MARGIN + 2, (cx, 0, SHOULDER_Z), 'X'))
    return finalize(o, "motor_box_R", print_rot=(0, 0, 0))


def clamp_half(name, cx, cy, cz, half):
    """Half-tube clamp around a motor with axis along X. Split plane = local x=0 (horizontal
    after rotation). half 'A' keeps x<=0, half 'B' keeps x>=0. Tabs carry M3 screws (vertical)."""
    ro = MOTOR_D / 2 + CLAMP_WALL
    ri = MOTOR_D / 2 + FIT_LOOSE / 2
    t = cyl(ro, MOTOR_L, (0, 0, 0), 'Z')
    cut(t, cyl(ri, MOTOR_L + 2, (0, 0, 0), 'Z'))
    for sy in (-1, 1):
        ty = sy * (ro + CLAMP_LUG_W / 2 - 1)
        union(t, box((CLAMP_LUG_T, CLAMP_LUG_W, MOTOR_L), (0, ty, 0)))
        cut(t, cyl(M3_HOLE / 2, MOTOR_L + 4, (0, ty, 0), 'X'))
    off = 2 * ro if half == 'A' else -2 * ro
    cut(t, box((4 * ro, 4 * ro, MOTOR_L + 4), (off, 0, 0)))
    t.rotation_euler = (0, math.pi / 2, 0)
    t.location = (cx, cy, cz)
    bake(t)
    return finalize(t, name, print_rot=(0, 90, 0))


def build_housing(side):
    ax = side * ARM_X
    h = cyl(HOUSING_R, HOUSING_L, (ax, 0, SHOULDER_Z), 'X')
    if side > 0:
        cut(h, cyl(CRANK_DISC_R + FIT_LOOSE, 8, (ax, 0, SHOULDER_Z), 'X'))  # crank cavity
        cut(h, cyl(ROD_D / 2 + FIT_SLIDE, HOUSING_L + 2, (ax, 0, SHOULDER_Z), 'X'))
        name = "shoulder_housing_R"
    else:
        cut(h, box((SERVO_BODY_H + FIT_LOOSE, SERVO_BODY_L + FIT_LOOSE, SERVO_BODY_W + FIT_LOOSE),
                   (ax, 0, SHOULDER_Z)))  # servo pocket
        cut(h, cyl(M2_HOLE / 2 + 0.6, HOUSING_L + 2, (ax, 0, SHOULDER_Z), 'X'))
        name = "shoulder_housing_L"
    return finalize(h, name, print_rot=(0, 90, 0))


def build_crank():
    ax = ARM_X
    d = cyl(CRANK_DISC_R, CRANK_DISC_T, (ax, 0, SHOULDER_Z), 'X')
    pin = cyl(CRANK_PIN_D / 2, 4, (ax + CRANK_DISC_T / 2 + 2, 0, SHOULDER_Z - CRANK_R), 'X')
    cut(d, cyl(ROD_D / 2 + FIT_PRESS, CRANK_DISC_T + 2, (ax, 0, SHOULDER_Z), 'X'))
    union(d, pin)
    return finalize(d, "crank_disc_R", print_rot=(0, -90, 0))


def build_con_rod():
    mx = ARM_X + ROD_OFF
    zt = SHOULDER_Z - CRANK_R
    zb = ELBOW_Z + LEVER_ARM
    r = box((ROD_W, ROD_T, zt - zb), (mx, 0, (zt + zb) / 2))
    cut(r, cyl(CRANK_PIN_D / 2 + FIT_SLIDE, ROD_W + 2, (mx, 0, zt), 'X'),
        cyl(CRANK_PIN_D / 2 + FIT_SLIDE, ROD_W + 2, (mx, 0, zb), 'X'))
    return finalize(r, "con_rod_R", print_rot=(90, 0, 0))


def build_upper_arm(side):
    ax = side * ARM_X
    top = SHOULDER_Z - HOUSING_R
    bot = top - UPPER_ARM_L
    a = rbox((UPPER_ARM_W, UPPER_ARM_D, UPPER_ARM_L), (ax, 0, (top + bot) / 2))
    if side > 0:
        mx = ax + ROD_OFF
        cut(a, box((ROD_W + 2 * FIT_LOOSE, ROD_T + 2 * FIT_LOOSE, UPPER_ARM_L + 4),
                   (mx, 0, (top + bot) / 2)))   # channel for the connecting rod
        name = "upper_arm_R"
    else:
        cut(a, cyl(SERVO_HORN_D / 2 + FIT_LOOSE / 2, 3, (ax, 0, top), 'Z'))  # horn recess
        name = "upper_arm_L"
    return finalize(a, name, print_rot=(0, 90, 0))


def build_elbow_lever():
    mx = ARM_X + ROD_OFF
    lv = box((ROD_W + 4, 10, 8), (mx, 0, ELBOW_Z))
    union(lv, cyl(CRANK_PIN_D / 2, 8, (mx, 0, ELBOW_Z + LEVER_ARM), 'X'))
    cut(lv, cyl(GEAR_BORE_SMALL / 2 + FIT_PRESS, 14, (mx, 0, ELBOW_Z), 'X'))
    return finalize(lv, "elbow_drive_lever_R", print_rot=(0, 0, 0))


def build_forearm(side):
    mx = side * ARM_X + (ROD_OFF if side > 0 else 0)
    top = ELBOW_Z
    f = rbox((FOREARM_W, FOREARM_D, FOREARM_L), (mx, -CENTRE_DIST, top - FOREARM_L / 2))
    cut(f, cyl(GEAR_BORE_LARGE / 2 + FIT_PRESS, FOREARM_D + 4, (mx, -CENTRE_DIST, top), 'X'))
    name = f"forearm_{'R' if side > 0 else 'L'}"
    return finalize(f, name, print_rot=(0, 90, 0))


def build_fist(side):
    mx = side * ARM_X + (ROD_OFF if side > 0 else 0)
    cz = ELBOW_Z - FOREARM_L - FIST_R + 4          # 4 mm overlap into forearm
    f = sph(FIST_R, (mx, -CENTRE_DIST, cz))
    cut_h = FIST_R + 2
    flat_z = cz + FIST_R - 4                       # flat top sits on forearm bottom
    cut(f, box((2 * FIST_R + 4, 2 * FIST_R + 4, cut_h), (mx, -CENTRE_DIST, flat_z + cut_h / 2)))
    return finalize(f, f"fist_{'R' if side > 0 else 'L'}", print_rot=(180, 0, 0))


# =====================================================================
# BUTTON CAPS (hit pads)
# =====================================================================

def build_cap(name, rot, loc):
    pad = rbox((16, 16, 2), (0, 0, 9), r=0.5)
    stem = box((BTN_SIZE, BTN_SIZE, 10), (0, 0, 5))
    union(pad, stem)
    pad.rotation_euler = rot
    pad.location = loc
    bake(pad)
    return finalize(pad, name, print_rot=(90, 0, 0))


# =====================================================================
# HEAD
# =====================================================================

def build_head():
    zc = TORSO_TOP + HEAD_PLATE_T                  # dome centre = top of head plate
    dome = sph(HEAD_R, (0, 0, zc))
    cut(dome, box((2 * HEAD_R + 10, 2 * HEAD_R + 10, HEAD_R + 10), (0, 0, zc - (HEAD_R + 10) / 2)))
    cut(dome, sph(HEAD_R - WALL_T, (0, 0, zc)))                       # hollow shell, open at bottom
    cut(dome, *[sph(EYE_R, (sx * 12, -HEAD_R * 0.75, zc + HEAD_R * 0.35)) for sx in (-1, 1)])
    cut(dome, cyl(ANTENNA_D / 2, 30, (0, 0, zc + HEAD_R), 'Z'))       # antenna wire hole
    for sx in (-1, 1):                                                # side button holes
        cut(dome, box((10, BTN_SIZE + FIT_SLIDE, BTN_SIZE + FIT_SLIDE), (sx * HEAD_R, 0, zc + HEAD_R * 0.2)))
    plate = rbox((TORSO_W, TORSO_D, HEAD_PLATE_T), (0, 0, TORSO_TOP + HEAD_PLATE_T / 2), r=1.0)
    cut(plate, cyl(HEAD_R - WALL_T, HEAD_PLATE_T + 2, (0, 0, TORSO_TOP + HEAD_PLATE_T / 2), 'Z'))
    for sx in (-1, 1):
        for sy in (-1, 1):
            cut(plate, cyl(M3_HOLE / 2, HEAD_PLATE_T + 4,
                           (sx * (TORSO_W / 2 - 9), sy * (TORSO_D / 2 - 9), TORSO_TOP + HEAD_PLATE_T / 2), 'Z'))
    union(plate, dome)
    return finalize(plate, "head_dome", print_rot=(180, 0, 0))


# =====================================================================
# PURCHASED PARTS - preview only (not exported)
# =====================================================================

def build_purchased():
    """Purchased parts: preview only, never exported."""
    x0 = -(WHEEL_OFF + WHEEL_W / 2)
    x1 = AXLE_GEAR_X + GEAR_THICKNESS
    if (x1 - x0) > ROD_MAX_L:
        print(f"WARNING: axle needs {x1 - x0:.1f} mm, longest rod is {ROD_MAX_L} mm")
    finalize(cyl(ROD_D / 2, x1 - x0, ((x0 + x1) / 2, 0, AXLE_Z), 'X'), "bought_axle_rod")
    for sx in (-1, 1):
        finalize(cyl(WHEEL_D / 2, WHEEL_W, (sx * WHEEL_OFF, 0, AXLE_Z), 'X'),
                 f"bought_wheel_{'L' if sx < 0 else 'R'}")
    gx = AXLE_GEAR_X + GEAR_THICKNESS / 2                      # gear centre X
    finalize(gear_on_x("bought_axle_gear", GEAR_LARGE_TEETH, GEAR_BORE_LARGE, gx, 0, AXLE_Z),
             "bought_axle_gear")
    finalize(gear_on_x("bought_motor_pinion", GEAR_SMALL_TEETH, GEAR_BORE_SMALL, gx, 0, DRIVE_MOTOR_Z),
             "bought_motor_pinion")
    dmx = gx + GEAR_THICKNESS / 2 + 4 + MOTOR_L / 2           # drive motor body centre X
    finalize(cyl(MOTOR_D / 2, MOTOR_L, (dmx, 0, DRIVE_MOTOR_Z), 'X'), "bought_drive_motor")
    finalize(cyl(SHAFT_D / 2, SHAFT_L, (gx, 0, DRIVE_MOTOR_Z), 'X'), "bought_drive_shaft")
    finalize(cyl(MOTOR_D / 2, MOTOR_L, (PUNCH_MOTOR_X, 0, SHOULDER_Z), 'X'), "bought_punch_motor")
    mx = ARM_X + ROD_OFF
    finalize(gear_on_x("bought_elbow_gear_small", GEAR_SMALL_TEETH, GEAR_BORE_SMALL, mx, 0, ELBOW_Z),
             "bought_elbow_gear_small")
    finalize(gear_on_x("bought_elbow_gear_large", GEAR_LARGE_TEETH, GEAR_BORE_LARGE,
                       mx, -CENTRE_DIST, ELBOW_Z), "bought_elbow_gear_large")
    finalize(box((SERVO_BODY_H, SERVO_BODY_L, SERVO_BODY_W), (-ARM_X, 0, SHOULDER_Z)), "bought_servo_L")


# =====================================================================
# BUILD ALL + EXPORT
# =====================================================================

def build_all():
    build_base()
    build_skid()
    for s in (-1, 1):
        build_bushing(s)
        build_collar(s)
    build_torso()
    build_back_panel()
    build_head()
    # caps: 2 on the chest (outward = -Y), 2 on the head sides (outward = +/-X)
    for sx in (-1, 1):
        side = 'L' if sx < 0 else 'R'
        build_cap(f"button_cap_chest_{side}", (math.pi / 2, 0, 0), (sx * CHEST_BTN_X, -32, CHEST_BTN_Z))
        build_cap(f"button_cap_head_{side}", (0, sx * math.pi / 2, 0),
                  (sx * (HEAD_R - 8), 0, TORSO_TOP + HEAD_PLATE_T + HEAD_R * 0.2))
    # drive motor clamp (axis along X at drive-motor position)
    dmx = AXLE_GEAR_X + GEAR_THICKNESS / 2 + GEAR_THICKNESS / 2 + 4 + MOTOR_L / 2
    clamp_half("motor_clamp_drive_A", dmx, 0, DRIVE_MOTOR_Z, 'A')
    clamp_half("motor_clamp_drive_B", dmx, 0, DRIVE_MOTOR_Z, 'B')
    clamp_half("motor_clamp_punch_A", PUNCH_MOTOR_X, 0, SHOULDER_Z, 'A')
    clamp_half("motor_clamp_punch_B", PUNCH_MOTOR_X, 0, SHOULDER_Z, 'B')
    build_motor_box()
    build_housing(+1)
    build_housing(-1)
    build_crank()
    build_con_rod()
    build_upper_arm(+1)
    build_upper_arm(-1)
    build_elbow_lever()
    build_forearm(+1)
    build_forearm(-1)
    build_fist(+1)
    build_fist(-1)
    build_purchased()


def check_size(name, me):
    xs = [v.co.x for v in me.vertices]
    ys = [v.co.y for v in me.vertices]
    zs = [v.co.z for v in me.vertices]
    dims = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    if dims[0] > PRINTER_X or dims[1] > PRINTER_Y or dims[2] > PRINTER_Z:
        print(f"WARNING {name}: {dims[0]:.0f}x{dims[1]:.0f}x{dims[2]:.0f} mm exceeds build volume")


def write_stl(obj, path):
    select_only(obj)
    if bpy.app.version >= (4, 1, 0):
        bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True, global_scale=1.0)
    else:
        bpy.ops.export_mesh.stl(filepath=path, use_selection=True, global_scale=1.0)


def export_all():
    os.makedirs(STL_DIR, exist_ok=True)
    for name, o, rot in PRINT_LIST:
        me = o.data.copy()
        M = (Matrix.Rotation(math.radians(rot[0]), 4, 'X')
             @ Matrix.Rotation(math.radians(rot[1]), 4, 'Y')
             @ Matrix.Rotation(math.radians(rot[2]), 4, 'Z'))
        me.transform(M)
        vs = [v.co for v in me.vertices]
        mn = [min(v[i] for v in vs) for i in range(3)]
        mx = [max(v[i] for v in vs) for i in range(3)]
        me.transform(Matrix.Translation((-(mn[0] + mx[0]) / 2, -(mn[1] + mx[1]) / 2, -mn[2])))
        repair_mesh(me)
        elephant_foot(me)
        check_mesh(name, me)
        check_size(name, me)
        tmp = bpy.data.objects.new(name + "_export", me)
        bpy.context.scene.collection.objects.link(tmp)
        write_stl(tmp, os.path.join(STL_DIR, name + ".stl"))
        bpy.data.objects.remove(tmp, do_unlink=True)
    print(f"Exported {len(PRINT_LIST)} printable parts to {STL_DIR}")


def main():
    global PREVIEW
    reset_scene()
    PREVIEW = coll("assembly_preview")
    build_all()
    export_all()


if __name__ == "__main__":
    main()
