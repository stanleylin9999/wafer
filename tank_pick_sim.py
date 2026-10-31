"""Isaac Sim 6.0.1 — two Franka arms, water tank, lift a CPU socket fixture out and let it drip (Script Editor).

Stand-alone: paste into Window > Script Editor and Run. It opens a NEW empty stage (unsaved
work in the current stage is lost), loads both arms, builds everything and starts by itself.

  1. Water tank with a CPU socket fixture lying flat on a stand in it; two Franka arms face each other across it.
  2. The first arm reaches in from above, pinches the fixture by two opposite edges and lifts it out. The joint path
     comes from the numeric IK below. The fixture is bound to the hand when the fingers close.
  3. Once the fixture clears the water, beads stay on it: some sit there for good, the rest creep
     to the rim and hang under it until a whole drop detaches and falls (wetcloth/drops.py idea:
     pending water per cell). Visual particles only, no fluid solver.
  4. The second arm carries an air gun: it comes in from the far side, tilted, and sweeps the
     jet over the fixture. Beads under the jet are blown off as spray.

Lines that depend on Isaac Sim / PhysX APIs are marked  # ISAAC API.
"""

# ============================================================== USER CONFIG
PICKER_PATH = "/World/ArmPicker"
HELPER_PATH = "/World/ArmHelper"
HELPER_BASE = (1.15, 0.0)        # [m] the first arm's base is at the origin
FRANKA_ASSETS = [                # tried in order, under the Isaac assets root
    "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    "/Isaac/Robots/Franka/franka.usd",
]
ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]
HAND_LINK = "panda_hand"

TANK_CENTER = (0.5, 0.0)         # [m]
TANK_INNER = (0.40, 0.40)
TANK_WALL_TOP = 0.22             # height of the rim above the ground
TANK_WALL_THICKNESS = 0.01
TANK_FLOOR_Z = 0.02              # top of the tank floor
WATER_Z = 0.17                   # water surface

# CPU socket fixture: a square plate lying flat. Width must stay under the 8 cm finger opening.
FIXTURE_SIZE = (0.07, 0.07, 0.012)    # [m] x, y, thickness
FRAME_WIDTH = 0.008              # metal frame around the recess
SOCKET_SIZE = 0.042              # beige socket in the middle
PIN_PITCH = 0.0013
PIN_WIDTH = 0.0007
SCREW_AT = [(-0.031, -0.031), (0.031, -0.031), (-0.031, 0.031), (0.031, 0.031), (-0.031, 0.0), (0.031, 0.0)]
STAND_HEIGHT = 0.08              # keeps the fixture under water, with room for the fingertips
GRASP_Z = 0.106                  # fingertip height: level with the plate
ABOVE_Z = 0.40
LIFT_Z = 0.55
GRASP_RADIUS = 0.12              # [m] fingertips must be this close to the fixture centre to bind
TRACK_TOL = 0.15                 # [rad] joint error after "above": larger means the drives do not follow,
                                 # and the arm switches to writing joint positions directly

FINGER_OPEN = 0.04               # [m] per finger
FINGER_CLOSED = 0.034

PHYSICS_HZ = 240
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]   # [rad] start pose of both arms
# (name, seconds, fingertip z or None to stay, finger target)
SEQUENCE = [
    ("settle", 1.5, None, FINGER_OPEN),
    ("above", 3.0, ABOVE_Z, FINGER_OPEN),
    ("reach", 3.0, GRASP_Z, FINGER_OPEN),
    ("close", 1.0, None, FINGER_CLOSED),
    ("lift", 6.0, LIFT_Z, FINGER_CLOSED),
]

DROP_COUNT = 900
DROP_WIDTH = 0.0046              # [m]
DROP_VOLUME = 5.0e-8             # [m^3] ~0.05 mL, as wetcloth/drops.py
DROP_EVERY_N_STEPS = 4           # USD write rate of the particles
# water left on the fixture when it leaves the water (beads on the plate)
TOP_BEADS = 320                  # on the upper face
UNDER_BEADS = 120                # hanging from the lower face
BEAD_VOLUME = 1.0e-8             # [m^3] mean, ~0.01 mL
TRAPPED_FRACTION = 0.45          # share of the upper-face beads that stay on it for good
SHED_SPEED = (0.004, 0.02)       # [m/s] the others creep to the rim at a speed in this range
UNDER_TAU = 3.0                  # [s] hanging beads feed the drops with this time constant
PENDING_CELLS = 8                # per side: under-face grid collecting water until a drop forms
# groove between the frame and the socket: water pools there and the frame shields it from the jet
GROOVE_BEADS = 170               # extra beads pooled in the groove
GROOVE_VOLUME = 2.0              # [-] their volume, relative to BEAD_VOLUME
GROOVE_BLOW = 0.12               # [-] jet removal rate in the groove, relative to an open surface
# water pouring off the rim as the fixture comes out
RUNOFF_RATE = 70.0               # [drops/s] just after leaving the water
RUNOFF_TAU = 1.2                 # [s] rate decays with this time constant
SPLASH_COUNT = 3                 # small droplets thrown up when a drop lands in the tank
SPLASH_WIDTH = 0.002             # [m]
SPLASH_SPEED = 0.7               # [m/s]
GRAVITY = 9.81

# air gun on the second arm. The first arm's hand sits right above the fixture, so the gun aims
# from the far side at GUN_TILT_DEG from vertical.
GUN_TIP = 0.19                   # [m] nozzle tip along the hand axis, from the hand link
GUN_TILT_DEG = 35.0
GUN_STANDOFF = 0.12              # [m] nozzle to plate along the jet
GUN_RASTER_X = (0.02, -0.01)     # [m] plate rows swept, relative to the plate centre
GUN_RASTER_Y = 0.03              # [m] half length of a row
GUN_PLATE_Z = LIFT_Z             # plate height while held
GUN_DELAY = 2.0                  # [s] natural dripping after the lift, before blowing
GUN_APPROACH = 4.0               # [s]
GUN_PASS = 2.0                   # [s] per row
GUN_RETREAT = 3.0                # [s]
# Moving-jet ideas from Ternes, arXiv 2510.00085: the removal rate peaks under the jet and falls
# off around it, and the jet slows down where there is more water, with a smoothed speed.
JET_RADIUS = 0.025               # [m] removal rate falls to 1/e at this distance
JET_RATE = 12.0                  # [1/s] removal rate of beads right under the jet
JET_LOAD_REF = 15.0              # [mean beads] water under the jet that gets the nominal speed
JET_SPEED_RANGE = (0.4, 2.0)     # [-] sweep speed limits, relative to the nominal speed
JET_SPEED_TAU = 0.3              # [s] smoothing of the sweep speed
JET_SPRAY_SPEED = (1.0, 3.0)     # [m/s] blown-off water, along the jet's horizontal direction
JET_POINTS = 40
JET_FLOW = 3.0                   # [1/s] speed of the drawn air particles along the jet
# ==========================================================================

import asyncio

import numpy as np
import omni.client                                         # ISAAC API
import omni.kit.app
import omni.physx                                          # ISAAC API
import omni.timeline
import omni.usd
from omni.physx.scripts import physicsUtils                # ISAAC API
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, Vt

TANK_PATH = "/World/WaterTank"
FIXTURE_PATH = "/World/CpuFixture"
DROPS_PATH = "/World/TankDrops"
BEADS_PATH = "/World/TankBeads"
JET_PATH = "/World/AirJet"

# stop a previous run of this script (and of the old towel script, if it shares this namespace)
for _old in ("TANK_PICK", "WETCLOTH_M2"):
    if globals().get(_old) is not None:
        try:
            globals()[_old].destroy()
        except Exception:
            pass
TANK_PICK = None
WETCLOTH_M2 = None


def _to_np(a):
    """warp / torch (cpu or cuda) / numpy -> numpy float64."""
    if hasattr(a, "detach"):
        a = a.detach().cpu().numpy()
    elif hasattr(a, "numpy"):
        a = a.numpy()
    return np.asarray(a, dtype=np.float64)


# ------------------------------------------------------------------ Franka kinematics
# modified DH: (a, d, alpha) per joint
FRANKA_DH = [(0.0, 0.333, 0.0), (0.0, 0.0, -np.pi / 2), (0.0, 0.316, np.pi / 2),
             (0.0825, 0.0, np.pi / 2), (-0.0825, 0.384, -np.pi / 2), (0.0, 0.0, np.pi / 2),
             (0.088, 0.0, np.pi / 2)]
FRANKA_TCP_D = 0.107 + 0.1034    # flange + hand to between the fingertips
FRANKA_LIMITS = (np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]),
                 np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]))


def franka_fk(q, tcp_d=FRANKA_TCP_D):
    """Fingertip-centre pose (position, rotation matrix) and every joint axis/origin."""
    T = np.eye(4)
    axes, origins = [], []
    for (a, d, al), th in zip(FRANKA_DH, q):
        ca, sa, ct, st = np.cos(al), np.sin(al), np.cos(th), np.sin(th)
        T = T @ np.array([[ct, -st, 0, a], [st * ca, ct * ca, -sa, -d * sa],
                          [st * sa, ct * sa, ca, d * ca], [0, 0, 0, 1]])
        axes.append(T[:3, 2].copy())
        origins.append(T[:3, 3].copy())
    T = T @ np.array([[np.cos(-np.pi / 4), -np.sin(-np.pi / 4), 0, 0],
                      [np.sin(-np.pi / 4), np.cos(-np.pi / 4), 0, 0],
                      [0, 0, 1, tcp_d], [0, 0, 0, 1]])
    return T[:3, 3], T[:3, :3], axes, origins


def tilted_down(angle):
    """Hand rotation: z axis tilted from straight down toward +x by angle [rad]."""
    z = np.array([np.sin(angle), 0.0, -np.cos(angle)])
    y = np.array([0.0, -1.0, 0.0])
    return np.stack([np.cross(y, z), y, z], 1)


def franka_ik(pos, seed, R_goal=None, tcp_d=FRANKA_TCP_D, iters=300):
    """Damped least squares: tool tip at pos; hand straight down unless R_goal is given."""
    R_goal = tilted_down(0.0) if R_goal is None else R_goal
    q = np.array(seed, dtype=np.float64)
    for _ in range(iters):
        p, R, axes, origins = franka_fk(q, tcp_d)
        ep = np.asarray(pos) - p
        Re = R_goal @ R.T
        eo = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        err = np.concatenate([ep, eo])
        if np.linalg.norm(err) < 1e-5:
            break
        J = np.array([np.concatenate([np.cross(z, p - o), z]) for z, o in zip(axes, origins)]).T
        q = q + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err)
        q = np.clip(q, *FRANKA_LIMITS)
    return q


def build_trajectory():
    """Joint path of SEQUENCE: list of (seconds, joint waypoints, finger target, name)."""
    q = np.array(READY, dtype=np.float64)
    p = franka_fk(q)[0]
    out = []
    for name, secs, goal_z, finger in SEQUENCE:
        if goal_z is None:
            out.append((secs, np.array([q, q]), finger, name))
            continue
        goal = np.array([TANK_CENTER[0], TANK_CENTER[1], goal_z])
        pts = [q]
        for s in np.linspace(0.0, 1.0, 30)[1:]:
            q = franka_ik(p + (goal - p) * s, q)
            pts.append(q)
        out.append((secs, np.array(pts), finger, name))
        p = goal
    return out


def build_gun_trajectory():
    """Second arm's path in its own base frame: list of (seconds, waypoints, blowing, name)."""
    tip_d = 0.107 + GUN_TIP
    tilt = np.radians(GUN_TILT_DEG)
    q = np.array(READY, dtype=np.float64)
    p = franka_fk(q, tip_d)[0]
    angle = 0.0

    def to_base(wx, wy, wz):
        return np.array([HELPER_BASE[0] - wx, HELPER_BASE[1] - wy, wz])

    def nozzle(dx, dy):
        return to_base(TANK_CENTER[0] + dx + GUN_STANDOFF * np.sin(tilt), TANK_CENTER[1] + dy,
                       GUN_PLATE_Z + GUN_STANDOFF * np.cos(tilt))

    def segment(goal, goal_angle, n):
        nonlocal q, p, angle
        pts = [q]
        for s in np.linspace(0.0, 1.0, n)[1:]:
            q = franka_ik(p + (goal - p) * s, q, tilted_down(angle + (goal_angle - angle) * s), tip_d)
            pts.append(q)
        p, angle = goal, goal_angle
        return np.array(pts)

    total = sum(step[1] for step in SEQUENCE) + GUN_DELAY
    out = [(total, np.array([q, q]), False, "gun wait")]
    start = nozzle(GUN_RASTER_X[0], -GUN_RASTER_Y)
    out.append((GUN_APPROACH, segment(start, tilt, 40), False, "gun approach"))
    y_end = GUN_RASTER_Y
    for k, dx in enumerate(GUN_RASTER_X):
        if k:
            out.append((GUN_PASS / 2, segment(nozzle(dx, y_end), tilt, 10), True, "gun shift"))
            y_end = -y_end
        out.append((GUN_PASS, segment(nozzle(dx, y_end), tilt, 30), True, "gun sweep"))
    out.append((GUN_RETREAT, segment(franka_fk(np.array(READY), tip_d)[0], 0.0, 40), False, "gun retreat"))
    return out


# ------------------------------------------------------------------ scene
def build_world(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(GRAVITY)
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr().Set(int(PHYSICS_HZ))
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(800.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500.0)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))
    make_box(stage, "/World/Ground", (0.5, 0, -0.05), (6, 6, 0.1), (0.45, 0.47, 0.5), 1.0, True)


def make_box(stage, path, center, size, color, opacity, collide):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    prim = cube.GetPrim()
    physicsUtils.setup_transform_as_scale_orient_translate(prim)
    physicsUtils.set_or_add_scale_op(prim, Gf.Vec3f(*size))
    physicsUtils.set_or_add_orient_op(prim, Gf.Quatf(1.0))
    physicsUtils.set_or_add_translate_op(prim, Gf.Vec3f(*center))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    cube.CreateDisplayOpacityAttr([opacity])
    if collide:
        UsdPhysics.CollisionAPI.Apply(prim)
    return prim


def build_tank(stage):
    UsdGeom.Xform.Define(stage, TANK_PATH)
    cx, cy = TANK_CENTER
    ix, iy = TANK_INNER
    w = TANK_WALL_THICKNESS
    h = TANK_WALL_TOP - TANK_FLOOR_Z
    mz = TANK_FLOOR_Z + h / 2
    glass, op = (0.75, 0.78, 0.82), 0.35
    make_box(stage, f"{TANK_PATH}/bottom", (cx, cy, TANK_FLOOR_Z / 2),
             (ix + 2 * w, iy + 2 * w, TANK_FLOOR_Z), glass, op, True)
    make_box(stage, f"{TANK_PATH}/front", (cx + ix / 2 + w / 2, cy, mz), (w, iy + 2 * w, h), glass, op, True)
    make_box(stage, f"{TANK_PATH}/back", (cx - ix / 2 - w / 2, cy, mz), (w, iy + 2 * w, h), glass, op, True)
    make_box(stage, f"{TANK_PATH}/left", (cx, cy + iy / 2 + w / 2, mz), (ix, w, h), glass, op, True)
    make_box(stage, f"{TANK_PATH}/right", (cx, cy - iy / 2 - w / 2, mz), (ix, w, h), glass, op, True)
    make_box(stage, f"{TANK_PATH}/water", (cx, cy, (TANK_FLOOR_Z + WATER_Z) / 2),
             (ix, iy, WATER_Z - TANK_FLOOR_Z), (0.2, 0.5, 0.9), 0.25, False)   # visual only


def build_fixture(stage):
    """Grey frame, black recess, beige socket with gold pins; both faces, z is the plate normal."""
    W, L, T = FIXTURE_SIZE
    root = UsdGeom.Xform.Define(stage, FIXTURE_PATH)
    physicsUtils.setup_transform_as_scale_orient_translate(root)
    physicsUtils.set_or_add_orient_op(root, Gf.Quatf(1.0))
    physicsUtils.set_or_add_translate_op(root, Gf.Vec3f(
        TANK_CENTER[0], TANK_CENTER[1], TANK_FLOOR_Z + STAND_HEIGHT + T / 2))
    grey, black, beige = (0.32, 0.34, 0.36), (0.03, 0.03, 0.03), (0.78, 0.70, 0.55)
    f = FRAME_WIDTH
    bars = [("north", (0, L / 2 - f / 2, 0), (W, f, T)), ("south", (0, -L / 2 + f / 2, 0), (W, f, T)),
            ("west", (-W / 2 + f / 2, 0, 0), (f, L - 2 * f, T)), ("east", (W / 2 - f / 2, 0, 0), (f, L - 2 * f, T))]
    for name, pos, size in bars:
        make_box(stage, f"{FIXTURE_PATH}/{name}", pos, size, grey, 1.0, True)
    make_box(stage, f"{FIXTURE_PATH}/recess", (0, 0, 0), (W - 2 * f, L - 2 * f, T / 2), black, 1.0, True)
    make_box(stage, f"{FIXTURE_PATH}/socket", (0, 0, 0), (SOCKET_SIZE, SOCKET_SIZE, 0.008), beige, 1.0, False)
    s = np.arange(-0.018, 0.0181, PIN_PITCH)
    X, Y = np.meshgrid(s, s)
    ring = np.maximum(np.abs(X), np.abs(Y))
    keep = (ring > 0.0095) | (ring < 0.005)                # pin field, gap, centre cluster
    for side, sign in (("top", 1.0), ("under", -1.0)):
        pts = np.stack([X[keep], Y[keep], np.full(keep.sum(), sign * 0.0041)], 1)
        pins = UsdGeom.Points.Define(stage, f"{FIXTURE_PATH}/pins_{side}")
        pins.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
        pins.CreateWidthsAttr(Vt.FloatArray([PIN_WIDTH] * len(pts)))
        pins.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
        pins.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.65, 0.2)])
        for i, (x, y) in enumerate(SCREW_AT):
            screw = UsdGeom.Cylinder.Define(stage, f"{FIXTURE_PATH}/screw_{side}{i}")
            screw.CreateAxisAttr(UsdGeom.Tokens.z)
            screw.CreateRadiusAttr(0.0022)
            screw.CreateHeightAttr(0.001)
            screw.AddTranslateOp().Set(Gf.Vec3d(x, y, sign * (T / 2 + 0.0005)))
            screw.CreateDisplayColorAttr([Gf.Vec3f(0.75, 0.75, 0.78)])
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim()).CreateKinematicEnabledAttr(True)


def build_gun(stage):
    """Air gun drawn on the second arm's hand (visual only); its axis is the hand's z axis."""
    hand = None
    for prim in stage.Traverse():
        if prim.GetName() == HAND_LINK and str(prim.GetPath()).startswith(HELPER_PATH):
            hand = str(prim.GetPath())
            break
    if hand is None:
        print(f"[tank] {HAND_LINK} not found under {HELPER_PATH}: air gun not drawn")
        return
    gun = f"{hand}/AirGun"
    UsdGeom.Xform.Define(stage, gun)
    dark, steel = (0.12, 0.13, 0.15), (0.7, 0.7, 0.74)
    make_box(stage, f"{gun}/body", (0, 0, 0.10), (0.032, 0.032, 0.09), dark, 1.0, False)
    make_box(stage, f"{gun}/handle", (0.03, 0, 0.09), (0.03, 0.02, 0.05), dark, 1.0, False)
    make_box(stage, f"{gun}/hose", (0.03, 0, 0.04), (0.008, 0.008, 0.08), (0.8, 0.25, 0.1), 1.0, False)
    make_box(stage, f"{gun}/nozzle", (0, 0, 0.17), (0.008, 0.008, 0.05), steel, 1.0, False)
    print(f"[tank] air gun on {hand}")


def find_franka_url():
    try:
        from isaacsim.storage.native import get_assets_root_path               # ISAAC API
    except ImportError:
        from isaacsim.core.utils.nucleus import get_assets_root_path           # ISAAC API
    root = get_assets_root_path()
    if not root:
        raise RuntimeError("Isaac assets root not found (Content panel > Isaac Sim must be reachable)")
    for rel in FRANKA_ASSETS:
        url = root + rel
        if omni.client.stat(url)[0] == omni.client.Result.OK:
            print(f"[tank] Franka asset: {url}")
            return url
    raise RuntimeError(f"Franka asset not found under {root}: tried {FRANKA_ASSETS}")


def add_arm(stage, path, url, xy, yaw_180):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(url)
    physicsUtils.setup_transform_as_scale_orient_translate(prim)
    physicsUtils.set_or_add_translate_op(prim, Gf.Vec3f(xy[0], xy[1], 0.0))
    physicsUtils.set_or_add_orient_op(prim, Gf.Quatf(0.0, 0.0, 0.0, 1.0) if yaw_180 else Gf.Quatf(1.0))


class Arm:
    def __init__(self, path):
        self.path = path
        try:
            from isaacsim.core.experimental.prims import Articulation          # ISAAC API
            self.art = Articulation(path)
            names = list(self.art.dof_names)
            self.api = "experimental"
        except Exception as exc:
            print(f"[tank] experimental Articulation unavailable: {exc}")
            from isaacsim.core.prims import SingleArticulation                 # ISAAC API
            self.art = SingleArticulation(prim_path=path, name=path.rsplit("/", 1)[-1])
            self.art.initialize()
            names = list(self.art.dof_names)
            self.api = "single"
        self.idx = [names.index(n) for n in ARM_JOINTS + FINGER_JOINTS]
        self.n = len(names)
        self.direct = False

    def read(self):
        """Measured arm joints [rad]."""
        pos = self.art.get_dof_positions() if self.api == "experimental" else self.art.get_joint_positions()
        return _to_np(pos).reshape(-1)[self.idx[:7]]

    def send(self, q, finger):
        vals = list(q) + [finger, finger]
        if self.direct:
            if self.api == "experimental":
                cur = _to_np(self.art.get_dof_positions()).reshape(-1)
                cur[self.idx] = vals
                self.art.set_dof_positions(cur.reshape(1, -1).astype(np.float32))     # ISAAC API
            else:
                cur = _to_np(self.art.get_joint_positions()).reshape(-1)
                cur[self.idx] = vals
                self.art.set_joint_positions(cur.astype(np.float32))                  # ISAAC API
            return
        if self.api == "experimental":
            cur = _to_np(self.art.get_dof_position_targets()).reshape(-1)
            cur[self.idx] = vals
            self.art.set_dof_position_targets(cur.reshape(1, -1).astype(np.float32))  # ISAAC API
        else:
            from isaacsim.core.utils.types import ArticulationAction           # ISAAC API
            self.art.apply_action(ArticulationAction(
                joint_positions=np.array(vals, dtype=np.float32),
                joint_indices=np.array(self.idx, dtype=np.int32)))


class Drips:
    """Ballistic drops drawn as one Points prim (no physics, so writing it during Play is safe)."""

    def __init__(self, stage):
        self.points = UsdGeom.Points.Define(stage, DROPS_PATH)
        self.points.CreateWidthsAttr(Vt.FloatArray([DROP_WIDTH] * DROP_COUNT))
        self.points.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
        self.points.CreateDisplayColorAttr([Gf.Vec3f(0.3, 0.65, 1.0)])
        self.x = np.tile([0.0, 0.0, -10.0], (DROP_COUNT, 1))
        self.width = np.full(DROP_COUNT, DROP_WIDTH)
        self.is_splash = np.zeros(DROP_COUNT, dtype=bool)
        self.v = np.zeros((DROP_COUNT, 3))
        self.live = np.zeros(DROP_COUNT, dtype=bool)
        self.landed_tank = 0
        self.steps = 0
        self.flush()

    def flush(self):
        self.points.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(self.x.astype(np.float32)))
        self.points.GetWidthsAttr().Set(Vt.FloatArray.FromNumpy(self.width.astype(np.float32)))
        self.points.SetWidthsInterpolation(UsdGeom.Tokens.vertex)

    def emit(self, pos, vel=(0.0, 0.0, 0.0), splash=False):
        free = np.nonzero(~self.live)[0]
        if free.size:
            i = free[0]
            self.x[i] = pos
            self.v[i] = vel
            self.live[i] = True
            self.is_splash[i] = splash
            self.width[i] = SPLASH_WIDTH if splash else DROP_WIDTH

    def update(self, h):
        self.v[self.live, 2] -= GRAVITY * h
        self.x[self.live] += self.v[self.live] * h
        in_tank = (np.abs(self.x[:, 0] - TANK_CENTER[0]) <= TANK_INNER[0] / 2) & \
                  (np.abs(self.x[:, 1] - TANK_CENTER[1]) <= TANK_INNER[1] / 2)
        floor = np.where(in_tank, WATER_Z, 0.0)
        landed = self.live & (self.x[:, 2] <= floor)
        self.landed_tank += int((landed & in_tank & ~self.is_splash).sum())
        hits = self.x[landed & in_tank & ~self.is_splash].copy()
        self.live[landed] = False
        self.x[landed] = (0.0, 0.0, -10.0)
        for pos in hits:
            for _ in range(SPLASH_COUNT):
                angle = np.random.uniform(0.0, 2.0 * np.pi)
                out = 0.3 * SPLASH_SPEED * np.array([np.cos(angle), np.sin(angle)])
                self.emit((pos[0], pos[1], WATER_Z + 0.001),
                          (out[0], out[1], np.random.uniform(0.4, 1.0) * SPLASH_SPEED), True)
        self.steps += 1
        if self.steps % DROP_EVERY_N_STEPS == 0:
            self.flush()


class SurfaceWater:
    """Beads on the plate, in plate coordinates; state 0 stays, 1 creeps, 2 hangs under, 3 gone."""

    def __init__(self, stage, drips):
        self.drips = drips
        self.points = UsdGeom.Points.Define(stage, BEADS_PATH)
        self.points.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.8, 1.0)])
        self.started = False
        self.pending = np.zeros((PENDING_CELLS, PENDING_CELLS))
        self.steps = 0
        self.out_time = 0.0
        self.carry = 0.0

    def in_groove(self):
        ring = np.maximum(np.abs(self.xy[:, 0]), np.abs(self.xy[:, 1]))
        return (ring >= SOCKET_SIZE / 2) & (ring < FIXTURE_SIZE[0] / 2 - FRAME_WIDTH)

    def surface_z(self):
        """Height of the surface under each upper-face bead, from the plate centre."""
        ring = np.maximum(np.abs(self.xy[:, 0]), np.abs(self.xy[:, 1]))
        z = np.full(len(ring), FIXTURE_SIZE[2] / 2)        # frame
        z[ring < FIXTURE_SIZE[0] / 2 - FRAME_WIDTH] = FIXTURE_SIZE[2] / 4     # groove floor
        z[ring < SOCKET_SIZE / 2] = 0.004                  # socket
        return z

    def start(self):
        W, L, _ = FIXTURE_SIZE
        n = TOP_BEADS + UNDER_BEADS + GROOVE_BEADS
        self.xy = np.random.uniform(-0.5, 0.5, (n, 2)) * (W, L)
        self.vol = BEAD_VOLUME * np.random.uniform(0.4, 1.6, n)
        self.state = np.ones(n, dtype=np.int32)
        self.state[np.random.random(n) < TRAPPED_FRACTION] = 0
        self.state[TOP_BEADS:] = 2
        first = TOP_BEADS + UNDER_BEADS
        mid = (SOCKET_SIZE / 2 + W / 2 - FRAME_WIDTH) / 2
        half = (W / 2 - FRAME_WIDTH - SOCKET_SIZE / 2) * 0.9 / 2
        side = np.random.randint(0, 4, GROOVE_BEADS)
        along = np.random.uniform(-mid, mid, GROOVE_BEADS)
        across = mid + np.random.uniform(-half, half, GROOVE_BEADS)
        self.xy[first:, 0] = np.where(side < 2, along, across * np.where(side == 2, 1.0, -1.0))
        self.xy[first:, 1] = np.where(side < 2, across * np.where(side == 0, 1.0, -1.0), along)
        self.vol[first:] *= GROOVE_VOLUME
        self.state[first:] = 0
        self.from_socket = np.maximum(np.abs(self.xy[:, 0]), np.abs(self.xy[:, 1])) < SOCKET_SIZE / 2
        r = np.maximum(np.linalg.norm(self.xy, axis=1, keepdims=True), 1e-6)
        self.vel = self.xy / r * np.random.uniform(*SHED_SPEED, (n, 1))
        self.started = True
        print(f"[tank] water on the fixture: {int((self.state == 0).sum())} beads stay, "
              f"{int((self.state == 1).sum())} creep, {int((self.state == 2).sum())} hang, "
              f"{int(((self.state == 0) & self.in_groove()).sum())} in the groove")

    def update(self, h, centre, out_of_water):
        if not self.started:
            if out_of_water:
                self.start()
            return
        W, L, T = FIXTURE_SIZE
        self.runoff(h, centre)
        half = np.array([W / 2, L / 2])
        creeping = self.state == 1
        self.xy[creeping] += self.vel[creeping] * h
        caught = creeping & self.from_socket & self.in_groove()
        self.state[caught] = 0                   # socket water stops in the groove
        creeping = self.state == 1
        over = creeping & (np.abs(self.xy) >= half).any(axis=1)
        self.xy[over] = np.clip(self.xy[over], -half, half)
        self.state[over] = 2                     # round the rim, now hanging under it
        hanging = self.state == 2
        gain = self.vol * (h / UNDER_TAU) * hanging
        self.vol -= gain
        cx = np.clip(((self.xy[:, 0] + W / 2) / W * PENDING_CELLS).astype(int), 0, PENDING_CELLS - 1)
        cy = np.clip(((self.xy[:, 1] + L / 2) / L * PENDING_CELLS).astype(int), 0, PENDING_CELLS - 1)
        np.add.at(self.pending, (cx, cy), gain)
        self.state[hanging & (self.vol < 0.05 * BEAD_VOLUME)] = 3
        for i, j in zip(*np.nonzero(self.pending >= DROP_VOLUME)):
            self.pending[i, j] -= DROP_VOLUME
            near = np.nonzero((cx == i) & (cy == j) & hanging)[0]
            xy = self.xy[near[0]] if near.size else ((i + 0.5) / PENDING_CELLS * W - W / 2,
                                                     (j + 0.5) / PENDING_CELLS * L - L / 2)
            self.drips.emit((centre[0] + xy[0], centre[1] + xy[1], centre[2] - T / 2 - 0.002))
        self.steps += 1
        if self.steps % DROP_EVERY_N_STEPS == 0:
            self.flush(centre, T)

    def runoff(self, h, centre):
        """Water still sheeting off the plate: fast drops from random rim points, fading out."""
        W, L, T = FIXTURE_SIZE
        self.out_time += h
        self.carry += RUNOFF_RATE * np.exp(-self.out_time / RUNOFF_TAU) * h
        while self.carry >= 1.0:
            self.carry -= 1.0
            a = np.random.uniform(-0.5, 0.5) * W
            b = np.random.choice((-0.5, 0.5)) * L
            x, y = (a, b) if np.random.random() < 0.5 else (b, a)
            self.drips.emit((centre[0] + x, centre[1] + y, centre[2] - T / 2), (0.0, 0.0, -0.1))

    def blow(self, h, centre, hit, direction):
        """Beads under the jet leave as spray; returns the water load under the jet [mean beads]."""
        if not self.started:
            return JET_LOAD_REF
        r = np.linalg.norm(self.xy + np.asarray(centre)[:2] - np.asarray(hit)[:2], axis=1)
        beta = np.exp(-(r / JET_RADIUS) ** 2)
        alive = self.state != 3
        load = float((self.vol[alive] * beta[alive]).sum()) / BEAD_VOLUME
        rate = JET_RATE * beta * np.where(self.in_groove() & (self.state != 2), GROOVE_BLOW, 1.0)
        taken = alive & (np.random.random(len(r)) < rate * h)
        z = centre[2] + self.surface_z()
        flat = np.array([direction[0], direction[1], 0.0])
        flat /= max(np.linalg.norm(flat), 1e-9)
        for i in np.nonzero(taken)[0]:
            vel = flat * np.random.uniform(*JET_SPRAY_SPEED) + (np.random.uniform(-0.6, 0.6) * flat[1],
                                                                np.random.uniform(-0.6, 0.6),
                                                                np.random.uniform(0.2, 1.0))
            self.drips.emit((centre[0] + self.xy[i, 0], centre[1] + self.xy[i, 1], z[i]), vel)
        self.state[taken] = 3
        return load

    def flush(self, centre, T):
        d = np.cbrt(6.0 * self.vol / np.pi)
        z = np.where(self.state == 2, -T / 2 - d / 2, self.surface_z() + d / 2)
        x = np.stack([centre[0] + self.xy[:, 0], centre[1] + self.xy[:, 1], centre[2] + z], 1)
        gone = self.state == 3
        x[gone] = (0.0, 0.0, -10.0)
        d[gone] = 0.0
        self.points.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(x.astype(np.float32)))
        self.points.GetWidthsAttr().Set(Vt.FloatArray.FromNumpy(d.astype(np.float32)))
        self.points.SetWidthsInterpolation(UsdGeom.Tokens.vertex)


class AirJet:
    """White particles from the nozzle to the plate, drawn only while blowing."""

    def __init__(self, stage):
        self.points = UsdGeom.Points.Define(stage, JET_PATH)
        self.points.CreateWidthsAttr(Vt.FloatArray([0.002] * JET_POINTS))
        self.points.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
        self.points.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.97, 1.0)])
        self.u = np.random.random(JET_POINTS)
        self.off = np.random.normal(0.0, 1.0, (JET_POINTS, 3))
        self.steps = 0
        self.hide()

    def hide(self):
        self.points.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(
            np.tile([0.0, 0.0, -10.0], (JET_POINTS, 1)).astype(np.float32)))

    def update(self, h, nozzle, hit):
        self.u = (self.u + JET_FLOW * h) % 1.0
        self.steps += 1
        if self.steps % DROP_EVERY_N_STEPS == 0:
            x = np.asarray(nozzle) + (np.asarray(hit) - np.asarray(nozzle)) * self.u[:, None] \
                + self.off * (0.3 * JET_RADIUS * self.u[:, None])
            self.points.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(x.astype(np.float32)))


class TankPick:
    def __init__(self, stage):
        self.block = stage.GetPrimAtPath(FIXTURE_PATH)
        self.drips = Drips(stage)
        self.water = SurfaceWater(stage, self.drips)
        self.jet = AirJet(stage)
        self.gun = build_gun_trajectory()
        self.gun_i = 0
        self.gun_t = 0.0
        self.gun_speed = 1.0
        self.blowing = False
        self.traj = build_trajectory()
        self.picker = Arm(PICKER_PATH)
        self.helper = Arm(HELPER_PATH)
        self.step_i = 0
        self.t = 0.0
        self.bound = None                # (block position, block rotation) in the hand frame
        self.ok = False
        self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(  # ISAAC API
            self.on_physics_step)
        print(f"[tank] ready, picker {self.picker.api}, phase {self.traj[0][3]}")

    def tcp(self):
        """Fingertip pose from the measured joints (the picker's base is at the origin)."""
        p, R, _, _ = franka_fk(self.picker.read())
        return p, R

    def bind_block(self):
        p, R = self.tcp()
        b = np.array(self.block.GetAttribute("xformOp:translate").Get(), dtype=np.float64)
        dist = float(np.linalg.norm(b - p))
        if dist > GRASP_RADIUS:
            print(f"[tank] fingertips {dist:.3f} m from the block, not grasped. Fingertips at {p}")
            return
        self.bound = R.T @ (b - p)
        print(f"[tank] block grasped ({dist:.3f} m from the fingertips)")

    def follow_block(self):
        p, R = self.tcp()
        physicsUtils.set_or_add_translate_op(self.block, Gf.Vec3f(*(p + R @ self.bound)))

    def check_tracking(self, target):
        err = float(np.abs(self.picker.read() - target).max())
        print(f"[tank] joint tracking error {err:.3f} rad")
        if err > TRACK_TOL and not self.picker.direct:
            print("[tank] drives are not following: writing joint positions directly from now on")
            self.picker.direct = True
            self.helper.direct = True

    def on_physics_step(self, h):
        try:
            secs, pts, finger, name = self.traj[self.step_i]
            self.t += h
            a = min(self.t / secs, 1.0) * (len(pts) - 1)
            k = min(int(a), len(pts) - 2)
            self.picker.send(pts[k] + (pts[k + 1] - pts[k]) * (a - k), finger)
            if self.t >= secs and self.step_i < len(self.traj) - 1:
                if name == "close":
                    self.bind_block()
                elif name == "above":
                    self.check_tracking(pts[-1])
                self.step_i += 1
                self.t = 0.0
                print(f"[tank] phase {self.traj[self.step_i][3]}")
            if self.bound is not None:
                self.follow_block()
            b = self.block.GetAttribute("xformOp:translate").Get()
            self.water.update(h, b, b[2] - FIXTURE_SIZE[2] / 2 > WATER_Z)
            self.step_gun(h, b)
            self.drips.update(h)
        except Exception as exc:
            self._sub = None
            print(f"[tank] step failed, stopping: {exc}")

    def step_gun(self, h, centre):
        secs, pts, blow, name = self.gun[self.gun_i]
        self.gun_t += h * (self.gun_speed if blow else 1.0)
        a = min(self.gun_t / secs, 1.0) * (len(pts) - 1)
        k = min(int(a), len(pts) - 2)
        q = pts[k] + (pts[k + 1] - pts[k]) * (a - k)
        self.helper.send(q, 0.0)
        if blow:
            p, R, _, _ = franka_fk(q, 0.107 + GUN_TIP)
            nozzle = np.array([HELPER_BASE[0] - p[0], HELPER_BASE[1] - p[1], p[2]])
            direction = np.array([-R[0, 2], -R[1, 2], R[2, 2]])
            top = centre[2] + FIXTURE_SIZE[2] / 2
            hit = nozzle + direction * (top - nozzle[2]) / direction[2]
            load = self.water.blow(h, centre, hit, direction)
            want = np.clip(JET_LOAD_REF / max(load, 1e-6), *JET_SPEED_RANGE)
            self.gun_speed += (want - self.gun_speed) * min(h / JET_SPEED_TAU, 1.0)
            self.jet.update(h, nozzle, hit)
        elif self.blowing:
            self.jet.hide()
            alive = self.water.state != 3
            print(f"[tank] blowing done, {int(alive.sum())} beads left on the fixture, "
                  f"{int((alive & self.water.in_groove()).sum())} in the groove")
        self.blowing = blow
        if self.gun_t >= secs and self.gun_i < len(self.gun) - 1:
            self.gun_i += 1
            self.gun_t = 0.0
            print(f"[tank] phase {self.gun[self.gun_i][3]}")

    def destroy(self):
        self._sub = None


async def restart():
    global TANK_PICK
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    await omni.kit.app.get_app().next_update_async()
    url = find_franka_url()
    await omni.usd.get_context().new_stage_async()
    stage = omni.usd.get_context().get_stage()
    build_world(stage)
    build_tank(stage)
    add_arm(stage, PICKER_PATH, url, (0.0, 0.0), False)
    add_arm(stage, HELPER_PATH, url, HELPER_BASE, True)
    make_box(stage, f"{TANK_PATH}/stand", (TANK_CENTER[0], TANK_CENTER[1], TANK_FLOOR_Z + STAND_HEIGHT / 2),
             (0.04, 0.04, STAND_HEIGHT), (0.3, 0.3, 0.32), 1.0, True)
    build_fixture(stage)
    for _ in range(30):                              # let the Franka references load
        await omni.kit.app.get_app().next_update_async()
    build_gun(stage)
    timeline.play()
    for _ in range(10):                              # let PhysX parse the stage
        await omni.kit.app.get_app().next_update_async()
    TANK_PICK = TankPick(stage)


asyncio.ensure_future(restart())
