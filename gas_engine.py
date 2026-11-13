"""Small grid (Eulerian) gas solver for the air-gun jet. Independent of Isaac Sim.

Incompressible air on a uniform box grid around the fixture, "stable fluids" style:

  1. boundary conditions: nozzle cells get the jet velocity, solid cells (the fixture) no-slip
  2. advection: semi-Lagrangian, trilinear interpolation (unconditionally stable)
  3. viscous / turbulent diffusion: implicit, Jacobi iterations (this is what makes the jet spread)
  4. projection: solve the pressure Poisson equation (Jacobi) and remove the divergence

Domain sides are open: velocity has zero gradient, pressure is 0. Velocity lives at cell centres
(collocated grid), which is enough for a visual jet and for the air speed over the plate.

Backends: NVIDIA Warp on the GPU when `import warp` works (it ships with Isaac Sim), otherwise
NumPy on the CPU (slow, meant for testing outside Isaac Sim). Both run the same algorithm.

    grid = GasGrid(origin=(x0, y0, z0), shape=(nx, ny, nz), dx=0.0075)
    grid.set_solid_box(centre, half_size)        # every step if the solid moves
    grid.set_nozzle(pos, direction, speed, radius)  # or grid.set_nozzle(None) to stop blowing
    grid.step(dt)
    vel = grid.sample(points)                    # (N, 3) world points -> (N, 3) air velocity [m/s]
"""

import numpy as np

try:
    import warp as wp
    wp.init()
    HAVE_WARP = True
except Exception:                                  # not installed, or no usable device
    wp = None
    HAVE_WARP = False


# ------------------------------------------------------------------ NumPy backend
def _trilinear_np(f, p):
    """f: (nx, ny, nz, C) field, p: (M, 3) continuous cell indices -> (M, C), clamped at the edges."""
    n = np.array(f.shape[:3])
    p = np.clip(p, 0.0, n - 1.0)
    i0 = np.minimum(np.floor(p).astype(np.int64), n - 2)
    t = p - i0
    out = 0.0
    for dx in (0, 1):
        wx = t[:, 0] if dx else 1.0 - t[:, 0]
        for dy in (0, 1):
            wy = t[:, 1] if dy else 1.0 - t[:, 1]
            for dz in (0, 1):
                wz = t[:, 2] if dz else 1.0 - t[:, 2]
                out = out + (wx * wy * wz)[:, None] * f[i0[:, 0] + dx, i0[:, 1] + dy, i0[:, 2] + dz]
    return out


def _neighbour_sum_np(f, mode):
    """Sum of the 6 face neighbours. mode 'edge': zero gradient at the sides, 'zero': value 0 outside."""
    pad = [(1, 1)] * 3 + [(0, 0)] * (f.ndim - 3)
    g = np.pad(f, pad, mode="edge" if mode == "edge" else "constant")
    return (g[2:, 1:-1, 1:-1] + g[:-2, 1:-1, 1:-1] + g[1:-1, 2:, 1:-1] + g[1:-1, :-2, 1:-1]
            + g[1:-1, 1:-1, 2:] + g[1:-1, 1:-1, :-2])


class _NumpyBackend:
    def __init__(self, shape):
        self.shape = shape
        self.u = np.zeros(shape + (3,))
        idx = np.indices(shape).reshape(3, -1).T.astype(np.float64)
        self.idx = idx

    def apply_bc(self, solid, source, source_vel):
        self.u[solid] = 0.0
        self.u[source] = source_vel[source]

    def advect(self, dt_over_dx):
        back = self.idx - dt_over_dx * self.u.reshape(-1, 3)
        self.u = _trilinear_np(self.u, back).reshape(self.u.shape)

    def diffuse(self, a, iters):
        u0 = self.u.copy()
        for _ in range(iters):
            self.u = (u0 + a * _neighbour_sum_np(self.u, "edge")) / (1.0 + 6.0 * a)

    def project(self, solid, iters):
        u = self.u
        g = np.pad(u, [(1, 1)] * 3 + [(0, 0)], mode="edge")
        div = 0.5 * (g[2:, 1:-1, 1:-1, 0] - g[:-2, 1:-1, 1:-1, 0]
                     + g[1:-1, 2:, 1:-1, 1] - g[1:-1, :-2, 1:-1, 1]
                     + g[1:-1, 1:-1, 2:, 2] - g[1:-1, 1:-1, :-2, 2])
        div[solid] = 0.0
        p = np.zeros(self.shape)
        sp = np.pad(solid, 1, mode="constant")
        nbs = [(sp[2:, 1:-1, 1:-1], (slice(2, None), slice(1, -1), slice(1, -1))),
               (sp[:-2, 1:-1, 1:-1], (slice(None, -2), slice(1, -1), slice(1, -1))),
               (sp[1:-1, 2:, 1:-1], (slice(1, -1), slice(2, None), slice(1, -1))),
               (sp[1:-1, :-2, 1:-1], (slice(1, -1), slice(None, -2), slice(1, -1))),
               (sp[1:-1, 1:-1, 2:], (slice(1, -1), slice(1, -1), slice(2, None))),
               (sp[1:-1, 1:-1, :-2], (slice(1, -1), slice(1, -1), slice(None, -2)))]
        for _ in range(iters):
            gp = np.pad(p, 1, mode="constant")                     # open sides: p = 0
            s = 0.0
            for is_solid, sl in nbs:
                s = s + np.where(is_solid, p, gp[sl])             # solid neighbour: dp/dn = 0
            p = np.where(solid, 0.0, (s - div) / 6.0)
        gp = np.pad(p, 1, mode="constant")
        u[..., 0] -= 0.5 * (gp[2:, 1:-1, 1:-1] - gp[:-2, 1:-1, 1:-1])
        u[..., 1] -= 0.5 * (gp[1:-1, 2:, 1:-1] - gp[1:-1, :-2, 1:-1])
        u[..., 2] -= 0.5 * (gp[1:-1, 1:-1, 2:] - gp[1:-1, 1:-1, :-2])
        u[solid] = 0.0

    def sample(self, p):
        return _trilinear_np(self.u, p)


# ------------------------------------------------------------------ Warp backend
if HAVE_WARP:
    @wp.func
    def _vel_at(u: wp.array3d(dtype=wp.vec3), i: int, j: int, k: int):
        nx, ny, nz = u.shape[0], u.shape[1], u.shape[2]
        return u[wp.clamp(i, 0, nx - 1), wp.clamp(j, 0, ny - 1), wp.clamp(k, 0, nz - 1)]

    @wp.func
    def _trilinear(u: wp.array3d(dtype=wp.vec3), p: wp.vec3):
        nx, ny, nz = u.shape[0], u.shape[1], u.shape[2]
        x = wp.clamp(p[0], 0.0, float(nx - 1))
        y = wp.clamp(p[1], 0.0, float(ny - 1))
        z = wp.clamp(p[2], 0.0, float(nz - 1))
        i = wp.min(int(wp.floor(x)), nx - 2)
        j = wp.min(int(wp.floor(y)), ny - 2)
        k = wp.min(int(wp.floor(z)), nz - 2)
        tx = x - float(i)
        ty = y - float(j)
        tz = z - float(k)
        c00 = u[i, j, k] * (1.0 - tx) + u[i + 1, j, k] * tx
        c10 = u[i, j + 1, k] * (1.0 - tx) + u[i + 1, j + 1, k] * tx
        c01 = u[i, j, k + 1] * (1.0 - tx) + u[i + 1, j, k + 1] * tx
        c11 = u[i, j + 1, k + 1] * (1.0 - tx) + u[i + 1, j + 1, k + 1] * tx
        c0 = c00 * (1.0 - ty) + c10 * ty
        c1 = c01 * (1.0 - ty) + c11 * ty
        return c0 * (1.0 - tz) + c1 * tz

    @wp.kernel
    def _k_bc(u: wp.array3d(dtype=wp.vec3), solid: wp.array3d(dtype=wp.int32),
              source: wp.array3d(dtype=wp.int32), source_vel: wp.array3d(dtype=wp.vec3)):
        i, j, k = wp.tid()
        if solid[i, j, k] != 0:
            u[i, j, k] = wp.vec3(0.0, 0.0, 0.0)
        if source[i, j, k] != 0:
            u[i, j, k] = source_vel[i, j, k]

    @wp.kernel
    def _k_advect(u: wp.array3d(dtype=wp.vec3), out: wp.array3d(dtype=wp.vec3), dt_over_dx: float):
        i, j, k = wp.tid()
        back = wp.vec3(float(i), float(j), float(k)) - dt_over_dx * u[i, j, k]
        out[i, j, k] = _trilinear(u, back)

    @wp.kernel
    def _k_diffuse(u0: wp.array3d(dtype=wp.vec3), u: wp.array3d(dtype=wp.vec3),
                   out: wp.array3d(dtype=wp.vec3), a: float):
        i, j, k = wp.tid()
        s = (_vel_at(u, i + 1, j, k) + _vel_at(u, i - 1, j, k) + _vel_at(u, i, j + 1, k)
             + _vel_at(u, i, j - 1, k) + _vel_at(u, i, j, k + 1) + _vel_at(u, i, j, k - 1))
        out[i, j, k] = (u0[i, j, k] + a * s) / (1.0 + 6.0 * a)

    @wp.kernel
    def _k_divergence(u: wp.array3d(dtype=wp.vec3), solid: wp.array3d(dtype=wp.int32),
                      div: wp.array3d(dtype=float)):
        i, j, k = wp.tid()
        d = 0.5 * (_vel_at(u, i + 1, j, k)[0] - _vel_at(u, i - 1, j, k)[0]
                   + _vel_at(u, i, j + 1, k)[1] - _vel_at(u, i, j - 1, k)[1]
                   + _vel_at(u, i, j, k + 1)[2] - _vel_at(u, i, j, k - 1)[2])
        if solid[i, j, k] != 0:
            d = 0.0
        div[i, j, k] = d

    @wp.func
    def _p_nb(p: wp.array3d(dtype=float), solid: wp.array3d(dtype=wp.int32), i: int, j: int, k: int, pc: float):
        if i < 0 or j < 0 or k < 0 or i >= p.shape[0] or j >= p.shape[1] or k >= p.shape[2]:
            return 0.0                                   # open side
        if solid[i, j, k] != 0:
            return pc                                    # wall: dp/dn = 0
        return p[i, j, k]

    @wp.kernel
    def _k_pressure(p: wp.array3d(dtype=float), div: wp.array3d(dtype=float),
                    solid: wp.array3d(dtype=wp.int32), out: wp.array3d(dtype=float)):
        i, j, k = wp.tid()
        if solid[i, j, k] != 0:
            out[i, j, k] = 0.0
            return
        pc = p[i, j, k]
        s = (_p_nb(p, solid, i + 1, j, k, pc) + _p_nb(p, solid, i - 1, j, k, pc)
             + _p_nb(p, solid, i, j + 1, k, pc) + _p_nb(p, solid, i, j - 1, k, pc)
             + _p_nb(p, solid, i, j, k + 1, pc) + _p_nb(p, solid, i, j, k - 1, pc))
        out[i, j, k] = (s - div[i, j, k]) / 6.0

    @wp.func
    def _p_at(p: wp.array3d(dtype=float), i: int, j: int, k: int):
        if i < 0 or j < 0 or k < 0 or i >= p.shape[0] or j >= p.shape[1] or k >= p.shape[2]:
            return 0.0
        return p[i, j, k]

    @wp.kernel
    def _k_subtract_grad(u: wp.array3d(dtype=wp.vec3), p: wp.array3d(dtype=float),
                         solid: wp.array3d(dtype=wp.int32)):
        i, j, k = wp.tid()
        if solid[i, j, k] != 0:
            u[i, j, k] = wp.vec3(0.0, 0.0, 0.0)
            return
        g = wp.vec3(_p_at(p, i + 1, j, k) - _p_at(p, i - 1, j, k),
                    _p_at(p, i, j + 1, k) - _p_at(p, i, j - 1, k),
                    _p_at(p, i, j, k + 1) - _p_at(p, i, j, k - 1))
        u[i, j, k] = u[i, j, k] - 0.5 * g

    @wp.kernel
    def _k_sample(u: wp.array3d(dtype=wp.vec3), pts: wp.array(dtype=wp.vec3), out: wp.array(dtype=wp.vec3)):
        t = wp.tid()
        out[t] = _trilinear(u, pts[t])


class _WarpBackend:
    def __init__(self, shape, device):
        self.shape = shape
        self.device = device
        z3 = np.zeros(shape + (3,), dtype=np.float32)
        self.u = wp.array(z3, dtype=wp.vec3, device=device)
        self.tmp = wp.zeros_like(self.u)
        self.u0 = wp.zeros_like(self.u)
        self.p = wp.zeros(shape, dtype=float, device=device)
        self.p2 = wp.zeros_like(self.p)
        self.div = wp.zeros_like(self.p)
        self.solid = None
        self.source = None
        self.source_vel = None

    def upload_bc(self, solid, source, source_vel):
        d = self.device
        self.solid = wp.array(solid.astype(np.int32), dtype=wp.int32, device=d)
        self.source = wp.array(source.astype(np.int32), dtype=wp.int32, device=d)
        self.source_vel = wp.array(source_vel.astype(np.float32), dtype=wp.vec3, device=d)

    def apply_bc(self, solid, source, source_vel):
        wp.launch(_k_bc, dim=self.shape, inputs=[self.u, self.solid, self.source, self.source_vel],
                  device=self.device)

    def advect(self, dt_over_dx):
        wp.launch(_k_advect, dim=self.shape, inputs=[self.u, self.tmp, float(dt_over_dx)], device=self.device)
        self.u, self.tmp = self.tmp, self.u

    def diffuse(self, a, iters):
        wp.copy(self.u0, self.u)
        for _ in range(iters):
            wp.launch(_k_diffuse, dim=self.shape, inputs=[self.u0, self.u, self.tmp, float(a)], device=self.device)
            self.u, self.tmp = self.tmp, self.u

    def project(self, solid, iters):
        wp.launch(_k_divergence, dim=self.shape, inputs=[self.u, self.solid, self.div], device=self.device)
        self.p.zero_()
        for _ in range(iters):
            wp.launch(_k_pressure, dim=self.shape, inputs=[self.p, self.div, self.solid, self.p2], device=self.device)
            self.p, self.p2 = self.p2, self.p
        wp.launch(_k_subtract_grad, dim=self.shape, inputs=[self.u, self.p, self.solid], device=self.device)

    def sample(self, p):
        pts = wp.array(p.astype(np.float32), dtype=wp.vec3, device=self.device)
        out = wp.zeros(len(p), dtype=wp.vec3, device=self.device)
        wp.launch(_k_sample, dim=len(p), inputs=[self.u, pts, out], device=self.device)
        return out.numpy().astype(np.float64)


# ------------------------------------------------------------------ public grid
class GasGrid:
    def __init__(self, origin, shape, dx, viscosity=2e-3, diffuse_iters=8, pressure_iters=40, backend="auto"):
        """origin: world position of cell (0, 0, 0)'s centre [m]; viscosity: effective (turbulent)
        kinematic viscosity [m^2/s], far above still air's 1.5e-5, which is what makes the jet spread."""
        self._init_geometry(origin, shape, dx, viscosity)
        self.diffuse_iters = diffuse_iters
        self.pressure_iters = pressure_iters
        device = self._pick_device(backend)
        if device:
            self.impl = _WarpBackend(self.shape, device)
            self.backend = f"stable fluids, warp ({device})"
        else:
            self.impl = _NumpyBackend(self.shape)
            self.backend = "stable fluids, numpy"

    def _init_geometry(self, origin, shape, dx, viscosity):
        self.origin = np.asarray(origin, dtype=np.float64)
        self.shape = tuple(int(n) for n in shape)
        self.dx = float(dx)
        self.viscosity = viscosity
        self.centres = np.indices(self.shape).transpose(1, 2, 3, 0) * self.dx + self.origin
        self.solid = np.zeros(self.shape, dtype=bool)
        self.source = np.zeros(self.shape, dtype=bool)
        self.source_vel = np.zeros(self.shape + (3,))
        self._dirty = True

    @staticmethod
    def _pick_device(backend):
        """Warp device name, or None for NumPy."""
        if not (HAVE_WARP if backend == "auto" else backend == "warp"):
            return None
        return "cuda:0" if wp.get_cuda_device_count() > 0 else "cpu"

    @property
    def upper(self):
        return self.origin + (np.array(self.shape) - 1) * self.dx

    def set_solid_box(self, centre, half_size):
        rel = np.abs(self.centres - np.asarray(centre))
        self.solid = (rel <= np.asarray(half_size)).all(axis=-1)
        self.source &= ~self.solid
        self._dirty = True

    def set_nozzle(self, pos, direction=None, speed=0.0, radius=0.004, length=None):
        """Cells in a short cylinder from pos along direction blow at speed. pos None: gun off."""
        self.source[:] = False
        if pos is not None:
            d = np.asarray(direction, dtype=np.float64)
            d /= max(np.linalg.norm(d), 1e-9)
            r = max(radius, 0.75 * self.dx)                       # at least one cell wide
            length = 2.0 * self.dx if length is None else length
            rel = self.centres - np.asarray(pos)
            along = rel @ d
            across = np.linalg.norm(rel - along[..., None] * d, axis=-1)
            self.source = (along >= -0.5 * self.dx) & (along <= length) & (across <= r) & ~self.solid
            self.source_vel[:] = d * speed
        self._dirty = True

    def step(self, dt):
        impl = self.impl
        if self._dirty and isinstance(impl, _WarpBackend):
            impl.upload_bc(self.solid, self.source, self.source_vel)
        self._dirty = False
        impl.apply_bc(self.solid, self.source, self.source_vel)
        impl.advect(dt / self.dx)
        a = self.viscosity * dt / self.dx ** 2
        if a > 0.0:
            impl.diffuse(a, self.diffuse_iters)
        impl.apply_bc(self.solid, self.source, self.source_vel)
        impl.project(self.solid, self.pressure_iters)
        impl.apply_bc(self.solid, self.source, self.source_vel)

    def inside(self, points):
        p = np.asarray(points, dtype=np.float64)
        return ((p >= self.origin) & (p <= self.upper)).all(axis=-1)

    def sample(self, points):
        """Air velocity [m/s] at world points; outside the grid it is the nearest edge value."""
        p = np.atleast_2d(np.asarray(points, dtype=np.float64))
        if not len(p):
            return np.zeros((0, 3))
        return self.impl.sample((p - self.origin) / self.dx)

    def velocity(self):
        """Whole field as (nx, ny, nz, 3) numpy, for checks and plots."""
        u = self.impl.u
        return u if isinstance(u, np.ndarray) else u.numpy().astype(np.float64)


# ================================================================== Lattice Boltzmann solver
# D3Q19 lattice: rest, 6 faces, 12 edges. Internally everything is in lattice units (dx = dt = 1);
# GasGrid's world interface is kept, so LBMGrid is a drop-in replacement.
_C = np.array([(0, 0, 0),
               (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1),
               (1, 1, 0), (-1, -1, 0), (1, -1, 0), (-1, 1, 0),
               (1, 0, 1), (-1, 0, -1), (1, 0, -1), (-1, 0, 1),
               (0, 1, 1), (0, -1, -1), (0, 1, -1), (0, -1, 1)], dtype=np.int32)
_W = np.array([1 / 3] + [1 / 18] * 6 + [1 / 36] * 12)
_OPP = np.array([int(np.nonzero((_C == -c).all(axis=1))[0][0]) for c in _C], dtype=np.int32)
_SMAG = 18.0 * np.sqrt(2.0)          # factor in the Smagorinsky relaxation-time formula


def _feq_np(rho, u):
    """Equilibrium (19, ...) for density (...) and lattice velocity (..., 3)."""
    cu = np.tensordot(_C.astype(np.float64), u, axes=([1], [u.ndim - 1]))     # (19, ...)
    uu = (u * u).sum(axis=-1)
    w = _W.reshape((19,) + (1,) * rho.ndim)
    return w * rho * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * uu)


def _shift_np(a, c, fill):
    """out[x] = a[x - c]; cells whose source lies outside the grid take fill (array or scalar)."""
    out = np.array(fill, dtype=a.dtype, copy=True) if np.ndim(fill) else np.full(a.shape, fill, dtype=a.dtype)
    dst, src = [], []
    for ci in c:
        dst.append(slice(1, None) if ci == 1 else slice(None, -1) if ci == -1 else slice(None))
        src.append(slice(None, -1) if ci == 1 else slice(1, None) if ci == -1 else slice(None))
    out[tuple(dst)] = a[tuple(src)]
    return out


class _NumpyLBM:
    def __init__(self, shape):
        self.shape = shape
        self.u = np.zeros(shape + (3,))                     # lattice velocity
        self.f = _feq_np(np.ones(shape), self.u)

    def _edge(self, f):
        """Populations entering from outside the grid: still far-field air (equilibrium at rest).
        Tried and unstable here: zero-gradient copy, and equilibrium at the local velocity (the
        jet's entrainment near the top side fed back on itself)."""
        return np.broadcast_to(_W.reshape(19, 1, 1, 1), f.shape)

    def upload(self, solid, source):
        pass

    def step(self, solid, source, u_src, tau0, cs2):
        f = self.f
        edge = self._edge(f)
        fn = np.empty_like(f)
        for q in range(19):                                 # streaming (pull)
            fn[q] = _shift_np(f[q], _C[q], edge[q])
            from_solid = _shift_np(solid, _C[q], False) & ~solid
            fn[q][from_solid] = f[_OPP[q]][from_solid]      # half-way bounce-back on the fixture
        rho = fn.sum(axis=0)
        cf = _C.astype(np.float64)
        u = np.tensordot(cf.T, fn, axes=([1], [0])).transpose(1, 2, 3, 0) / rho[..., None]
        feq = _feq_np(rho, u)
        neq = fn - feq
        pi = np.einsum("qa,qb,q...->...ab", cf, cf, neq)
        pi_norm = np.sqrt((pi * pi).sum(axis=(-1, -2)))
        tau = 0.5 * (tau0 + np.sqrt(tau0 * tau0 + cs2 * _SMAG * pi_norm / rho))   # Smagorinsky LES
        f = fn - neq / tau
        if source.any():                                    # nozzle: equilibrium at the jet velocity
            us = np.broadcast_to(u_src, (int(source.sum()), 3))
            f[:, source] = _feq_np(rho[source], us)
            u[source] = u_src
        f[:, solid] = _W[:, None]
        u[solid] = 0.0
        self.f, self.u = f, u

    def sample(self, p):
        return _trilinear_np(self.u, p)

    def velocity(self):
        return self.u

    def density(self):
        return self.f.sum(axis=0)


if HAVE_WARP:
    @wp.func
    def _lbm_feq(q: int, rho: float, u: wp.vec3, cx: wp.array(dtype=wp.int32), cy: wp.array(dtype=wp.int32),
                 cz: wp.array(dtype=wp.int32), w: wp.array(dtype=float)):
        cu = float(cx[q]) * u[0] + float(cy[q]) * u[1] + float(cz[q]) * u[2]
        return w[q] * rho * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * wp.dot(u, u))

    @wp.func
    def _lbm_pull(f: wp.array4d(dtype=float), solid: wp.array3d(dtype=wp.int32), u: wp.array3d(dtype=wp.vec3),
                  cx: wp.array(dtype=wp.int32), cy: wp.array(dtype=wp.int32), cz: wp.array(dtype=wp.int32),
                  w: wp.array(dtype=float), opp: wp.array(dtype=wp.int32), q: int, i: int, j: int, k: int):
        si = i - cx[q]
        sj = j - cy[q]
        sk = k - cz[q]
        if si < 0 or sj < 0 or sk < 0 or si >= f.shape[1] or sj >= f.shape[2] or sk >= f.shape[3]:
            return w[q]                                              # open side: still air
        if solid[si, sj, sk] != 0:
            return f[opp[q], i, j, k]                                # bounce-back
        return f[q, si, sj, sk]

    @wp.kernel
    def _k_lbm(f: wp.array4d(dtype=float), fo: wp.array4d(dtype=float), u: wp.array3d(dtype=wp.vec3),
               solid: wp.array3d(dtype=wp.int32), source: wp.array3d(dtype=wp.int32), u_src: wp.vec3,
               cx: wp.array(dtype=wp.int32), cy: wp.array(dtype=wp.int32), cz: wp.array(dtype=wp.int32),
               w: wp.array(dtype=float), opp: wp.array(dtype=wp.int32), tau0: float, cs2: float):
        i, j, k = wp.tid()
        if solid[i, j, k] != 0:
            for q in range(19):
                fo[q, i, j, k] = w[q]
            u[i, j, k] = wp.vec3(0.0, 0.0, 0.0)
            return
        rho = float(0.0)
        m = wp.vec3(0.0, 0.0, 0.0)
        for q in range(19):
            fq = _lbm_pull(f, solid, u, cx, cy, cz, w, opp, q, i, j, k)
            rho += fq
            m += wp.vec3(float(cx[q]), float(cy[q]), float(cz[q])) * fq
        uu = m / rho
        pxx = float(0.0)
        pyy = float(0.0)
        pzz = float(0.0)
        pxy = float(0.0)
        pxz = float(0.0)
        pyz = float(0.0)
        for q in range(19):
            n = _lbm_pull(f, solid, u, cx, cy, cz, w, opp, q, i, j, k) - _lbm_feq(q, rho, uu, cx, cy, cz, w)
            ax = float(cx[q])
            ay = float(cy[q])
            az = float(cz[q])
            pxx += ax * ax * n
            pyy += ay * ay * n
            pzz += az * az * n
            pxy += ax * ay * n
            pxz += ax * az * n
            pyz += ay * az * n
        pn = wp.sqrt(pxx * pxx + pyy * pyy + pzz * pzz + 2.0 * (pxy * pxy + pxz * pxz + pyz * pyz))
        tau = 0.5 * (tau0 + wp.sqrt(tau0 * tau0 + cs2 * 25.455844 * pn / rho))   # 18 * sqrt(2)
        if source[i, j, k] != 0:
            for q in range(19):
                fo[q, i, j, k] = _lbm_feq(q, rho, u_src, cx, cy, cz, w)
            u[i, j, k] = u_src
            return
        for q in range(19):
            fq = _lbm_pull(f, solid, u, cx, cy, cz, w, opp, q, i, j, k)
            fo[q, i, j, k] = fq - (fq - _lbm_feq(q, rho, uu, cx, cy, cz, w)) / tau
        u[i, j, k] = uu


class _WarpLBM:
    def __init__(self, shape, device):
        self.shape = shape
        self.device = device
        f0 = _feq_np(np.ones(shape), np.zeros(shape + (3,))).astype(np.float32)
        self.f = wp.array(f0, dtype=float, device=device)
        self.f2 = wp.zeros_like(self.f)
        self.u = wp.zeros(shape, dtype=wp.vec3, device=device)
        self.cx, self.cy, self.cz = (wp.array(_C[:, a].copy(), dtype=wp.int32, device=device) for a in range(3))
        self.w = wp.array(_W.astype(np.float32), dtype=float, device=device)
        self.opp = wp.array(_OPP, dtype=wp.int32, device=device)
        self.solid = self.source = None

    def upload(self, solid, source):
        self.solid = wp.array(solid.astype(np.int32), dtype=wp.int32, device=self.device)
        self.source = wp.array(source.astype(np.int32), dtype=wp.int32, device=self.device)

    def step(self, solid, source, u_src, tau0, cs2):
        wp.launch(_k_lbm, dim=self.shape, device=self.device,
                  inputs=[self.f, self.f2, self.u, self.solid, self.source, wp.vec3(*[float(v) for v in u_src]),
                          self.cx, self.cy, self.cz, self.w, self.opp, float(tau0), float(cs2)])
        self.f, self.f2 = self.f2, self.f

    def sample(self, p):
        pts = wp.array(p.astype(np.float32), dtype=wp.vec3, device=self.device)
        out = wp.zeros(len(p), dtype=wp.vec3, device=self.device)
        wp.launch(_k_sample, dim=len(p), inputs=[self.u, pts, out], device=self.device)
        return out.numpy().astype(np.float64)

    def velocity(self):
        return self.u.numpy().astype(np.float64)

    def density(self):
        return self.f.numpy().sum(axis=0)


class LBMGrid(GasGrid):
    """Lattice Boltzmann (D3Q19, BGK + Smagorinsky LES) with the same interface as GasGrid.

    Weakly compressible: the lattice time step is chosen so the fastest air (max_speed) stays at
    `mach` in lattice units (well below the lattice sound speed 0.577), so step(dt) runs many small
    sub-steps. Fixture: half-way bounce-back. Nozzle: equilibrium at the jet velocity. Open sides:
    populations coming in from outside are still air (equilibrium at rest density, zero velocity)."""

    def __init__(self, origin, shape, dx, viscosity=5e-4, max_speed=3.0, mach=0.1, smagorinsky=0.17,
                 backend="auto"):
        self._init_geometry(origin, shape, dx, viscosity)
        self.dt_lbm = mach * self.dx / max_speed            # [s] one lattice step
        self.tau0 = 0.5 + 3.0 * viscosity * self.dt_lbm / self.dx ** 2
        self.cs2 = smagorinsky ** 2
        self.scale = self.dx / self.dt_lbm                  # lattice velocity -> m/s
        self._left = 0.0
        self.substeps = 0
        device = self._pick_device(backend)
        self.impl = _WarpLBM(self.shape, device) if device else _NumpyLBM(self.shape)
        where = f"warp ({device})" if device else "numpy"
        self.backend = (f"lattice Boltzmann D3Q19, {where}, dt {self.dt_lbm * 1e3:.3f} ms, "
                        f"tau0 {self.tau0:.4f}")

    def step(self, dt):
        if self._dirty:
            self.impl.upload(self.solid, self.source)
            self._dirty = False
        u_src = self.source_vel.reshape(-1, 3)[0] / self.scale
        self._left += dt
        n = int(self._left / self.dt_lbm)
        self._left -= n * self.dt_lbm
        for _ in range(n):
            self.impl.step(self.solid, self.source, u_src, self.tau0, self.cs2)
        self.substeps += n

    def sample(self, points):
        return super().sample(points) * self.scale

    def velocity(self):
        return self.impl.velocity() * self.scale

    def density(self):
        """Lattice density (1 = ambient); its spread shows the compressibility error."""
        return self.impl.density()
