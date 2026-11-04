"""SI units, column vectors, T_A_B maps B into A; URDF RPY = Rz Ry Rx."""
import math
import numpy as np


def vector(value, size=3):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError('Expected finite vector of size %d' % size)
    return a


def rotation(rpy):
    r, p, y = vector(rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def validate_transform(t):
    t = np.asarray(t, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError('Expected finite 4x4 transform')
    r = t[:3, :3]
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9) or not np.allclose(r.T @ r, np.eye(3), atol=1e-8) or not np.isclose(np.linalg.det(r), 1, atol=1e-8):
        raise ValueError('Transform must contain a proper orthonormal rotation')
    return t


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rotation(rpy), vector(xyz)
    return t


def point(t, p):
    return t[:3, :3] @ vector(p) + t[:3, 3]


def inverse(t):
    t = validate_transform(t)
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def motion(kind, axis, q):
    if not math.isfinite(q):
        raise ValueError('Nonfinite joint position')
    if kind == 'fixed':
        return np.eye(4)
    a = vector(axis)
    if np.linalg.norm(a) < 1e-12:
        raise ValueError('Zero joint axis')
    a = a / np.linalg.norm(a)
    t = np.eye(4)
    if kind == 'prismatic':
        t[:3, 3] = a * q
    elif kind in ('revolute', 'continuous'):
        x, y, z = a
        k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        t[:3, :3] = np.eye(3) + math.sin(q)*k + (1-math.cos(q))*(k @ k)
    else:
        raise ValueError('Unsupported joint type: ' + kind)
    return t


def from_quaternion(xyz, quat):
    x, y, z, w = vector(quat, 4)
    if not np.isclose(np.linalg.norm([x, y, z, w]), 1, atol=1e-5):
        raise ValueError('Invalid TF quaternion norm')
    t = transform(xyz)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    return validate_transform(t)


def quaternion(t):
    # Eigenvector method is stable near pi, where trace-only methods are singular.
    r = validate_transform(t)[:3, :3]
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], r.trace()]]) / 3
    _, v = np.linalg.eigh(k)
    q = v[:, -1]
    return q if q[3] >= 0 else -q


def inertia_tensor(value):
    i = np.asarray(value, dtype=float)
    if i.shape != (3, 3) or not np.isfinite(i).all() or not np.allclose(i, i.T, atol=1e-12):
        raise ValueError('Inertia must be finite symmetric 3x3')
    eig = np.linalg.eigvalsh(i)
    tol = max(1e-12, np.max(np.abs(eig))*1e-8)
    if eig[0] < -tol or eig[2] > eig[0]+eig[1]+tol:
        raise ValueError('Unphysical principal moments/triangle inequality')
    return i


def rotate_inertia(i, t):
    r = validate_transform(t)[:3, :3]
    return r @ inertia_tensor(i) @ r.T


def import_properties(mass, cog, inertia=None, mass_unit='kg', length_unit='m', inertia_unit='kg_m2'):
    m = float(mass) * {'kg': 1, 'g': 1e-3}[mass_unit]
    c = vector(cog) * {'m': 1, 'mm': 1e-3}[length_unit]
    if not math.isfinite(m) or m <= 0:
        raise ValueError('Mass must be positive and finite')
    i = None if inertia is None else inertia_tensor(np.asarray(inertia)*{'kg_m2': 1, 'g_mm2': 1e-9}[inertia_unit])
    return m, c, i
