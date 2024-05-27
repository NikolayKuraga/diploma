import math
import numpy as np


def sat(a, maxv):
    n = np.linalg.norm(a)
    if n > maxv:
        return a / n * maxv
    else:
        return a


def bound(x, lim):
    if x > lim:
        return lim
    if x < -lim:
        return -lim
    return x


def bound_interval(x, lim):
    return x


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


def rot_matrix(theta) :
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])

    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])

    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R


def get_rotation_matrix(i_v, unit=None):
    # From http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q38
    if unit is None:
        unit = [1.0, 0.0, 0.0]
    # Normalize vector length
    i_v = i_v / np.linalg.norm(i_v)

    # Get axis
    uvw = np.cross(i_v, unit)

    # compute trig values - no need to go through arccos and back
    rcos = np.dot(i_v, unit)
    rsin = np.linalg.norm(uvw)

    #normalize and unpack axis
    if not np.isclose(rsin, 0):
        uvw /= rsin
    u, v, w = uvw

    # Compute rotation matrix - re-expressed to show structure
    return (
        rcos * np.eye(3) +
        rsin * np.array([
            [ 0, -w,  v],
            [ w,  0, -u],
            [-v,  u,  0]
        ]) +
        (1.0 - rcos) * uvw[:,None] * uvw[None,:]
    )


def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.arctan2(t3, t4)

    return X, Y, Z # roll, pitch, yaw


### SE3 ###


# input of size 3
def uwedge(u):
    return np.array([[0, -u[2], u[1]],
                     [u[2], 0, -u[0]],
                     [-u[1], u[0], 0]])


# input of size 6
def vwedge(v):
    Xi = np.zeros([4, 4])
    Xi[0:3, 0:3] = uwedge(v[3:])
    Xi[0:3, 3] = v[0:3]
    return Xi


def wedge(v):
    if v.shape[0] == 3:
        return uwedge(v)
    return vwedge(v)


def uvee(u):
    phi = np.empty([3])
    phi[0] = u[2, 1]
    phi[1] = u[0, 2]
    phi[2] = u[1, 0]
    return np.squeeze(phi)


def vvee(v):
    xi = np.empty(6)
    xi[0:3] = v[0:3, 3]
    xi[3:6] = uvee(v[0:3, 0:3])
    return xi


def vee(v):
    if v.shape[0] == 3:
        return uvee(v)
    return vvee(v)


def splipose(p):
    return p[:3, :3], p[:3, 3]


def skew(A):
    return 0.5 * (A - A.T)


def vec(p):
    R, t = splipose(p)


### rbm ###


def relative_pose(p1, p2):
    return np.linalg.inv(p1) @ p2


# p - pose 4x4 matrix, v - body velocity in world frame
def rbm(p, v):
    return p @ wedge(v)


def rrbm(gco, Vbwc, Vbwo):
    return -wedge(Vbwc) @ gco + gco @ Vbwo


### Conversions ###


def to_hom(po):
    b = np.ones((po.shape[0], 1))
    return np.concatenate((po, b), axis=1)


### Camera ###

def project_px(gco, K, po):
    po = to_hom(po)
    res = np.zeros((po.shape[0], 2))
    i = 0
    for p in po:
        pc_ = gco @ p
        pi = K @ pc_[:3]
        pi = (pi / pi[2])[:2]
        res[i] = pi
        i += 1
    return res


def project_px1(gco, K, po):
    po = to_hom(po)
    res = np.zeros((po.shape[0] * 2, 1))
    i = 0
    for p in po:
        pc_ = gco @ p
        pi = K @ pc_[:3]
        pi = (pi / pi[2])[:2]
        res[i:i + 2] = np.reshape(pi, (2, 1))
        i += 1
    return np.squeeze(res)


def project(gco, focal, po):
    po = to_hom(po)
    res = np.zeros((po.shape[0] * 2, 1))
    i = 0
    for p in po:
        pc_ = gco @ p
        res[i:i + 2] = np.reshape(np.array([focal * pc_[0], focal * pc_[1]]) / pc_[2], (2, 1))
        i += 2
    return np.squeeze(res)


def inverse_image_jacobian_test(gco_bar, po, focal):
    Rco_bar, _ = splipose(gco_bar)
    po_ = to_hom(po)

    nf = po.shape[0]
    J = np.zeros((2 * nf, 6))
    i = 0
    for p in po_:
        pc_bar_i = gco_bar @ p
        poi = po[i]
        Ji = np.array([
            [focal / pc_bar_i[2], 0, -focal * pc_bar_i[0] / pc_bar_i[2] ** 2],
            [0, focal / pc_bar_i[2], -focal * pc_bar_i[0] / pc_bar_i[2] ** 2]
        ])
        Ji = Ji @ np.hstack([Rco_bar, -Rco_bar * wedge(poi)])
        J[2 * i:2 * i + 2, :] = Ji
        i += 1
    return np.linalg.pinv(J)


def inverse_image_jacobian(gco_bar, po, ff):
    Rco_bar, _ = splipose(gco_bar)
    po_ = to_hom(po)

    nf = po.shape[0]
    J = np.zeros((2 * nf, 6))
    i = 0
    for p in po_:
        pc_bar_i = gco_bar @ p
        poi = po[i]
        Ji = np.array([
            [ff[0] / pc_bar_i[2], 0, -ff[0] * pc_bar_i[0] / pc_bar_i[2] ** 2],
            [0, ff[1] / pc_bar_i[2], -ff[1] * pc_bar_i[0] / pc_bar_i[2] ** 2]
        ])
        Ji = Ji @ np.hstack([Rco_bar, -Rco_bar * wedge(poi)])
        J[2 * i:2 * i + 2, :] = Ji
        i += 1
    return np.linalg.pinv(J)


def left_jacobian(phi):
    angle = np.linalg.norm(phi)

    if np.isclose(angle, 0.):
        return np.identity(3) + 0.5 * wedge(phi)

    axis = phi / angle
    s = np.sin(angle)
    c = np.cos(angle)

    return (s / angle) * np.identity(3) + \
        (1 - s / angle) * np.outer(axis, axis) + \
        ((1 - c) / angle) * wedge(axis)


def exp(v):
    rho = v[0:3]
    phi = v[3:6]

    angle = np.linalg.norm(phi)

    R = None
    if np.isclose(angle, 0.):
        R = np.identity(3) + wedge(phi)
    else:
        axis = phi / angle
        s = np.sin(angle)
        c = np.cos(angle)
        R = c * np.identity(3) + (1 - c) * np.outer(axis, axis) + s * wedge(axis)

    t = left_jacobian(phi).dot(rho)
    res = np.identity(4)
    res[:3, :3] = R
    res[:3, 3] = t

    return res
