import numpy as np
from utils import *
import collections


class VMO:

    def __init__(self, fl, pp):
        self.error_limit = 0.001
        self.max_iter = 30
        self.fl = fl
        self.pp = pp
        self.K = np.array([
            [fl[0], 0., pp[0]],
            [0., fl[1], pp[1]],
            [0., 0., 1.]
        ])  # Camera matrix:
        self.Ke = 3.5 * np.identity(6)
        of = 0.5
        fp1 = np.array([0.0, of, 0.0])
        fp2 = np.array([of, 0.0, 0.0])
        fp3 = np.array([0.0, -of, 0.0])
        fp4 = np.array([-of, 0.0, 0.0])
        self.po = np.vstack((fp1, fp2, fp3, fp4))

    # points - np.array 8x1
    def estimate(self, points):
        gco_bar = np.array([[1, 0, 0, 0.0],
                            [0, 1, 0, 0.0],
                            [0, 0, 1, 1.0],
                            [0, 0, 0, 1]])

        for i in range(self.max_iter):
            f_bar = project_px(gco_bar, self.K, self.po)
            f_bar = (f_bar - self.pp)
            f_bar = np.squeeze(np.reshape(f_bar, (8, 1)))
            ee = inverse_image_jacobian(gco_bar, self.po, self.fl) @ (points - f_bar)
            ue = exp(self.Ke @ ee)
            gco_bar = -gco_bar - gco_bar @ ue
            gco_bar /= gco_bar[3][3]
            if np.linalg.norm(ee[:3]) < self.error_limit:
                break

        return gco_bar


class MotionDirectionEstimator:

    def __init__(self, fl, pp, R_cb):
        self.vmo = VMO(fl, pp)
        self.pp = pp
        self.R_cb = R_cb
        self.v_u = np.zeros(3)
        self.n = 30
        self.buff = collections.deque(maxlen=self.n)

    def update(self, pos_info, pos_ref):
        gwo = self.vmo_est(pos_info, pos_ref)
        self.buff.append(gwo)
        if self.is_zero_motion():
            return np.zeros([0, 0, 0])
        return self.velocity_vector()

    def vmo_est(self, pos_info, pos_ref):
        cx, cy, x, y, w, h = pos_ref
        if cx == 0 and cy == 0:
            return
        f = np.array([
            [cx, cy + h/2],
            [cx + w/2, cy],
            [cx, cy - h/2],
            [cx - w/2, cy]
        ])
        f -= self.pp
        f = np.squeeze(np.reshape(f, (8, 1)))
        gco_bar = self.vmo.estimate(f)
        gco_bar_t = self.R_cb @ splipose(gco_bar)[1]
        gco_bar_t = pos_info["mav_R"] @ gco_bar_t
        gwo_t = (gco_bar_t + pos_info["mav_pos"])
        return gwo_t

    def velocity_vector(self):
        pass

    def is_zero_motion(self):
        pass

    def reset(self):
        self.buff.clear()


if __name__ == '__main__':
    buf = collections.deque(maxlen=3)
    for i in range(10):
        buf.append(i)
        print(len(buf))
    buf.clear()
    print(buf)