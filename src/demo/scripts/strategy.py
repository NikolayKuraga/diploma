import math
from abc import ABC, abstractmethod
import numpy as np
from utils import *
from enum import Enum
from vmo import MotionDirectionEstimator


class StrategyType(Enum):
    VELOCITY = 0  # vx, vy, vz, yaw_rate
    ACCELERATION = 1  # ax, ay, az, yaw_rate
    THRUST = 2  # roll_rate, pitch_rate, yaw_rate, thrust


class Strategy(ABC):

    def __init__(self):
        self.pos_prev = None
        self.pos_ref_prev = None

    def attack_cmd(self, pos_info, pos_ref, img_center, reset):
        if reset:
            self.on_reset()
        return self.create_cmd(pos_info, pos_ref, img_center)

    @abstractmethod
    def search_cmd(self):
        pass

    # Type VELOCITY: [vx, vy, vz, yawrate]
    # Type ACCELERATION [ax, ay, az, yawrate]
    # Type THRUST [roll, pitch, yaw, thrust]
    @abstractmethod
    def create_cmd(self, pos_info, pos_ref, img_center):
        pass

    @abstractmethod
    def on_reset(self):
        pass

    @abstractmethod
    def get_type(self):
        pass


class BasicStrategy(Strategy):

    def search_cmd(self):
        return 0, 0, 0, 0

    def __init__(self):
        super().__init__()
        self.sp = 10

    def create_cmd(self, pos_info, pos_ref, img_center):
        return self.sp, 0, 0.01 * (pos_ref[1] - pos_info[1]), 0.01 * (pos_ref[0] - pos_info[0])

    def on_reset(self):
        pass

    def get_type(self):
        pass


class HighSpeedStrategy(Strategy):

    def __init__(self, w, h):
        super().__init__()
        self.v_norm_d = 15.0
        self.R_cb = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        self.n_cc = np.array([0, 0, 1])
        self.count = 0
        self.f = 1117.0
        self.u0 = w * 0.5
        self.v0 = h * 0.5
        self.u_prev = 1
        self.fl = [1117.7832090530542, 1117.7832090530542]
        self.pp = [640.5098521801531, 360.5]
        self.K = np.array([
            [self.fl[0], 0., self.pp[0]],
            [0., self.fl[1], self.pp[1]],
            [0., 0., 1.]
        ])
        self.motion_estimator = MotionDirectionEstimator(self.fl, self.pp, self.R_cb)

    def create_cmd(self, pos_info, pos_ref, img_center):
        self.motion_estimator.update(pos_info, pos_ref)
        n_bc = self.R_cb.dot(self.n_cc)

        n_co = np.array([pos_ref[0] - self.u0, pos_ref[1] - self.v0, self.f], dtype=np.float64)
        self.u_prev = -np.sign(n_co[0])
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)

        cos_beta = n_bo.dot(n_bc)
        v_b = n_bo * cos_beta - n_bc

        self.count += 1
        v_m = np.array([0., 0., 0.])
        v_m[2] = (self.v_norm_d) * v_b[2]
        v_m[1] = sat(self.count * 0.06, self.v_norm_d)
        v_m[0] = (self.v_norm_d) * v_b[0]

        v = pos_info["mav_R"].dot(v_m)

        yaw_rate = 0.003 * (img_center[0] - pos_ref[0])

        return [v[0], v[1], v[2], yaw_rate]

    def brake(self, pos_info):
        pass

    def on_reset(self):
        self.count = 0
        self.u_prev = 1

    def search_cmd(self):
        return [0.0, 0.0, 0.0, 0.5 * self.u_prev]

    def get_type(self):
        return StrategyType.VELOCITY


class AngleHighSpeedStrategy(Strategy):

    def __init__(self, w, h):
        super().__init__()
        # TODO use dynamic max speed
        self.v_norm_d = 20.0
        self.R_cb = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        self.n_cc = np.array([0, 0, 1])
        self.count = 0
        self.f = 560 # 600 1000
        self.u0 = w * 0.5
        self.v0 = h * 0.5
        self.u_prev = 1

        self.n_bo_prev = np.zeros(3)
        self.sc = 1

    def create_cmd(self, pos_info, pos_ref, img_center):
        n_bc = self.R_cb.dot(self.n_cc)

        # calculate the no
        n_co = np.array([pos_ref[0] - self.u0, pos_ref[1] - self.v0, self.f], dtype=np.float64)
        self.u_prev = -np.sign(n_co[0])
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        # n_eo = pos_info["mav_R"].dot(n_bo)

        n_bo_delta = n_bo - self.n_bo_prev
        sy = 1.25
        sx = 0.9
        if abs(n_bo_delta[0]) > 0.002 and pos_ref[-2] < 40:
            print(n_bo_delta[0])
            sy = -3.0 * np.sign(n_bo_delta[0])
            sx = abs(1 / (sy - 1))

        cos_beta = n_bo.dot(n_bc)
        v_b = n_bo * cos_beta - n_bc

        self.count += 1

        v_m = np.array([0., 0., 0.])

        v_m[1] = sat(self.count * 0.05, self.v_norm_d) * sx
        v_m[0] = (self.v_norm_d + 1) * v_b[0] * sy
        v_m[2] = (self.v_norm_d - 1) * v_b[2]

        v = pos_info["mav_R"].dot(v_m)
        yaw_rate = 0.005 * (img_center[0] - pos_ref[0])

        self.n_bo_prev = n_bo

        return [v[0], v[1], v[2], yaw_rate]

    def on_reset(self):
        self.count = 0
        self.u_prev = 1

    def search_cmd(self):
        return [0.0, 0.0, 0.0, 0.5 * self.u_prev]

    def get_type(self):
        return StrategyType.VELOCITY


class TestStrategy(Strategy):

    def __init__(self):
        super().__init__()
        self.R_cb = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        self.n_cc = np.array([0, 0, 1])
        self.t = 0

    def search_cmd(self):
        return [0.0, 0.0, 0.0, 0.5]

    def create_cmd(self, pos_info, pos_ref, img_center):
        self.t += 0.003
        vy = -0.7 * math.cos(self.t)
        vx = 0.2 * math.sin(self.t)
        vz = 0
        v = pos_info["mav_R"].dot(np.array([vy, vx, vz]))
        yaw_rate = 0.008 * (img_center[0] - pos_ref[0])
        return [v[0], v[1], v[2], yaw_rate]

    def on_reset(self):
        self.t = 0

    def get_type(self):
        return StrategyType.VELOCITY


class AccStrategy(Strategy):

    def __init__(self, w, h):
        super().__init__()
        self.R_cb = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        self.n_cc = np.array([0, 0, 1])
        self.count = 0
        self.f = 640
        self.u0 = w * 0.5
        self.v0 = h * 0.5
        self.u_prev = 1
        self.cnt = 0

    def search_cmd(self):
        return [0.0, 0.0, 0.0, 0.5 * self.u_prev]

    # TODO bound max speed to 15 m/s
    def create_cmd(self, pos_info, pos_ref, img_center):
        # calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)

        # calacute the no
        n_co = np.array([pos_ref[0] - self.u0, pos_ref[1] - self.v0, self.f], dtype=np.float64)
        self.u_prev = -np.sign(n_co[0])
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        n_td = np.array([np.cos(pos_info["mav_yaw"]), np.sin(pos_info["mav_yaw"]), 0.], dtype=np.float64)
        v_1 = max(2.5 - pos_ref[2] / 120., 0.5) * (n_eo - n_td)
        v_2 = 1.0 * n_td

        v_d = v_1 + v_2
        v_d /= np.linalg.norm(v_d)
        V = np.linalg.norm(pos_info["mav_vel"])
        v_d *= V + 1.5
        # v_d *= V + 2.0

        a_d = sat(1.0 * (v_d - pos_info["mav_vel"]), 6.)
        a_d[2] = sat(a_d[2], 2.)

        yaw_rate = 0.0025 * (self.u0 - pos_ref[0])

        # print("n_co:{}, n_bo:{}, n_eo:{}, v_1:{}, v_2:{}, v_d:{}".format(n_co, n_bo, n_eo, v_1, v_2, v_d))
        return [a_d[0], a_d[1], a_d[2], yaw_rate]

    def on_reset(self):
        self.cnt = 0
        self.u_prev = 1

    def get_type(self):
        return StrategyType.ACCELERATION


class AccStrategyVelCmd(Strategy):

    def __init__(self, w, h):
        super().__init__()
        self.R_cb = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
        self.n_cc = np.array([0, 0, 1])
        self.count = 0
        self.f = 640
        self.u0 = w * 0.5
        self.v0 = h * 0.5
        self.u_prev = 1
        self.cnt = 0

    def search_cmd(self):
        return [0.0, 0.0, 0.0, 0.5 * self.u_prev]

    def SaftyZ(self, vz, satf):
        if vz > satf:
            return satf
        elif vz < -satf:
            return -satf
        else:
            return vz

    def create_cmd(self, pos_info, pos_ref, img_center):
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)

        # calacute the no
        n_co = np.array([pos_ref[0] - self.u0, pos_ref[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        # 两种用法：1）给定世界系下固定的n_td，限定打击方向；2）相对光轴一向量，随相机运动
        # n_td = np.array([np.cos(yaw_d), np.sin(yaw_d), 0], dtype=np.float64)
        n_td = np.array([np.cos(pos_info["mav_yaw"]), np.sin(pos_info["mav_yaw"]), 0.], dtype=np.float64)
        # n_td = n_ec
        # n_td /= np.linalg.norm(n_td)
        v_1 = max(2.0 - pos_ref[2] / 100., 0.5) * (n_eo - n_td)  # n_t -> n_td
        v_2 = 1.0 * n_td  # v   -> n_td

        v_d = v_1 + v_2
        v_d /= np.linalg.norm(v_d)
        V = np.linalg.norm(pos_info["mav_vel"])
        v_d *= V + 0.6
        v_d[2] = self.SaftyZ(v_d[2], 1.)
        # v_d *= V + 2.0

        yaw_rate = 0.0025 * (self.u0 - pos_ref[0])

        # print("n_co:{}, n_bo:{}, n_eo:{}, v_1:{}, v_2:{}, v_d:{}".format(n_co, n_bo, n_eo, v_1, v_2, v_d))
        return [v_d[0], v_d[1], v_d[2], yaw_rate]

    def on_reset(self):
        self.cnt = 0
        self.u_prev = 1

    def get_type(self):
        return StrategyType.VELOCITY
