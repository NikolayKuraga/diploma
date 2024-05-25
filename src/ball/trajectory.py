import time
from abc import ABC, abstractmethod

import numpy as np


class TimeDelta:

    def __init__(self):
        self.start_t = time.time()
        self.t = 0
        self.dt = 0

    def get(self):
        new_t = time.time() - self.start_t
        self.dt = new_t - self.t
        self.t = new_t
        return self.t, self.dt


class Trajectory:

    def __init__(self, x, y, z, vel_func):
        self.start = np.array([x, y, z])
        self.pos = np.array([x, y, z])
        self.vel_func = vel_func # f: R --> R
        self.td = TimeDelta()
        self.t_pos = 0

    @abstractmethod
    def position(self, t):
        pass

    def arc_length(self, t0, t1):
        n = 10
        step = (t1 - t0) / n
        s = 0
        pos = self.position(t0)
        for i in range(1, n + 1):
            new_pos = self.position(t0 + i * step)
            s += np.linalg.norm(new_pos - pos)
            pos = new_pos
        return s

    def param(self, t0, t, target_length):
        for i in range(100):
            length = self.arc_length(t0, t)
            e = target_length - length
            # print(i, e, t)
            if abs(e) < 0.01:
                break
            t += e / 110
        return t, t - t0

    def next(self):
        t, dt = self.td.get()
        dist = self.vel_func(t) * dt
        tn, _ = self.param(self.t_pos, dt, dist)
        self.t_pos = tn
        return self.position(tn)

# без использования матрицы перехода
# class CircleTrajectory(Trajectory):

    # def __init__(self, x, y, z, r, vel_func):
    #     super().__init__(x, y, z, vel_func)
    #     self.r = r

    # def velocity(self, t):
    #     return 5

    # def position(self, t):
    #     return self.start + np.array([
    #         self.r * np.cos(t),
    #         self.r * np.sin(t),
    #         0]
    #     )


# class LemniscateBernoulliTrajectory(Trajectory):

#     def __init__(self, x, y, z, c, vel_func):
#         super().__init__(x, y, z, vel_func)
#         self.c = c

#     def position(self, t):
#         return self.start + np.array([
#             self.c * pow(2, 1/2) * np.cos(t) / (1 + np.sin(t) ** 2),
#             self.c * pow(2, 1/2) * np.cos(t) *
#             np.sin(t) / (1 + np.sin(t) ** 2),
#             0]
#         )

# class LineTrajectory(Trajectory):

#     def __init__(self, x, y, z, k, vel_func):
#         super().__init__(x, y, z, vel_func)
#         self.k = k
#     def position(self, t):
#         # l = pow((self.k[0] - self.start[0]) ** 2 + (self.k[1] - self.start[1]) ** 2, 1/2)
#         lx = np.linalg.norm((self.k[0] - self.start[0]))
#         ly = np.linalg.norm((self.k[1] - self.start[1]))

#         return self.start + np.array([
#             np.sin(t) * (lx) ,
#             np.sin(t) * (ly) ,
#             0])
# class CircleTrajectory3D(Trajectory):

#     def __init__(self, x, y, z, r, vel_func):
#         super().__init__(x, y, z, vel_func)
#         self.r = r

#     def velocity(self, t):
#         return 5

#     def position(self, t):
#         return self.start + np.array([
#             self.r * np.cos(t),
#             self.r * np.sin(t),
#             5 * np.cos(t)]
#         )
# class LemniscateBernoulliTrajectory3D(Trajectory):

#     def __init__(self, x, y, z, c, vel_func):
#         super().__init__(x, y, z, vel_func)
#         self.c = c

#     def position(self, t):
#         return self.start + np.array([
#             self.c * pow(2,1/2)* np.cos(t) / (1+ np.sin(t) **2),
#             self.c * pow(2,1/2) * np.cos(t) * np.sin(t) / (1+ np.sin(t) **2),
#             5 * np.cos(t)]
#         )
# class LineTrajectory3D(Trajectory):

#     def __init__(self, x, y, z, k, vel_func):
#         super().__init__(x, y, z, vel_func)
#         self.k = k

#     def position(self, t):
#         # l = pow((self.k[0] - self.start[0]) ** 2 + (self.k[1] - self.start[1]) ** 2, 1/2)
#         lx = np.linalg.norm((self.k[0] - self.start[0]))
#         ly = np.linalg.norm((self.k[1] - self.start[1]))
#         lz = np.linalg.norm((self.k[2] - self.start[2]))
#         return self.start + np.array([
#             np.sin(t) * (lx),
#             np.sin(t) * (ly),
#             np.sin(t) * (lz)])

# с использованием матрицы
class CircleTrajectory(Trajectory):

    def __init__(self, x, y, z, r, vel_func):
        super().__init__(x, y, z, vel_func)
        self.r = r

    def velocity(self, t):
        return 5

    def position(self, t):
        return self.start + np.array([
            -(self.r * np.sin(t)),
            self.r * np.cos(t),
            0]
        )
    
class LemniscateBernoulliTrajectory(Trajectory):

    def __init__(self, x, y, z, c, vel_func):
        super().__init__(x, y, z, vel_func)
        self.c = c

    def position(self, t):
        return self.start + np.array([
            -(self.c * pow(2, 1/2) * np.cos(t) *
              np.sin(t) / (1 + np.sin(t) ** 2)),
            self.c * pow(2, 1/2) * np.cos(t) / (1 + np.sin(t) ** 2),
            0]
        )

class LineTrajectory(Trajectory):

    def __init__(self, x, y, z, k, vel_func):
        super().__init__(x, y, z, vel_func)
        self.k = k
    def position(self, t):
        # l = pow((self.k[0] - self.start[0]) ** 2 + (self.k[1] - self.start[1]) ** 2, 1/2)
        lx = np.linalg.norm((self.k[0] - self.start[0]))
        ly = np.linalg.norm((self.k[1] - self.start[1]))

        return self.start + np.array([
            -(np.sin(t) * (ly)),
            np.sin(t) * (lx),
            0])        

class CircleTrajectory3D(Trajectory):

    def __init__(self, x, y, z, r, vel_func):
        super().__init__(x, y, z, vel_func)
        self.r = r

    def velocity(self, t):
        return 5

    def position(self, t):
        return self.start + np.array([
            -(self.r * np.sin(t)),
            self.r * np.cos(t),
            5 * np.cos(t)]
        )
class LemniscateBernoulliTrajectory3D(Trajectory):

    def __init__(self, x, y, z, c, vel_func):
        super().__init__(x, y, z, vel_func)
        self.c = c

    def position(self, t):
        return self.start + np.array([
            -(self.c * pow(2, 1/2) * np.cos(t) *
              np.sin(t) / (1 + np.sin(t) ** 2)),
            self.c * pow(2, 1/2) * np.cos(t) / (1 + np.sin(t) ** 2),
            5 * np.cos(t)]
        )
            
class LineTrajectory3D(Trajectory):

    def __init__(self, x, y, z, k, vel_func):
        super().__init__(x, y, z, vel_func)
        self.k = k

    def position(self, t):
        # l = pow((self.k[0] - self.start[0]) ** 2 + (self.k[1] - self.start[1]) ** 2, 1/2)
        lx = np.linalg.norm((self.k[0] - self.start[0]))
        ly = np.linalg.norm((self.k[1] - self.start[1]))
        lz = np.linalg.norm((self.k[2] - self.start[2]))
        return self.start + np.array([
            -(np.sin(t) * (ly)),
            np.sin(t) * (lx),
            np.sin(t) * (lz)])

class PointTrajectory(Trajectory):

    def __init__(self, x, y, z):
        super().__init__(x, y, z, lambda _: 0)

    def position(self, t):
        return np.zeros(3)


if __name__ == '__main__':
    # circle = CircleTrajectory(20, 20, 10, 20, lambda t: 2)
    # for i in range(10):
    #     print(circle.next())
    #     time.sleep(1)
    # line = LineTrajectory(0, 0, 10, [10,10,10 ], lambda _: 1) # with constant speed = 1
    line = LineTrajectory(0, 0, 10, [10,10,10 ], 2, lambda v: v) # with acceleration
    for i in range(100):
        print(line.next())
        time.sleep(1)
