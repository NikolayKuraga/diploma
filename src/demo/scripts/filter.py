#!/usr/bin/python3

import numpy as np

import rospy
from std_msgs.msg import Float32MultiArray, UInt64
import cv2


class KalmanFilter(object):

    def __init__(self, bx, by, dt):
        # delta T - time difference
        self.dt = dt

        # Previous state vector {x,y,vx,vy, ax, ay}
        self.xkp = np.array([bx, by, 0, 0, 0, 0])
        self.xk = self.xkp

        # Measured vector
        self.xm = np.array([0, 0])

        # State Transition Matrix
        self.F = np.eye(self.xkp.shape[0])
        self.F[0][2] = self.dt
        self.F[0][4] = (self.dt ** 2) / 2
        self.F[1][3] = self.dt
        self.F[1][5] = (self.dt ** 2) / 2
        self.F[2][4] = self.dt
        self.F[3][5] = self.dt

        # Initial Process Covariance Matrix
        self.Pkp = np.eye(self.xkp.shape[0])

        # Process Noise Covariance Matrix
        self.Qk = 100 * np.eye(self.xkp.shape[0])

        # Control Matrix
        # self.Bk = np.array([(self.dt**2)/2, self.dt])

        # Control Vector - initialised to acceleration due to gravity
        # self.uk = np.array([-9.8])

        # Sensor Matrix
        self.Hk = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0]])

        # Measurement covariance matrix
        self.R = 10 * np.eye(self.xm.shape[0])

    def predict(self):
        # Predicted Vector
        self.xk = self.F @ self.xkp  # + self.Bk @ self.uk

        # Setting Previously predicted to current for next frame
        self.xkp = self.xk

        # Getting the updated process cov matrix
        self.Pkp = self.F @ self.Pkp @ self.F.T

        return self.xk

    def update(self, bx, by):
        # Update Measurement Vector
        self.xm = np.array([bx, by])
        # Kalman Gain
        A = self.Hk @ self.Pkp @ self.Hk.T + self.R
        K = np.linalg.inv(A)
        K = self.Hk.T @ K
        K = self.Pkp @ K

        # Most Likely state Vector
        self.xk = self.xk + K @ (self.xm - self.Hk @ self.xk)

        # Updated Process covariance Matrix
        self.Pkp = self.Pkp - K @ self.Hk @ self.Pkp

        return self.xk


class TestFilter:

    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def update(self, cx, cy):
        measured = np.array([[np.float32(cx)], [np.float32(cy)]])
        predicted = self.kf.predict()
        self.kf.correct(measured)
        return predicted

    def predict(self):
        return self.kf.predict()


class FilterNode:

    def __init__(self):
        self.publisher = rospy.Publisher('/filter/pos_image', Float32MultiArray, queue_size=1)
        self.msg = Float32MultiArray()
        self.hz = 60
        self.rate = rospy.Rate(self.hz)

        self.filter = None
        self.state = 0
        self.data = [0.0] * 5
        self.cx_prev = 0.0
        self.cy_prev = 0.0
        self.has_update = False
        self.cnt = 0

    def start(self):
        rospy.Subscriber('/detector/pos_image', Float32MultiArray, self.detector_cb)
        # while not rospy.is_shutdown():
        #     if self.state == 0:
        #         print("Detection lost")
        #         pass
        #     elif self.state == 1:
        #         print("Filter reset")
        #         self.filter = KalmanFilter(self.cx, self.cy, 1 / self.hz)
        #         self.state = 2
        #     else:
        #         xp = [0.] * 6
        #         if self.has_update:
        #             xp = self.filter.update(self.cx, self.cy)
        #             self.cnt += 1
        #             self.has_update = False
        #         elif self.cnt > 10:
        #             xp = self.filter.predict()
        #         self.publish(xp)
        #
        #     self.rate.sleep()
        while not rospy.is_shutdown():
            xp = [0.] * 2
            if self.state == 0:
                # Object is not visible
                pass
            elif self.state == 1:
                # Initializing filter at first observation
                self.filter = TestFilter()
                self.state = 2
            else:
                if self.has_update:
                    xp = self.filter.update(self.data[0], self.data[1])[:2]
                    self.has_update = False
                else:
                    xp = self.filter.predict()[:2]
            self.data[0] = xp[0]
            self.data[1] = xp[1]
            self.publish(self.data)
            self.rate.sleep()

    def detector_cb(self, data):
        self.data = list(data.data)
        self.has_update = True
        if self.data[0] == 0 and self.data[1] == 0:
            self.state = 0
        elif self.state == 0:
            self.state = 1

    def publish(self, data):
        self.msg.data = data
        self.publisher.publish(self.msg)


if __name__ == '__main__':
    rospy.init_node('filter_node', anonymous=True)
    filter_node = FilterNode()
    filter_node.start()
    while not rospy.is_shutdown():
        rospy.spin()
