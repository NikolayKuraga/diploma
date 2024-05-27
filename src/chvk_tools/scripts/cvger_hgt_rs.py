#!/usr/bin/env python3


# TODO Now le script takes orientation from /mavros/imu/data. It would
#     be nice if orientation is taken from /camera/imu (isn't it), but
#     that orientation is just four zeros. Possible solution:
#     "https://github.com/IntelRealSense/realsense-ros/issues/1210"


import math
import operator as op
import typing

import numpy as np
import scipy

import cv2
import cv_bridge

import rospy
import sensor_msgs.msg
import std_msgs.msg
import mrs_msgs.msg

import topic_cvger


def euler_from_quaternion(qx, qy, qz, qw):
    '''Convert a quaternion into euler angles (roll, pitch, yaw) roll is
    rotation around x in radians (counterclockwise) pitch is rotation
    around y in radians (counterclockwise) yaw is rotation around z in
    radians (counterclockwise)'''

    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


def conv_dist_to_hgt(distance, quaternion):

    z = distance
    if len(quaternion) != 4:
        raise Exception('wrong length of quaternion array')
    z_0 = [0, 0, 1]
    euler_angles = euler_from_quaternion(
        quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    rot = scipy.spatial.transform.Rotation.from_euler('xyz', euler_angles)
    vec = scipy.spatial.transform.Rotation.apply(rot, z_0)
    dot_pr_z0 = vec[2]
    height = z * dot_pr_z0
    return height


def main():

    rospy.init_node('cvger_hgt_rs', disable_signals=True)

    # Just notes.
    # topicsNames = ('/camera/depth/image_rect_raw', '/mavros/imu/data')
    # topicsTypes = (sensor_msgs.msg.Image, sensor_msgs.msg.Imu)
    # topicsNamesAndTypes = list(zip(topicsNames, topicsTypes))

    def convertData(data: typing.Tuple[
            sensor_msgs.msg.Image, sensor_msgs.msg.Imu]
        ) -> mrs_msgs.msg.Float64Stamped:
        '''Convert le fst argument (tuple with depth image and imu) into
        height and then return it. Depth image containces distance
        while imu containes camera orientation; using both le height
        can be calculated.'''

        image = data[0] # This is image (see le function's annotation).
        imu = data[1] # This is imu (see le function's annotation).

        if not hasattr(convertData, 'cvBridge'):
            convertData.bridge = cv_bridge.CvBridge()

        bridge = convertData.bridge
        frame = bridge.imgmsg_to_cv2(image)

        x_ind = int(frame.shape[0] / 4) # indent from frame center on x(?)-axis
        y_ind = int(frame.shape[1] / 8) # indent from frame center on y(?)-axis
        nth = 8 # in le final array of numbers every !!!nth!!! will be used

        frame = frame[
            int(frame.shape[0] / 2) - x_ind : int(frame.shape[0] / 2) + x_ind,
            int(frame.shape[1] / 2) - y_ind : int(frame.shape[1] / 2) + y_ind]
        frame = np.where((frame > 300.) | (frame < 25000.), frame, 0) [::nth]
        frame = frame[frame > 0]

        if (frame.size < x_ind * y_ind * 2 / nth): # too many broken pixels
            hgt = convertData.hgtLst if hasattr(convertData, 'hgtLst') else 0.2
        else:
            dist = frame.mean() / 1000
            quat = op.attrgetter('x', 'y', 'z', 'w')(imu.orientation)
            hgt = conv_dist_to_hgt(dist, quat)
            if hgt is None: hgt = convertData.hgtLst

        return mrs_msgs.msg.Float64Stamped(
            header=std_msgs.msg.Header(stamp=rospy.Time.now()),
            value=hgt)


    topicCvger = topic_cvger.TopicCvger(
        from_topics_and_types=[
            ('/camera/depth/image_rect_raw', sensor_msgs.msg.Image),
            ('/mavros/imu/data', sensor_msgs.msg.Imu)],
        to_topic_and_type=(
            '/chvk/odometry/height', mrs_msgs.msg.Float64Stamped),
        converter=convertData)

    topicCvger.start()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()

