from strategy import *

import time

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, AttitudeTarget


class VelocityPublisher:

    def __init__(self):
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.twist = TwistStamped()

    def publish(self, cmd):
        self.twist.header.stamp = rospy.Time.now()
        self.twist.twist.linear.x = cmd[0]
        self.twist.twist.linear.y = cmd[1]
        self.twist.twist.linear.z = cmd[2]
        self.twist.twist.angular.z = cmd[3]
        self.vel_pub.publish(self.twist)


class AccelerationPublisher:

    def __init__(self):
        self.publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.pos_target = PositionTarget()
        self.pos_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.pos_target.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_YAW

    def publish(self, cmd):
        self.pos_target.header.stamp = rospy.Time.now()
        self.pos_target.acceleration_or_force.x = cmd[0]
        self.pos_target.acceleration_or_force.y = cmd[1]
        self.pos_target.acceleration_or_force.z = cmd[2]
        self.pos_target.yaw_rate = cmd[3]
        self.publisher.publish(self.pos_target)


class PositionPublisher:

    def __init__(self):
        self.publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.pos_target = PositionTarget()
        self.pos_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.pos_target.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_YAW

    def publish(self, cmd):
        self.pos_target.header.stamp = rospy.Time.now()
        self.pos_target.position.x = cmd[0]
        self.pos_target.position.y = cmd[1]
        self.pos_target.position.z = cmd[2]
        self.pos_target.yaw_rate = cmd[3]
        self.publisher.publish(self.pos_target)


class ThrustPublisher:

    def __init__(self):
        self.publisher = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.attitude = AttitudeTarget()

    def publish(self, cmd):
        target_raw_attitude = AttitudeTarget()
        target_raw_attitude.header.stamp = rospy.Time.now()
        target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        target_raw_attitude.body_rate.x = cmd[0]
        target_raw_attitude.body_rate.y = cmd[1]
        target_raw_attitude.body_rate.z = cmd[2]
        target_raw_attitude.thrust = cmd[3]
        self.publisher.publish()


class MovementController:

    def __init__(self, strategy):
        self.strategy = strategy
        self.strategy_type = strategy.get_type()
        self.attack_publisher = self.create_publisher()
        self.search_publisher = VelocityPublisher()
        self.reset = False

    def update(self, pos_info, pos_ref, image_center):
        if pos_ref[0] == 0 and pos_ref[1] == 0:
            cmd = self.strategy.search_cmd()
            self.search_publisher.publish(cmd)
            self.reset = True
        else:
            cmd = self.strategy.attack_cmd(pos_info, pos_ref, image_center, self.reset)
            self.attack_publisher.publish(cmd)
            self.reset = False

    def create_publisher(self):
        if self.strategy_type == StrategyType.VELOCITY:
            return VelocityPublisher()
        elif self.strategy_type == StrategyType.ACCELERATION:
            return AccelerationPublisher()
        else:
            return ThrustPublisher()

