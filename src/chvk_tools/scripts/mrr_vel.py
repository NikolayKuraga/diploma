#!/usr/bin/env python3


import operator as op

import rospy

import geometry_msgs.msg
import mrs_msgs.msg
import nav_msgs.msg
import std_msgs.msg

import topic_mrr


def main():

    rospy.init_node('mrr_velocity', disable_signals=True)

    def convertData(data: geometry_msgs.msg.TwistWithCovarianceStamped
        ) -> nav_msgs.msg.Odometry:
        '''Convert from le first function argument's type to le output
        function argument's type (see le annotation) and then return
        it.'''

        odom = nav_msgs.msg.Odometry()

        # odom.header
        odom.header.seq = 0
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom_ned'

        # odom.child_frame_id
        odom.child_frame_id = 'base_link_frd'

        # odom.pose
        odom.pose = geometry_msgs.msg.PoseWithCovariance()

        # odom.twist
        odom.twist = data.twist

        x, y, z = op.attrgetter('x', 'y', 'z')(data.twist.twist.linear)
        odom.twist.twist.linear.x = y
        odom.twist.twist.linear.y = -x
        odom.twist.twist.linear.z = z

        x, y, z = op.attrgetter('x', 'y', 'z')(data.twist.twist.angular)
        odom.twist.twist.angular.x = y
        odom.twist.twist.angular.y = -x
        odom.twist.twist.angular.z = z

        return odom

    mrr = topic_mrr.TopicMrr(
        topic_from='/chvk/optic_flow/velocity',
        topic_to='/mavros/odometry/out',
        converter=convertData)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()

