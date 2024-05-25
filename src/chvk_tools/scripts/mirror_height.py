#!/usr/bin/env python3


import rospy
import std_msgs.msg
import geometry_msgs.msg
import mrs_msgs.msg

from topicmirror import TopicMirror


if __name__ == '__main__':

    rospy.init_node('mirror_height')

    def convertData(data: geometry_msgs.msg.PoseStamped
        ) -> mrs_msgs.msg.Float64Stamped:
        '''Convert from geometry_msgs.msg.PoseStamped type to
        mrs_msgs.msg.Float64Stamped type.'''

        float64Stamped = mrs_msgs.msg.Float64Stamped(
            header=std_msgs.msg.Header(stamp=rospy.Time.now()),
            value=data.pose.position.z)

        return float64Stamped

    mrr = TopicMirror(
        '/mavros/local_position/pose',
        '/chvk/odometry/height',
        convertData)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
