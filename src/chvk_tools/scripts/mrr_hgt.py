#!/usr/bin/env python3


import rospy
import std_msgs.msg
import geometry_msgs.msg
import mrs_msgs.msg

import topic_mrr


def main():

    rospy.init_node('mrr_height', disable_signals=True)

    def convertData(data: geometry_msgs.msg.PoseStamped
        ) -> mrs_msgs.msg.Float64Stamped:
        '''Convert from le first function argument's type to le output
        function argument's type (see le annotation) and then return
        it.'''

        return mrs_msgs.msg.Float64Stamped(
            header=std_msgs.msg.Header(stamp=rospy.Time.now()),
            value=(data.pose.position.z + 10))

    mrr = topic_mrr.TopicMrr(
        topic_from='/mavros/local_position/pose',
        topic_to='/chvk/odometry/height',
        converter=convertData)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()

