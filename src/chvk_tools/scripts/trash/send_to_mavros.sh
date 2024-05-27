#!/bin/sh


send_continuously_increasing_odometry() {
    while : ; do
        TMP_NUM=$(($TMP_NUM+1))
        rostopic pub --rate=50 /mavros/odometry/out nav_msgs/Odometry "
header:
  auto
child_frame_id: '11'
pose:
  pose: { position: {x: $TMP_NUM, y: 0., z: 0.}, orientation: {x: 0., y: 0., z: 0., w: 1.} }
  covariance: [
    0.001, 0.0,   0.0,     0.0,     0.0,     0.0,
    0.0,   0.001, 0.0,     0.0,     0.0,     0.0,
    0.0,   0.0,   1000000, 0.0,     0.0,     0.0,
    0.0,   0.0,   0.0,     1000000, 0.0,     0.0,
    0.0,   0.0,   0.0,     0.0,     1000000, 0.0,
    0.0,   0.0,   0.0,     0.0,     0.0,     1000]
twist:
  twist: { linear: {x: 0., y: 0., z: 0.}, angular: {x: 0., y: 0., z: 0.} }
  covariance: [
    0.001, 0.0,   0.0,     0.0,     0.0,     0.0,
    0.0,   0.001, 0.0,     0.0,     0.0,     0.0,
    0.0,   0.0,   1000000, 0.0,     0.0,     0.0,
    0.0,   0.0,   0.0,     1000000, 0.0,     0.0,
    0.0,   0.0,   0.0,     0.0,     1000000, 0.0,
    0.0,   0.0,   0.0,     0.0,     0.0,     1000]
" &
        sleep 3
        pkill -TERM -P $$
    done
}


send_continuously_increasing_vision_speed() {
    while : ; do
        TMP_NUM=$(($TMP_NUM+1))
        rostopic pub --rate=50 /mavros/vision_speed/speed_twist_cov geometry_msgs/TwistWithCovarianceStamped "
header:
  auto
twist:
  twist: { linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0} }
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
" &
        sleep 3
        pkill -TERM -P $$
    done
}


send_continuously_increasing_odometry
# send_continuously_increasing_vision_speed


exit 0
