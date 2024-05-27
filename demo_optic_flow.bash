#!/usr/bin/env bash


# HELLO1 Le script strives to run MRS Optical Flow using: RealSense
# D455 and some MAVLink connection (UART, USB, other) with FCU.


# TODO Would be better to run any script (including this one) in
#     Bourne Shell (shizlo?). However, when we build catkin workspace
#     using "catkin build" command from python3-catkin-tools, works
#     sourcing only devel/setup.bash (not devel/setup.sh) (le issue:
#     "https://github.com/catkin/catkin_tools/issues/376").


# TODO Would be wonderful to add command-line options handler and some
#     command-line options like
#     "--mavproxy_remote_udp_ip_port=<le_port>".


# TODO Would be wonderful to write some "mavproxy|mavros|mrs|converter
#     has been executed and now works just fine"-checker to know
#     exactly that the latest stage has started up and didn't
#     fail. This is meant to be a alternative to "sleep
#     $TIME_TO_SLEEP".


# TODO Would be wonderful to do something about
#     "chvk/bluefox_optflow_optical" tf2 frames in MRS OF sources,
#     like bind 'em somehow with main tf2 tree of le project.


### FUNS

stopBackgndPrcs() {
    echo 'Stopping all background processes'
    pkill --signal TERM -P $$

    # pkill --signal KILL -P $$
    # kill -s KILL -- -$$ # idk
    # kill -s INT $(jobs -p) # Doesn't work in Bourne Shell.

    wait
    exit 0
}


### VARS

TIME_TO_SLEEP=3

echo # A bit of info.

if [ -z ${CHVK_DEVMAV+x} ]; then
    CHVK_DEVMAV='/dev/ttyTHS0,921600'
    echo "i: \$CHVK_DEVMAV was set    to ${CHVK_DEVMAV} !"
else
    echo "i: \$CHVK_DEVMAV            is ${CHVK_DEVMAV} !"
fi

if [ -z ${CHVK_WS+x} ]; then
    CHVK_WS="/home/$(whoami)/Work/balloon_ws/"
    echo "i: \$CHVK_WS was set        to ${CHVK_WS} !"
else
    echo "i: \$CHVK_WS                is ${CHVK_WS} !"
fi

if [ -z ${CHVK_PKG+x} ]; then
    CHVK_PKG=${CHVK_WS}/src/chvk_tools/
    echo "i: \$CHVK_PKG was set       to ${CHVK_PKG} !"
else
    echo "i: \$CHVK_PKG               is ${CHVK_PKG} !"
fi

if [ -z ${QGC_IP+x} ]; then
    QGC_IP='0.0.0.0'
    echo "i: \$QGC_IP was set         to ${QGC_IP} !"
else
    echo "i: \$QGC_IP                 is ${QGC_IP} !"
fi

if [ -z ${DISPLAY+x} ]; then
    DISPLAY=':0.0'
    echo "i: May need \$DISPLAY, set  to ${DISPLAY} !"
else
    echo "i: \$DISPLAY                is ${DISPLAY} !"
fi


echo
echo 'Wanna change some CHVK_* variable? Next time before execution do:'
echo ' $ export CHVK_*="your value"'

echo # END A bit of info.


### CHECKS

cd "$CHVK_WS" || \
    { echo "\nProvided catkin workspace does not exist\n"; exit 1; }


### TRAPS

trap stopBackgndPrcs INT QUIT HUP TERM


### ACTION


# Source all needed catkin workspaces.
. /opt/ros/noetic/setup.bash
. $CHVK_WS/devel/setup.bash


# echo '[i] Start roscore.'
# # BTW Seems to be de trop cause of le following launch-file executing.
# roscore >/dev/null 2>&1 &
# sleep $TIME_TO_SLEEP


echo '[i] Launch realsense.'
roslaunch chvk_tools rs_camera.launch \
    initial_reset:=true \
    enable_color:=false \
    enable_infra1:=true \
    infra_width:=848 \
    infra_height:=480 \
    infra_fps:=60 \
    enable_depth:=true \
    enable_gyro:=true `# dummy commentary (WoW!)` \
    enable_accel:=true `# another dummy commentary` \
    unite_imu_method:=linear_interpolation \
    >/dev/null 2>&1 &
sleep $TIME_TO_SLEEP


echo '[i] Start mavproxy.'
mavproxy.py --daemon --mav20 --logfile=/tmp/mavproxy.log \
    --master="$CHVK_DEVMAV" \
    --out="udp:${QGC_IP}:14550" `# for QGC` \
    --out='udp:0.0.0.0:14551' \
    --out='udp:0.0.0.0:14552' \
    >/dev/null 2>&1 &
sleep $TIME_TO_SLEEP


echo '[i] Launch mavros.'
roslaunch chvk_tools px4.launch \
    fcu_url:="udp://0.0.0.0:14551@" \
    >/dev/null 2>&1 &
sleep $TIME_TO_SLEEP


echo '[i] Try to reboot FCU (just in case).'
rosservice call /mavros/cmd/command "{
broadcast: false,
command: 246,
confirmation: 0,
param1: 1,
param2: 0.0,
param3: 0.0,
param4: 0.0,
param5: 0.0,
param6: 0.0,
param7: 0.0}" \
    >/dev/null 2>&1 &
sleep $((${TIME_TO_SLEEP}*2))


echo '[i] Run relaying "/mavros/imu/data_raw" to "/imu/data_raw".'
# "imu_filter_madgwick" from "imu_tools" requires "/imu/data_raw" topic.
rosrun topic_tools relay /mavros/imu/data_raw /imu/data_raw >/dev/null 2>&1 &


echo '[i] Launch imu_filter_madgwick.launch.'
# imu_filter_madgwick is supposed to provide more high-quality imu data
# data than "/mavros/imu/data" topic.
roslaunch chvk_tools imu_filter_madgwick.launch &


echo '[i] Run publishing dummy odometry to FCU.'
# In order to run MRS OF we need to provide for MRS's
# "odometry/orientation" topic some odometry data from smth like
# "/mavros/odometry/in" or "/mavros/local_position/odom", but because of
# disabled GPS these are silent. So we will have been sending dummy
# odometry data to "/mavros/odometry/out" for a while to make FCU feel
# comfortable about its local position...
rostopic pub --rate=50 /mavros/odometry/out nav_msgs/Odometry "
header:
  seq: 0
  stamp: now
  frame_id: 'odom_ned'
child_frame_id: 'base_link_frd'
pose:
  pose:
    position: {x: 0., y: 0., z: 0.}
    orientation: {x: 0., y: 0., z: 0., w: 1.}
  covariance: [
    0.001,   0.0,     0.0,     0.0,     0.0,     0.0,
    0.0,     0.001,   0.0,     0.0,     0.0,     0.0,
    0.0,     0.0,     1000000, 0.0,     0.0,     0.0,
    0.0,     0.0,     0.0,     1000000, 0.0,     0.0,
    0.0,     0.0,     0.0,     0.0,     1000000, 0.0,
    0.0,     0.0,     0.0,     0.0,     0.0,     1000]
twist:
  twist:
    linear: {x: 0., y: 0., z: 0.}
    angular: {x: 0., y: 0., z: 0.}
  covariance: [
    0.001,   0.0,     0.0,     0.0,     0.0,     0.0,
    0.0,     0.001,   0.0,     0.0,     0.0,     0.0,
    0.0,     0.0,     1000000, 0.0,     0.0,     0.0,
    0.0,     0.0,     0.0,     1000000, 0.0,     0.0,
    0.0,     0.0,     0.0,     0.0,     1000000, 0.0,
    0.0,     0.0,     0.0,     0.0,     0.0,     1000]
" >/dev/null 2>&1 &
PID_DMY_ODOM=$!
# sleep $TIME_TO_SLEEP


# echo '[i] Start publishing dummy OF to FCU.'
# # In order to run MRS OF we need to provide for MRS's
# # "odometry/orientation" topic some odometry data from smth like
# # "/mavros/odometry/in" or "/mavros/local_position/odom", but because
# # of disabled GPS these are silent. So we will have been sending dummy
# # OF data to "/mavros/px4flow/raw/send" for a while to make FCU feel
# # comfortable about its local position...
# rostopic pub --rate=1 /mavros/px4flow/raw/send mavros_msgs/OpticalFlowRad \
# '{
#   header: { seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: '' },
#   integration_time_us: 0,
#   integrated_x: 0.0,
#   integrated_y: 0.0,
#   integrated_xgyro: 0.0,
#   integrated_ygyro: 0.0,
#   integrated_zgyro: 0.0,
#   temperature: 0,
#   quality: 0,
#   time_delta_distance_us: 0,
#   distance: 0.0
# }' &
# PID_DMY_OF=$!
# # sleep $TIME_TO_SLEEP


echo '[i] Run convergencing height from "/mavros/imu/data" and'
echo '    "/camera/depth/image_rect_raw" to "/chvk/odometry/height".'
rosrun chvk_tools cvger_hgt_rs.py &
# sleep $TIME_TO_SLEEP


echo '[i] Start MRS OF.'
# After a while FCU had started publishing odometry to
# "/mavros/local_position/odom". Time to start MRS OF.
roslaunch chvk_tools optic_flow.launch \
    UAV_NAME:=chvk \
    topic_camera_in:="/camera/infra1/image_rect_raw" \
    topic_camera_info_in:="/camera/infra1/camera_info" \
    topic_odometry_in:="/mavros/local_position/odom" \
    topic_uav_height_in:="/chvk/odometry/height" \
    topic_imu_in:="/mavros/imu/data" &
sleep $((${TIME_TO_SLEEP}*2))


# echo '[i] Finish dummy OF or dummy odometry to FCU.'
# echo '[i] Start mirroring MRS OF data from "/chvk/optic_flow/velocity" to'
# echo '    "/mavros/odometry/out".'
# # After a while MRS OF had started publishing OF data to
# # "/chvk/optic_flow/velocity" on its own. Time to finish befooling FCU and
# # start mirroring MRS OF data from "/chvk/optic_flow/velocity" to
# # "/mavros/odometry/out".
# kill -KILL $PID_DMY_OF >/dev/null 2>&1
# kill -KILL $PID_DMY_ODOM >/dev/null 2>&1
# unset PID_DMY_OF
# unset PID_DMY_ODOM
# rosrun chvk_tools mrr_vel.py &
# sleep $TIME_TO_SLEEP


echo '[i] Finish dummy OF or dummy odometry to FCU.'
echo '[i] Start mirroring MRS OF data from "/chvk/optic_flow/velocity" to'
echo '    "/mavros/odometry/out".'
# After a while MRS OF had started publishing OF data to
# "/chvk/optic_flow/velocity" on its own. Time to finish befooling FCU
# and start convergencing MRS OF data from "/chvk/optic_flow/velocity"
# and "/imu/data" to "/mavros/odometry/out".
kill -KILL $PID_DMY_OF >/dev/null 2>&1
kill -KILL $PID_DMY_ODOM >/dev/null 2>&1
unset PID_DMY_OF
unset PID_DMY_ODOM
rosrun chvk_tools cvger_odom.py &
sleep $TIME_TO_SLEEP


wait
exit 0


# KU FAQ or other interesting stuff.

# available mavros px4 OF topics
# /mavros/px4flow/ground_distance
# /mavros/px4flow/raw/optical_flow_rad
# /mavros/px4flow/raw/send
# /mavros/px4flow/temperature

# available MRS OF topics
# /chvk/optic_flow/active_tracker_in
# /chvk/optic_flow/allsac_chosen
# /chvk/optic_flow/max_velocity_out
# /chvk/optic_flow/points_raw_out
# /chvk/optic_flow/velocity
# /chvk/optic_flow/velocity_longrange
# /chvk/optic_flow/velocity_out_longrange_diff
# /chvk/optic_flow/velocity_stddev

# END KU FAQ
