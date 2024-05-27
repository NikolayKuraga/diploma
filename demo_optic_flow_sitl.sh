#!/bin/sh


### VARS

SUR_CATKIN_WS_PATH=
TIME_TO_SLEEP=5


### CHECKS

if [ $# -lt 1 ]; then
	echo 'provide a path to directory that contains ardupilot and'\
		'balloon_ws catkin workspaces like this:'\
		'$ ./le_script.sh /path/to/dirthatcontains/da/workspaces'
	exit 0
fi

if [ ! -d "$1" ]; then
	echo 'provided directory does not exist or is not a directory'
	exit 0
fi

if [ ! -d "$1/ardupilot" ]; then
	echo 'provided directory contains no ardupilot catkin workspace'
	exit 0
elif [ ! -d "$1/balloon_ws" ]; then
	echo 'provided directory contains no balloon_ws catkin workspace'
	exit 0
fi
SUR_CATKIN_WS_PATH=$1
cd "$SUR_CATKIN_WS_PATH" || { echo 'provided path does not exist'; exit 1; }


### TRAPS

stopBackgndPrcs() {
	echo 'Stopping all background processes'
	pkill --signal INT -P $$

	wait
	#kill -s KILL -- -$$ # idk
	#kill -s INT $(jobs -p) # not for sh (bourne shell / dash)
	exit 0
}
trap stopBackgndPrcs INT QUIT HUP TERM


### ACTION

# start roscore
. /opt/ros/noetic/setup.sh
roscore &
sleep $TIME_TO_SLEEP

# start SITL ardupilot and mavproxy
bash -c "
source /opt/ros/noetic/setup.bash
source $SUR_CATKIN_WS_PATH/ardupilot/Tools/completion/completion.bash
$SUR_CATKIN_WS_PATH/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
	-f gazebo-iris \
	-m '--daemon --streamrate=50 --console --no-state --mav20 --out=udp:127.0.0.1:14551' -I0" &

# start gazebo
. /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$SUR_CATKIN_WS_PATH/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=$SUR_CATKIN_WS_PATH/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=$SUR_CATKIN_WS_PATH/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=$SUR_CATKIN_WS_PATH/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
. $SUR_CATKIN_WS_PATH/balloon_ws/devel/setup.sh
roslaunch demo gazebo.launch &
sleep $TIME_TO_SLEEP

# start mavros
roslaunch chvk_tools apm.launch &
sleep $TIME_TO_SLEEP

# run the square
roslaunch demo square.launch &
sleep $TIME_TO_SLEEP

# start mirroring height from /mavros/local_position/pose to /chvk/odometry/height
rosrun chvk_tools mirror_height.py &
sleep $TIME_TO_SLEEP

# start mrs optic flow
roslaunch chvk_tools flow_mrs_usb.launch &
sleep $TIME_TO_SLEEP

# # start loggers
# rosrun stat logger_gazebo.py
# sleep $TIME_TO_SLEEP

wait
exit 0
