#!/bin/sh


### VARS

SUR_CATKIN_WS_PATH=$1


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
	#kill -s INT $(jobs -p) # doesn't work with sh (bourne shell / dash)
	exit 0
}
trap stopBackgndPrcs INT QUIT HUP TERM


### ACTION

# start roscore
roscore &
sleep 3

# start ardupilot and mavproxy
bash -c "
source /opt/ros/noetic/setup.bash
source $SUR_CATKIN_WS_PATH/ardupilot/Tools/completion/completion.bash
$SUR_CATKIN_WS_PATH/ardupilot/Tools/autotest/sim_vehicle.py\
       	-v ArduCopter -f gazebo-iris -m '--mav20 --daemon' -I0" &

# start gazebo
. /usr/share/gazebo/setup.sh
. $SUR_CATKIN_WS_PATH/balloon_ws/devel/setup.sh
export GAZEBO_MODEL_PATH=$SUR_CATKIN_WS_PATH/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=$SUR_CATKIN_WS_PATH/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=$SUR_CATKIN_WS_PATH/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=$SUR_CATKIN_WS_PATH/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
roslaunch demo gazebo.launch &
sleep 10

#echo $(jobs -p)
#echo $(ps)

# start mavros
. /opt/ros/noetic/setup.sh
roslaunch mavros apm.launch fcu_url:='udp://127.0.0.1:14550@14550' &
sleep 10

# start chasing da ball
. $SUR_CATKIN_WS_PATH/balloon_ws/devel/setup.sh
roslaunch demo chase.launch &

wait
exit 0
