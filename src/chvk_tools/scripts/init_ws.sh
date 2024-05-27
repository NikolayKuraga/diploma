#!/bin/sh


### VARS

CATKIN_WS_PATH=
CATKIN_WS_USER='>w<'
#ROS_DISTRO='noetic'


### CHECKS

if [ $(id -u) -ne 0 ]; then
	echo 'run the script by root'
	exit 0
fi

if [ $# -lt 2 ]; then
	echo 'provide a path to catkin workspace and its owner user like this:'
	echo '$ ./le_script.sh /path/to/workspace le_user'
	exit 0
fi

if [ ! -d "$1" ]; then
	echo 'provided directory does not exist or is not a directory'
	exit 0
fi
CATKIN_WS_PATH=$1

id -u "$2" 1>/dev/null 2>&1
if [ $? -ne 0 ]; then
	echo 'provided user does not exist'
	exit 0
fi
CATKIN_WS_USER=$2


### ACTION

# install mavproxy
apt-get update -q
apt-get install -q -y python3-dev python3-opencv python3-wxgtk4.0 \
	python3-pip python3-matplotlib python3-lxml python3-pygame
python3 -m pip install --system mavproxy

# add CTU MRS stable repository (PPA)
apt-get install -q -y curl
curl https://ctu-mrs.github.io/ppa-stable/add_ppa.sh | bash
cat <<EOT > /etc/apt/preferences.d/ctu-mrs-stable-preferences
Package: *
Pin: release o=ctu-mrs,l=stable,c=
Pin-Priority: 100
EOT

rosdep fix-permissions
su - $CATKIN_WS_USER -c "rosdep fix-permissions"
su - $CATKIN_WS_USER -c "rosdep update"

# init the workspace, install dependencies de la packages
su - $CATKIN_WS_USER -c "catkin init -w ${CATKIN_WS_PATH}"
rosdep install --rosdistro noetic -y --from-paths ${CATKIN_WS_PATH}/src
