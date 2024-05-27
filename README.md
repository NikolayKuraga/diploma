Hello!

	0. Install ROS noetic (ros-noetic-desktop)

	1. Install Gazebo
sudo apt-get install gazebo11 libgazebo11-dev

	2. Install Mavros
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras 

	3. Install Ardupilot
mkdir ~/Work
cd ~/Work
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
sudo apt-get install python-is-python3 python3-future
$PWD/modules/waf/waf-light configure --board=sitl
$PWD/modules/waf/waf-light all
./Tools/environment_install/install-prereqs-ubuntu.sh -y

	4. Install Ardupilot_gazebo
cd ~/Work
git clone https://github.com/SwiftGust/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j2
sudo make install

	5. Add to ~/.bashrc
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=~/Work/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=~/Work/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=~/Work/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=~/Work/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}

	6. Copy contents of models directory to ardupilot_gazebo/models
cd ~/Work
git clone https://github.com/Intelligent-Quads/iq_sim.git
cp -r iq_sim/models ardupilot_gazebo

	7. Install balloon
cd ~/Work
git clone https://gitlab.visrobo.com/lkz-uav/balloon.git
mv balloon balloon_ws
cp balloon_ws/worlds/test.world ardupilot_gazebo/worlds
cd balloon_ws
catkin init
catkin_make

	8. Change paths (if you need) in files (for example replace ~/ardupilot/ with ~/Work/ardupilot)
cd ~/Work
nano balloon_ws/ardupilot.sh

	9. Paste this to /opt/ros/noetic/share/gazebo_ros/launch/multi.launch and change any /home/alex in it to /home/<your user>/Work
<?xml version="1.0"?>
<launch>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="extra_gazebo_args" default=""/>
  <arg name="gui" default="true"/>
  <arg name="recording" default="false"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>
  <arg name="physics" default="ode"/>
  <arg name="verbose" default="true"/>
  <arg name="output" default="screen"/>
  <arg name="world_name" default="/home/alex/ardupilot_gazebo/worlds/test.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->
  <arg name="respawn_gazebo" default="false"/>
  <arg name="use_clock_frequency" default="false"/>
  <arg name="pub_clock_frequency" default="100"/>
  <arg name="enable_ros_network" default="true" />
  <arg name="server_required" default="false"/>
  <arg name="gui_required" default="false"/>

  <!-- set use_sim_time flag -->
  <param name="/use_sim_time" value="$(arg use_sim_time)"/>

  <!-- set command arguments -->
  <arg unless="$(arg paused)" name="command_arg1" value=""/>
  <arg 	if="$(arg paused)" name="command_arg1" value="-u"/>
  <arg unless="$(arg recording)" name="command_arg2" value=""/>
  <arg 	if="$(arg recording)" name="command_arg2" value="-r"/>
  <arg unless="$(arg verbose)" name="command_arg3" value=""/>
  <arg 	if="$(arg verbose)" name="command_arg3" value="--verbose"/>
  <arg unless="$(arg debug)" name="script_type" value="gzserver"/>
  <arg 	if="$(arg debug)" name="script_type" value="debug"/>

  <!-- start gazebo server-->
  <group if="$(arg use_clock_frequency)">
	<param name="gazebo/pub_clock_frequency" value="$(arg pub_clock_frequency)" />
  </group>
  <group>
	<param name="gazebo/enable_ros_network" value="$(arg enable_ros_network)" />
  </group>
  <node name="gazebo" pkg="gazebo_ros" type="$(arg script_type)" respawn="$(arg respawn_gazebo)" output="$(arg output)"
  args="$(arg command_arg1) $(arg command_arg2) $(arg command_arg3) -e $(arg physics) $(arg extra_gazebo_args) $(arg world_name)"
  required="$(arg server_required)" />
  <!-- start gazebo client -->
  <group if="$(arg gui)">
	<node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false" output="$(arg output)" args="$(arg command_arg3)"
	required="$(arg gui_required)"/>
  </group>
</launch>

	10. To run: ./ardupilot.sh, gazebo.sh, mavros.sh, st.sh OR

	10.1. To run execute optic_flow_demo.sh like this:
./optic_flow_demo.sh <path-to-directory-where-balloon_ws-directory-is>
