#!/bin/bash
# source /home/dji/dji_n3_controller/devel/setup.bash
# source /home/dji/catkin_ws/devel/setup.bash
#roscore & sleep 1;

roslaunch djiros djiros.launch & sleep 5s;
roslaunch vrpn_client_ros sample.launch & sleep 2s;


#rviz -d /home/dji/tfes/src/VINS-Fusion/config/vins_rviz_config.rviz
roslaunch n3ctrl ctrl_md.launch & sleep 5s;

roslaunch ego_planner real.launch & sleep 2s

# rosbag record -a


