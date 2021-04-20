#!/bin/bash
source /home/fly-vision/dji_n3_controller/devel/setup.sh
#roscore & sleep 1;

roslaunch djiros djiros.launch & sleep 5s;

# your sensor suits VINS, such as:
roslaunch realsense2_camera rs_camera.launch & sleep 10s;

rosrun vins vins_node /home/dji/tfes/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml  & sleep 5s;

rosrun loop_fusion loop_fusion_node /home/dji/tfes/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml  & sleep 5s;

#rviz -d /home/dji/tfes/src/VINS-Fusion/config/vins_rviz_config.rviz
roslaunch n3ctrl ctrl_md.launch & sleep 5s;

roslaunch bezier_planer real.launch & sleep 2s;

rosbag record -a


