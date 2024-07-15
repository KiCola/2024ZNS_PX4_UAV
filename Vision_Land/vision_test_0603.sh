#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/uav_ws/devel/setup.bash
source ~/smart_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600" gcs_url:="udp://@192.168.76.193"; exec bash"' \
--tab -e  'bash -c "sleep 5; roslaunch usb_cam usb_cam-test.launch; exec bash"' \
--tab -e  'bash -c "sleep 5; roslaunch darknet_ros darknet_ros_yolov3t_test.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch realsense2_camera rs_t265.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch rplidar_ros rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command px4_vel_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command vision_test_0603.launch; exec bash"' \
