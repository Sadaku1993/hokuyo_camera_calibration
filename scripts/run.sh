#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+0+0 &
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch hokuyo_camera_calibration run.launch" --geometry=50x12+0+250 &
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/hokuyo_camera_calibration.rviz" --geometry=50x12+0+500 &

