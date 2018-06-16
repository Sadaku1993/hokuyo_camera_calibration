#!/bin/bash

hander()
{
    sleep 1
}

trap hander SIGINT

TIME=$(date +%Y-%m-%d-%H-%M-%S)
HOKUYO="/eth_221/cloud/tf"
REALSENSE="/camera/color/camera_info /camera/color/image_raw /camera/depth/camera_info /camera/depth/color/points"
TF="/tf /tf_static"

echo $TIME &
echo $HOKUYO &
echo $REALSENSE &
echo $TF &

/opt/ros/kinetic/bin/rosbag record $HOKUYO $REALSENSE $TF -O /home/amsl/$TIME.bag
