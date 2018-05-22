/*

author Yudai Sadakuni

velodyne座標系の点群をzed座標系に変換

*/

#include <ros/ros.h>
#include "ros/package.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace std;

ros::Publisher pub;
ros::Time t;

string target_frame = "/camera_color_optical_frame";
string source_frame = "/laser";


sensor_msgs::PointCloud pc_;
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_);
    t = msg->header.stamp;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "hokuyo_pointcloud_transform");
    ros::NodeHandle n;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber pc_sub    = n.subscribe("/eth_221/cloud",10,pcCallback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/eth_221/cloud/tf", 10);
    ros::Rate rate(20);

    while(ros::ok())
    {
        sensor_msgs::PointCloud pc_trans;
        sensor_msgs::PointCloud2 pc2_trans;
        try{
            listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), t, ros::Duration(1.0));
            listener.transformPointCloud(target_frame.c_str(), t, pc_, source_frame.c_str(), pc_trans);
            sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
        }catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        }

        pub.publish(pc2_trans);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
