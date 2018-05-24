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

string TARGET_FRAME = "/camera";
string SOURCE_FRAME = "/laser";
string SUBSCRIBE_TOPIC ="/cloud";
string PUBLISH_TOPIC = "/cloud_tf";

// Load Parameter
void getParams(ros::NodeHandle &n)
{
    n.getParam("TARGET_FRAME", TARGET_FRAME);
    n.getParam("SOURCE_FRAME", SOURCE_FRAME);
    n.getParam("SOUSCRINE_TOPIC", SUBSCRIBE_TOPIC);
    n.getParam("PUBLISH_TOPIC", PUBLISH_TOPIC);
}

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
    getParams(n);
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber pc_sub    = n.subscribe(SUBSCRIBE_TOPIC,10,pcCallback);
    pub = n.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 10);
    ros::Rate rate(20);

    while(ros::ok())
    {
        sensor_msgs::PointCloud pc_trans;
        sensor_msgs::PointCloud2 pc2_trans;
        try{
            listener.waitForTransform(TARGET_FRAME.c_str(), SOURCE_FRAME.c_str(), t, ros::Duration(1.0));
            listener.transformPointCloud(TARGET_FRAME.c_str(), t, pc_, SOURCE_FRAME.c_str(), pc_trans);
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
