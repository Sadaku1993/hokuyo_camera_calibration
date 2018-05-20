#include <ros/ros.h>
#include "ros/package.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ssd_ros_msgs/BoundingBox.h>
#include <ssd_ros_msgs/BoundingBoxArray.h>
#include <ssd_ros_msgs/SSD.h>
#include <map>
#include <sensor_msgs/image_encodings.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Publisher pub_person;
ros::Publisher pub_people;
ros::Time t;

string target_frame = "/camera";
string source_frame = "/laser";

bool pc_flag = false;
bool camera_flag = false;
bool image_flag = false;
bool boxes_flag = false;

sensor_msgs::PointCloud pc_;
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_);
    t = msg->header.stamp;
    pc_flag = true;
}

sensor_msgs::CameraInfoConstPtr camera_;
void cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    camera_ = msg;
    camera_flag = true;
}

sensor_msgs::ImageConstPtr image_;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    image_ = msg;
    image_flag = true;
}

ssd_ros_msgs::BoundingBoxArrayConstPtr boxes_;
void boundingboxCallback(const ssd_ros_msgs::BoundingBoxArrayConstPtr& msg)
{
    boxes_ = msg;
    boxes_flag = true;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "ssd_human_for_hokuyo");
    ros::NodeHandle n;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    //ros::Subscriber pc_sub    = n.subscribe("/velodyne_points",10,pcCallback);
    // 三分割したvelodyneの点群をsubscribe
    ros::Subscriber pc_sub    = n.subscribe("/eth221/cloud",10,pcCallback);
    ros::Subscriber cinfo_sub = n.subscribe("/camera/color/camera_info",10,cameraCallback);
    ros::Subscriber image_sub = n.subscribe("/camera/color/image_raw",10,imageCallback);
    ros::Subscriber box_sub   = n.subscribe("/realsense/ssd/BoxArray",10,boundingboxCallback);
 
    pub_person  = n.advertise<sensor_msgs::PointCloud2> ("SSD/person",10);
    pub_people = n.advertise<ssd_ros_msgs::SSD> ("SSD/people",1000);

    ros::Rate rate(20);

    while(ros::ok())
    {
        sensor_msgs::PointCloud pc_trans;
        sensor_msgs::PointCloud2 pc2_trans;
        try{
            listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), t, ros::Duration(10.0));
            listener.transformPointCloud(target_frame.c_str(), t, pc_, source_frame.c_str(), pc_trans);
            sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
        }catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        }

        if(pc_flag && camera_flag && image_flag && boxes_flag){
            printf("ALL GREEN\n");
            // human_detection(pc2_trans, camera_, image_, boxes_);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
