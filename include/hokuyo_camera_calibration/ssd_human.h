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

#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>

#include <map>
#include <sensor_msgs/image_encodings.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Publisher pub_person;
ros::Publisher pub_people;
ros::Publisher pub_diagnostic;
ros::Time t;

// Sensor Frame
string TARGET_FRAME;
string SOURCE_FRAME;
// Subscribe Topic
string LASER_TOPIC;
string IMAGE_TOPIC;
string CAMERA_INFO;
string BBOX_TOPIC;
// Publish Topic
string OUTPUT_CLOUD;
string OUTPUT_BBOX;
string DIAGNOSTIC;

bool pc_flag = false;
bool camera_flag = false;
bool image_flag = false;
bool boxes_flag = false;

sensor_msgs::PointCloud2ConstPtr pc2_;
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pc2_ = msg;
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

amsl_recog_msgs::ObjectInfoArrayConstPtr boxes_;
void boundingboxCallback(const amsl_recog_msgs::ObjectInfoArrayConstPtr& msg)
{
    boxes_ = msg;
    boxes_flag = true;
}

