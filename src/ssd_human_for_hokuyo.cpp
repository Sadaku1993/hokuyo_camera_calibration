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

int main(int argc, char**argv)
{
    ros::init(argc, argv, "ssd_human_for_hokuyo");
    ros::NodeHandle n;

    n.getParam("/ssd_human/target_frame",   TARGET_FRAME );
    n.getParam("/ssd_human/source_frame",   SOURCE_FRAME );
    n.getParam("/ssd_human/laser_topic",    LASER_TOPIC );
    n.getParam("/ssd_human/image_topic",    IMAGE_TOPIC );
    n.getParam("/ssd_human/camera_info",    CAMERA_INFO );
    n.getParam("/ssd_human/bbox_topic",     BBOX_TOPIC );
    n.getParam("/ssd_human/output_cloud",   OUTPUT_CLOUD );
    n.getParam("/ssd_human/output_bbox",    OUTPUT_BBOX );
    n.getParam("/ssd_human/diagnostic",     DIAGNOSTIC);

    ros::Subscriber pc_sub    = n.subscribe(LASER_TOPIC,10,pcCallback);
    ros::Subscriber cinfo_sub = n.subscribe(CAMERA_INFO,10,cameraCallback);
    ros::Subscriber image_sub = n.subscribe(IMAGE_TOPIC,10,imageCallback);
    ros::Subscriber box_sub   = n.subscribe(BBOX_TOPIC,10,boundingboxCallback);
 
    pub_person  = n.advertise<sensor_msgs::PointCloud2> (OUTPUT_CLOUD,10);
    pub_people = n.advertise<amsl_recog_msgs::ObjectInfoArray> (OUTPUT_BBOX,10);
    pub_diagnostic = n.advertise<diagnostic_msgs::DiagnosticArray> (DIAGNOSTIC, 10);

    ros::Rate rate(20);

    diagnostic_msgs::DiagnosticArray diagnostic;
    diagnostic_msgs::DiagnosticStatus status;
    status.level = 3;
    status.name = "Human Detection Using LiDAR Clustering and Camera Recognition";
    status.message = "Subscribe Data";
    diagnostic.status.push_back(status);

    while(ros::ok())
    {
        if(pc_flag && camera_flag && image_flag && boxes_flag){
            diagnostic.status[0].level=0;
            // human_detection(pc2_trans, camera_, image_, boxes_);
        }
        else{
             diagnostic.status[0].level=1;
        }
        
        pub_diagnostic.publish(diagnostic);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
