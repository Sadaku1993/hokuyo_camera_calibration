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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <laser_geometry/laser_geometry.h>
#include <image_geometry/pinhole_camera_model.h>

#include <map>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

using namespace std;

ros::Publisher pub_pc;
ros::Publisher pub_pc2;
ros::Time t;

// string target_frame = "/zed_left_camera";
string target_frame = "/camera_link";
string source_frame = "/laser";

bool pc_flag = false;
bool camera_flag = false;
bool image_flag = false;

sensor_msgs::LaserScan scan_;
sensor_msgs::PointCloud pc_;
sensor_msgs::PointCloud2 pc2_;
laser_geometry::LaserProjection projector_;
void scan_Callback(const sensor_msgs::LaserScanConstPtr msg){
    scan_ = *msg;
    projector_.projectLaser(*msg, pc_);
    pc_flag = true;
    sensor_msgs::convertPointCloudToPointCloud2(pc_, pc2_);
    pub_pc2.publish(pc2_);
}

sensor_msgs::ImageConstPtr image_;
void image_Callback(const sensor_msgs::ImageConstPtr msg){
    image_ = msg;
    printf("height:%d width:%d\n", image_->height, image_->width);
    image_flag = true;
}

sensor_msgs::CameraInfoConstPtr info_;
void info_Callback(const sensor_msgs::CameraInfoConstPtr msg){
    info_ = msg;
    camera_flag = true;
}


void colouring(sensor_msgs::PointCloud2 pcl_msg, const sensor_msgs::CameraInfoConstPtr& cinfo_msg, const sensor_msgs::ImageConstPtr& image_msg)
{
    // convert from sensor_msgs::Image to opencv::Mat
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
      cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    image = cv_bridge::toCvShare(image_msg)->image;

    // pointcloud to image 
    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudXYZRGB::Ptr coloured = PointCloudXYZRGB::Ptr(new PointCloudXYZRGB);

    fromROSMsg(pcl_msg, *trans_cloud);

    trans_cloud->header.frame_id = target_frame;

    pcl::copyPointCloud(*trans_cloud, *coloured);
    
    cout<<"size:"<<coloured->points.size()<<endl;
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = coloured->points.begin(); pt < coloured->points.end(); ++pt)
    {
        printf("x:%.2f y:%.2f z:%.2f\n", (*pt).x, (*pt).y, (*pt).z);
        if ((*pt).x<0) continue;
        // cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);

        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);
        printf("x:%.2f y:%.2f\n", uv.x, uv.y);

        if(uv.x>0 && uv.x<image.cols && uv.y > 0 && uv.y < image.rows){
            if (-0.1<(*pt).y && (*pt).y<0.1){
                cv::circle(image, cv::Point(uv.x, uv.y), 1, cv::Scalar(255, 0, 0), -1, 4);
            }
            else{
                cv::circle(image, cv::Point(uv.x, uv.y), 1, cv::Scalar(0, 255, 0), -1, 4);
            }
        }
    }

    cv::imshow("projection", image);
    cv::waitKey(1);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_data");
    ros::NodeHandle n;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber sub_scan  = n.subscribe("/eth_221/scan", 10, scan_Callback);
    // ros::Subscriber sub_image = n.subscribe("usb_cam/image_raw", 10 ,image_Callback);
    // ros::Subscriber sub_info  = n.subscribe("usb_cam/camera_info", 10, info_Callback);
    // ros::Subscriber sub_image = n.subscribe("zed/left/image_rect_color", 10 ,image_Callback);
    // ros::Subscriber sub_info  = n.subscribe("zed/left/camera_info", 10, info_Callback);
    ros::Subscriber sub_image = n.subscribe("camera/color/image_raw", 10 ,image_Callback);
    ros::Subscriber sub_info  = n.subscribe("camera/color/camera_info", 10, info_Callback);

    pub_pc2 = n.advertise<sensor_msgs::PointCloud2>("/eth_221/points", 1);
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

        if(pc_flag && camera_flag && image_flag){
            colouring(pc2_trans, info_, image_);
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
