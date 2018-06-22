#include <ros/ros.h>

using namespace std;
using namespace sensor_msgs;
using namespace amsl_recog_msgs;

void callback(const ImageConstPtr &image_msg,
              const CameraInfoConstPtr &cinfo_msg,
              const ObjectInfoArrayConstPtr &cluster_msg,
              const ObjectInfoArrayConstPtr &ssd_msg)
{
    // convert from ros_image to opencv_image 
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
        cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // get CamInfo
    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg);

    // Debug用 Image
    cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;

}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "integration");

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    message_fileters::Subscriber<Image> image_sub(n, "image", 1);
    message_fileters::Subscriber<CameraInfo> cinfo_sub(n, "camera_info", 1);
    message_fileters::Subscriber<ObjectInfoArray> cluster_sub(n, "cluster" , 1);
    message_fileters::Subscriber<ObjectInfoArray> ssd_sub(n, "ssd", 1);

    typedef sync_policies::ApproximateTime<Image, CameraInfo, ObjectInfoArray, ObjectInfoArray> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cinfo_sub, cluster_sub, ssd_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    pub = n.advertise<PointCloud2>("/output", 10);

    ros::spin();

    return 0;
}

