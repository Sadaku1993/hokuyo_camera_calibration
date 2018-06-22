#include <hokuyo_camera_calibration/integration.h>

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

    ObjectInfoArray cluster_roi;
    ObjectInfoArray bbox_roi;
    ObjectInfoArray detect_roi;

    // ROI(pointcloud clustering)
    roi_for_cluster(cluster_msg, cam_model_, cluster_roi, image);
    int cluster_pick_up = int(cluster_roi.object_array.size());
    
    // ROI(object detector)
    roi_for_bbox(ssd_msg, bbox_roi, image);
    int bbox_pick_up    = int(bbox_roi.object_array.size()); 

    // ROI(sensor fusion)
    roi_for_fusion(bbox_roi, cluster_roi, detect_roi, image);
    int detect_pick_up = int(detect_roi.object_array.size());

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	image_pub.publish(msg);

}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "integration");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

	message_filters::Subscriber<Image> image_sub(nh, "/image", 10);
	message_filters::Subscriber<CameraInfo> cinfo_sub(nh, "/camera_info", 10);
    message_filters::Subscriber<ObjectInfoArray> cluster_sub(nh, "/cluster", 10);
    message_filters::Subscriber<ObjectInfoArray> ssd_sub(nh, "/ssd", 10);

	
    typedef sync_policies::ApproximateTime<Image, CameraInfo, ObjectInfoArray, ObjectInfoArray> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), image_sub, cinfo_sub, cluster_sub, ssd_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));


	image_pub = it.advertise("/image_output", 10);

    ros::spin();

    return 0;
}

