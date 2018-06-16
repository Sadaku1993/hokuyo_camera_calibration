/*

Calcrate Human Position Using LiDAR Clustering and Camera Recognition

Publish
    OUTPUT_CLOUD : visualize
    OUTPUT_BBOX  : amsl_recog_msgs::BoundingBoxArray
    DIAGNOSTIC   : Check Error
Subscribe
    INPUT_CLOUD : lidar points
    CAMERA_INFO : camera info
    INPUT_IMAGE : camera image 
    INPUT_BBOX  : SSD  

author : Yudai Sadakuni

*/

#include <hokuyo_camera_calibration/ssd_human.h>


bool info_flag    = false;
bool image_flag   = false;
bool cluster_flag = false;
bool box_flag     = false;

// camera image callback
sensor_msgs::ImageConstPtr image_;
void image_Callback(const sensor_msgs::ImageConstPtr msg){
    image_ = msg;
    image_flag = true;
}

// camera info callback
sensor_msgs::CameraInfoConstPtr info_;
void info_Callback(const sensor_msgs::CameraInfoConstPtr msg){
    info_ = msg;
    info_flag = true;
}

// pointcloud cluster callback
amsl_recog_msgs::ObjectInfoArrayConstPtr clusters_;
void cluster_Callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr msg){
    clusters_ = msg;
    cluster_flag = true;
}

// ssd boundingbox callback
amsl_recog_msgs::ObjectInfoArrayConstPtr boxes_;
void boundingbox_Callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr msg){
    boxes_ = msg;
    box_flag = true;
}

// cluster and boundingbox fusion
void boundingbox_maker(const sensor_msgs::ImageConstPtr& image_msg,
                       const sensor_msgs::CameraInfoConstPtr& cinfo_msg,
                       const amsl_recog_msgs::ObjectInfoArrayConstPtr& cluster_msg,
                       const amsl_recog_msgs::ObjectInfoArrayConstPtr& bbox_msg){
    // convert from ros_image to opencv_image 
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
      cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg);
    
    // DEBUG用のImgeを作成
    // cv::Mat image = cv::Mat::zeros(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;

    amsl_recog_msgs::ObjectInfoArray cluster_roi;
    amsl_recog_msgs::ObjectInfoArray bbox_roi;
    amsl_recog_msgs::ObjectInfoArray detect_roi;


    // ROI(pointcloud clustering)
    roi_for_cluster(cluster_msg, cam_model_, cluster_roi, image);
    int cluster_pick_up = int(cluster_roi.object_array.size());
    
    // ROI(object detector)
    roi_for_bbox(bbox_msg, bbox_roi, image);
    int bbox_pick_up    = int(bbox_roi.object_array.size()); 
    
    // ROI(sensor fusion)
    roi_for_fusion(bbox_roi, cluster_roi, detect_roi, image);
    int detect_pick_up = int(detect_roi.object_array.size());

    // Publish Data Assosiation Result
    pub_people.publish(detect_roi);

    
    // Visualize Rectangle on Image 
    if (DEBUG){
        char value_cluster[256];
        sprintf(value_cluster, "Cluster:%d", cluster_pick_up);
        cv::putText(image, value_cluster, cv::Point(20,100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2, CV_AA); 

        char value_bbox[256];
        sprintf(value_bbox, "BBox:%d", bbox_pick_up);
        cv::putText(image, value_bbox, cv::Point(20,150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2, CV_AA); 

        char value_detect[256];
        sprintf(value_detect, "Detect:%d", detect_pick_up);
        cv::putText(image, value_detect, cv::Point(20,200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,0), 2, CV_AA); 

        cv::imshow("DEBUG", image);
        cv::waitKey(1);
    }
}



int main(int argc, char**argv)
{
    ros::init(argc, argv, "ssd_human_for_hokuyo");
    ros::NodeHandle n;

    n.getParam("/ssd_human/target_frame",   TARGET_FRAME );
    n.getParam("/ssd_human/source_frame",   SOURCE_FRAME );
    n.getParam("/ssd_human/image_topic",    IMAGE_TOPIC );
    n.getParam("/ssd_human/camera_info",    CAMERA_INFO );
    n.getParam("/ssd_human/laser_cluster",  LASER_CLUSTER );
    n.getParam("/ssd_human/camera_cluster", CAMERA_CLUSTER );
    n.getParam("/ssd_human/output_cloud",   OUTPUT_CLOUD );
    n.getParam("/ssd_human/output_bbox",    OUTPUT_BBOX );
    n.getParam("/ssd_human/diagnostic",     DIAGNOSTIC);

    ros::Subscriber image_sub   = n.subscribe(IMAGE_TOPIC,10,image_Callback);
    ros::Subscriber cinfo_sub   = n.subscribe(CAMERA_INFO,10,info_Callback);
    ros::Subscriber cluster_sub = n.subscribe(LASER_CLUSTER, 10, cluster_Callback); 
    ros::Subscriber box_sub     = n.subscribe(CAMERA_CLUSTER,10, boundingbox_Callback);
 
    pub_person  = n.advertise<sensor_msgs::PointCloud2> (OUTPUT_CLOUD,10);
    pub_people = n.advertise<amsl_recog_msgs::ObjectInfoArray> (OUTPUT_BBOX,10);
    pub_diagnostic = n.advertise<diagnostic_msgs::DiagnosticArray> (DIAGNOSTIC, 10);

    ros::Rate rate(10);

    diagnostic_msgs::DiagnosticArray diagnostic;
    diagnostic_msgs::DiagnosticStatus status;
    status.level = 3;
    status.name = "Human Detection Using LiDAR Clustering and Camera Recognition";
    status.message = "Subscribe Data";
    diagnostic.status.push_back(status);

    while(ros::ok())
    {
        if(image_flag && info_flag && cluster_flag && box_flag){
            diagnostic.status[0].level=0;
            boundingbox_maker(image_, info_, clusters_, boxes_);
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
