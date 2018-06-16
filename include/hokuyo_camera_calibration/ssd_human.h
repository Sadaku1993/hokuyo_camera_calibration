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

#define DEBUG 1
#define RESULT 1
#define SHOW 1

using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Publisher pub_person;
ros::Publisher pub_people;
ros::Publisher pub_diagnostic;
ros::Time t;

// Sensor Frame
string TARGET_FRAME;
string SOURCE_FRAME;
// Subscribe Topic
string IMAGE_TOPIC;
string CAMERA_INFO;
string LASER_CLUSTER;
string CAMERA_CLUSTER;
// Publish Topic
string OUTPUT_CLOUD;
string OUTPUT_BBOX;
string DIAGNOSTIC;

struct BBox{
    Vector3f p1;
    Vector3f p2;
    Vector3f p3;
    Vector3f p4;
};

struct Data{
    cv::Rect bbox;
    cv::Rect cluster;
    cv::Rect overlap;
    
    int bbox_size;
    int cluster_size;
    int overlap_size;
    int union_size;
    
    float iou;
};

void get_bbox_data(const amsl_recog_msgs::ObjectInfoWithROI cluster, BBox& bbox)
{
    Vector3f centroid;
    centroid[0] = cluster.pose.position.x; //depth 
    centroid[1] = cluster.pose.position.y; //width
    centroid[2] = cluster.pose.position.z; //height

    float depth  = cluster.depth;
    float width  = cluster.width;
    float height = cluster.height;
    
    // depthはcentroidの値を利用する
    bbox.p1[0] = centroid[0];
    bbox.p1[1] = centroid[1] + width/2;
    bbox.p1[2] = centroid[2] + height/2;

    bbox.p2[0] = centroid[0];
    bbox.p2[1] = centroid[1] - width/2;
    bbox.p2[2] = centroid[2] + height/2;

    bbox.p3[0] = centroid[0];
    bbox.p3[1] = centroid[1] + width/2;
    bbox.p3[2] = centroid[2] - height/2;

    bbox.p4[0] = centroid[0];
    bbox.p4[1] = centroid[1] - width/2;
    bbox.p4[2] = centroid[2] - height/2;
}

void show_data(const amsl_recog_msgs::ObjectInfoWithROI cluster, BBox& bbox){
    Vector3f centroid;
    centroid[0] = cluster.pose.position.x; //depth 
    centroid[1] = cluster.pose.position.y; //width
    centroid[2] = cluster.pose.position.z; //height

    float depth  = cluster.depth;
    float width  = cluster.width;
    float height = cluster.height;

    printf("Ceter x:%.2f y:%.2f z:%.2f\n", centroid[0], centroid[1], centroid[2]);
    printf("Depth:%.2f Width:%.2f Height:%.2f\n", depth, width, height);
}

void roi_for_cluster(const amsl_recog_msgs::ObjectInfoArrayConstPtr& cluster_msg,
                     const image_geometry::PinholeCameraModel cam_model_,
                     amsl_recog_msgs::ObjectInfoArray& cluster_roi,
                     cv::Mat &image){
    int cluster_size = int(cluster_msg->object_array.size());
    for(int i=0;i<cluster_size;i++){
        amsl_recog_msgs::ObjectInfoWithROI cluster = cluster_msg->object_array[i];
        BBox bbox;
        get_bbox_data(cluster, bbox);

        // if(cluster.pose.position.x<0) continue;
        if(DEBUG) show_data(cluster, bbox);
        cv::Point3d pt_cv1(-bbox.p1[1], -bbox.p1[2], bbox.p1[0]);
        // cv::Point3d pt_cv2(-bbox.p2[1], -bbox.p2[2], bbox.p2[0]);
        // cv::Point3d pt_cv3(-bbox.p3[1], -bbox.p3[2], bbox.p3[0]);
        cv::Point3d pt_cv4(-bbox.p4[1], -bbox.p4[2], bbox.p4[0]);
        
        cv::Point2d uv1 = cam_model_.project3dToPixel(pt_cv1);
        // cv::Point2d uv2 = cam_model_.project3dToPixel(pt_cv2);
        // cv::Point2d uv3 = cam_model_.project3dToPixel(pt_cv3);
        cv::Point2d uv4 = cam_model_.project3dToPixel(pt_cv4);

        amsl_recog_msgs::ObjectInfoWithROI roi;
        roi.roi.x_offset = uv1.x;
        roi.roi.y_offset = uv1.y;
        roi.roi.width    = uv4.x-uv1.x;
        roi.roi.height   = uv4.y-uv1.y;
        
        roi.width     = cluster.width;
        roi.height    = cluster.height;
        roi.depth     = cluster.depth;
        // roi.curvature = cluster.curvature;
        
        roi.pose      = cluster.pose;
        roi.points    = cluster.points;

        cluster_roi.object_array.push_back(roi);

        // if (DEBUG) cv::rectangle(image, cv::Point(uv1.x, uv1.y), cv::Point(uv4.x, uv4.y), cv::Scalar(0,0,200), 3, 4);
        if (DEBUG) cv::line(image, cv::Point(uv1.x, uv1.y), cv::Point(uv4.x, uv4.y), cv::Scalar(0,0,200), 3, 4);

    }
}


void roi_for_bbox(const amsl_recog_msgs::ObjectInfoArrayConstPtr& bbox_msg,
                  amsl_recog_msgs::ObjectInfoArray& bbox_roi,
                  cv::Mat &image){
    int bbox_size = int(bbox_msg->object_array.size());
    for(int i=0;i<bbox_size;i++){
        amsl_recog_msgs::ObjectInfoWithROI bbox = bbox_msg->object_array[i];
        if(bbox.Class!="person") continue;

        amsl_recog_msgs::ObjectInfoWithROI roi;
        roi.Class        = bbox.Class;
        roi.probability  = bbox.probability;
        roi.roi.x_offset = bbox.xmin;
        roi.roi.y_offset = bbox.ymin;
        roi.roi.width    = bbox.xmax - bbox.xmin;
        roi.roi.height   = bbox.ymax - bbox.ymin;

        bbox_roi.object_array.push_back(roi);

        if(DEBUG) cv::rectangle(image,cv::Point(bbox.xmin, bbox.ymin),cv::Point(bbox.xmax, bbox.ymax),cv::Scalar(0,200,0), 3, 4);
    }
}


void roi_for_fusion(const amsl_recog_msgs::ObjectInfoArray bbox_roi,
                    const amsl_recog_msgs::ObjectInfoArray cluster_roi,
                    amsl_recog_msgs::ObjectInfoArray& detect_roi,
                    cv::Mat &image){
    int bbox_pick_up = int(bbox_roi.object_array.size());
    int cluster_pick_up = int(cluster_roi.object_array.size());

    if(0<bbox_pick_up && 0<cluster_pick_up){

        Data data[bbox_pick_up][cluster_pick_up];

        for(int i=0;i<bbox_pick_up;i++){
            for(int j=0;j<cluster_pick_up;j++){
                amsl_recog_msgs::ObjectInfoWithROI bbox    = bbox_roi.object_array[i];
                amsl_recog_msgs::ObjectInfoWithROI cluster = cluster_roi.object_array[j];
                cv::Rect bbox_rect(bbox.roi.x_offset, bbox.roi.y_offset, bbox.roi.width, bbox.roi.height);
                cv::Rect cluster_rect(cluster.roi.x_offset, cluster.roi.y_offset, cluster.roi.width, cluster.roi.height);
                cv::Rect overlap_rect = cluster_rect & bbox_rect;

                int bbox_area    = bbox_rect.width*bbox_rect.height;
                int cluster_area = cluster_rect.width*cluster_rect.height;
                int overlap_area = overlap_rect.width*overlap_rect.height;
                int union_area = 0;
                float iou = 0.0;
                if(0.0<overlap_area){
                    union_area = cluster_area+bbox_area-overlap_area;
                    iou = float(overlap_area) / float(union_area);
                }
                data[i][j].bbox    = bbox_rect;
                data[i][j].cluster = cluster_rect;
                data[i][j].overlap = overlap_rect;
                data[i][j].iou = iou;
            }
        }

        int id_data[bbox_pick_up] = {};

        for(int i=0;i<bbox_pick_up;i++){
            int max_id=0;
            float max_iou=data[i][0].iou;
            for(int j=1;j<cluster_pick_up;j++){
                if(max_iou<data[i][j].iou){
                    max_id = j;
                    max_iou = data[i][j].iou;
                    id_data[i] = max_id;
                }
            }
        }

        
        for(int i=0;i<bbox_pick_up;i++){
            int j = id_data[i];
            if(0.3 < data[i][j].iou){
                amsl_recog_msgs::ObjectInfoWithROI roi;

                // object detection data
                roi.Class       = bbox_roi.object_array[i].Class;
                roi.probability = bbox_roi.object_array[i].probability;
                
                // overlap area
                roi.xmin = data[i][j].overlap.x;
                roi.ymin = data[i][j].overlap.y;
                roi.xmax = data[i][j].overlap.x + data[i][j].overlap.width;
                roi.ymax = data[i][j].overlap.y + data[i][j].overlap.height;
                
                // pointcloud cluster data
                roi.roi       = cluster_roi.object_array[j].roi;
                roi.pose      = cluster_roi.object_array[j].pose;
                roi.width     = cluster_roi.object_array[j].width;
                roi.height    = cluster_roi.object_array[j].height;
                roi.depth     = cluster_roi.object_array[j].depth;
                // roi.curvature = cluster_roi.object_array[j].curvature; 
                roi.points    = cluster_roi.object_array[j].points;

                detect_roi.object_array.push_back(roi);

            }
        }

        int detect_pick_up = int(detect_roi.object_array.size());
        
        // Data Association Result
        if(RESULT){
            printf("Cluster:%d BBox:%d Detect:%d\n", cluster_pick_up, bbox_pick_up, detect_pick_up);
            for(int i=0;i<bbox_pick_up;i++){
                for(int j=0;j<cluster_pick_up;j++){
                    printf("%.2f ", data[i][j].iou);
                }
                printf(" --> %d\n", id_data[i]);
            }
        }

        if(DEBUG){
            for(int i=0;i<int(detect_roi.object_array.size());i++){
                amsl_recog_msgs::ObjectInfoWithROI roi;
                roi = detect_roi.object_array[i];
                int xmin = roi.xmin;
                int ymin = roi.ymin;
                int xmax = roi.xmax;
                int ymax = roi.ymax;
                cv::rectangle(image, cv::Point(xmin, ymin), cv::Point(xmax, ymax), cv::Scalar(200,0,0), 3, 4);
            }
        }

    }
}


