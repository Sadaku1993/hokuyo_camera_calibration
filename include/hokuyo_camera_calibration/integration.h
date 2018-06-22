#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>

using namespace std;
using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace amsl_recog_msgs;

ros::Publisher pub;
image_transport::Publisher image_pub;

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

void roi_for_cluster(const ObjectInfoArrayConstPtr cluster_msg,
                     const image_geometry::PinholeCameraModel cam_model_,
                     ObjectInfoArray& cluster_roi,
                     cv::Mat &image)
{
    int cluster_size = int(cluster_msg->object_array.size());
    for(int i=0;i<cluster_size; i++)
    {
        ObjectInfoWithROI cluster = cluster_msg->object_array[i];
        BBox bbox;
        get_bbox_data(cluster, bbox);
        
        if(cluster.pose.position.x<0) continue;

        // BBox (PointCloud)
        cv::Point3d pt_cv1(-bbox.p1[1], -bbox.p1[2], bbox.p1[0]);
        cv::Point3d pt_cv4(-bbox.p4[1], -bbox.p4[2], bbox.p4[0]);
        // BBox (Image)
        cv::Point2d uv1 = cam_model_.project3dToPixel(pt_cv1);
        cv::Point2d uv4 = cam_model_.project3dToPixel(pt_cv4);

        ObjectInfoWithROI roi;
        roi.roi.x_offset = uv1.x;
        roi.roi.y_offset = uv1.y;
        roi.roi.width    = uv4.x-uv1.x;
        roi.roi.height   = uv4.y-uv1.y;
        roi.width     = cluster.width;
        roi.height    = cluster.height;
        roi.depth     = cluster.depth;
        roi.pose      = cluster.pose;
        roi.points    = cluster.points;

        cluster_roi.object_array.push_back(roi);

        cv::line(image, cv::Point(uv1.x, uv1.y), cv::Point(uv4.x, uv4.y), cv::Scalar(0,0,200), 3, 4);
    }
}

void roi_for_bbox(const ObjectInfoArrayConstPtr bbox_msg,
                  ObjectInfoArray& bbox_roi,
                  cv::Mat& image)
{
    int bbox_size = int(bbox_msg->object_array.size());
    for(int i=0;i<bbox_size;i++){
        ObjectInfoWithROI bbox = bbox_msg->object_array[i];

        if(bbox.Class!="person") continue;
        
        ObjectInfoWithROI roi;
        roi.Class        = bbox.Class;
        roi.probability  = bbox.probability;
        roi.roi.x_offset = bbox.xmin;
        roi.roi.y_offset = bbox.ymin;
        roi.roi.width    = bbox.xmax - bbox.xmin;
        roi.roi.height   = bbox.ymax - bbox.ymin;
        bbox_roi.object_array.push_back(roi);
        cv::rectangle(image,cv::Point(bbox.xmin, bbox.ymin),cv::Point(bbox.xmax, bbox.ymax),cv::Scalar(0,200,0), 3, 4);
    }
}

void roi_for_fusion(const ObjectInfoArray bbox_roi,
                    const ObjectInfoArray cluster_roi,
                    ObjectInfoArray& detect_roi,
                    cv::Mat& image)
{
    int bbox_size    = int(bbox_roi.object_array.size());
    int cluster_size = int(cluster_roi.object_array.size());

    cout<<"SSD"<<endl;
    for(int i=0;i<bbox_size;i++){
        ObjectInfoWithROI bbox = bbox_roi.object_array[i];
        printf("offset_x:%5d offset_y:%5d width:%5d height:%5d\n",
                bbox.roi.x_offset, bbox.roi.y_offset, bbox.roi.width, bbox.roi.height);
    }
    cout<<"Cluster"<<endl;
    for(int i=0;i<cluster_size;i++){
        ObjectInfoWithROI cluster = cluster_roi.object_array[i];
        printf("offset_x:%5d offset_y:%5d width:%5d height:%5d\n",
                cluster.roi.x_offset, cluster.roi.y_offset, cluster.roi.width, cluster.roi.height);
    }

    if(0<bbox_size && 0<cluster_size)
    {
        Data data[bbox_size][cluster_size];

        for(int i=0;i<bbox_size;i++){
            for(int j=0;j<cluster_size;j++){
                ObjectInfoWithROI bbox    = bbox_roi.object_array[i];
                ObjectInfoWithROI cluster = cluster_roi.object_array[j];
                cv::Rect bbox_rect(bbox.roi.x_offset, bbox.roi.y_offset, bbox.roi.width, bbox.roi.height);
                cv::Rect cluster_rect(cluster.roi.x_offset, cluster.roi.y_offset, cluster.roi.width, 1);
                cv::Rect overlap_rect = cluster_rect & bbox_rect;

                int bbox_area    = bbox_rect.width*bbox_rect.height;
                int cluster_area = cluster_rect.width*cluster_rect.height;
                int overlap_area = overlap_rect.width*overlap_rect.height;

                // hokuyo lidarのheightが1のため、iou計算の際にheightを統一したほうがいいはず
                int bbox_area_dumy = bbox_rect.width*1;

                int union_area = 0;
                float iou = 0.0;
                if(0.0<overlap_area){
                    union_area = cluster_area+bbox_area_dumy-overlap_area;
                    iou = float(overlap_area) / float(union_area);
                }

                printf("i:%d j:%d bbox:%4d cluster:%4d overlap:%4d union:%4d iou:%.3f\n", 
                        i, j, bbox_area, cluster_area, overlap_area, union_area, iou);
                
                data[i][j].bbox    = bbox_rect;
                data[i][j].cluster = cluster_rect;
                data[i][j].overlap = overlap_rect;
                data[i][j].iou = iou;
            }
        }
 
        int id_data[bbox_size] = {};

        for(int i=0;i<bbox_size;i++){
            int max_id=0;
            float max_iou=data[i][0].iou;
            for(int j=1;j<cluster_size;j++){
                if(max_iou<data[i][j].iou){
                    max_id = j;
                    max_iou = data[i][j].iou;
                    id_data[i] = max_id;
                }
            }
        }
    }
}
