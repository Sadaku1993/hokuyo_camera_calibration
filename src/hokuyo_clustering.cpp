/*

Hokuyo_Lidar_clustering using PCL Libraries

Publish
    pub_bbox : cluster boundingbox (for visualize in Rviz)
    pub_cluster : cluster boundingbox
    pub_centroid : cluster cecntroid (pointcloud2)
    pub_points: cluster points (pointcloud2)
Subscribe
    sub : laser data using

Causion:
    using original msg (amsl_recog_msgs, jsk_recognition_msgs)

author:Yudai Sadakuni

*/

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>

#include <Eigen/Core>

#include <hokuyo_camera_calibration/clustering.h>
#include <hokuyo_camera_calibration/pubPC2Msg.h>

using namespace std;
using namespace Eigen;

bool flag = false;

ros::Publisher pub_bbox;
ros::Publisher pub_cluster;
ros::Publisher pub_centroid;
ros::Publisher pub_points;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    //Clustring
    vector<Clusters> cluster_array;
    if(0<cloud->points.size())
        clustering(cloud, cluster_array);

    std::string target_frame = msg->header.frame_id;
    amsl_recog_msgs::ObjectInfoArray object_array;
    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    for(size_t i=0;i<cluster_array.size();i++)
    {
        sensor_msgs::PointCloud2 pc2_cloud;
        toROSMsg(cluster_array[i].points, pc2_cloud);
        pc2_cloud.header.frame_id = target_frame;
        pc2_cloud.header.stamp = ros::Time::now();

        amsl_recog_msgs::ObjectInfoWithROI data;
        data.header.frame_id    = target_frame;
        data.header.stamp = ros::Time::now();
        data.pose.position.x = cluster_array[i].data.x;
        data.pose.position.y = cluster_array[i].data.y;
        data.pose.position.z = cluster_array[i].data.z;
        data.pose.orientation.x = 0;
        data.pose.orientation.y = 0;
        data.pose.orientation.z = 0;
        data.pose.orientation.w = 1;
        data.width = cluster_array[i].data.width;
        data.height = cluster_array[i].data.height;
        data.depth = cluster_array[i].data.depth;
        data.points = pc2_cloud;

        object_array.object_array.push_back(data);

    }
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "hokuyo_clustering");
    ros::NodeHandle n;

    ros::Subscriber sub =  n.subscribe("/cloud", 10, pcCallback);

    pub_bbox = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/cloud/bbox", 10);
    pub_cluster = n.advertise<amsl_recog_msgs::ObjectInfoArray>("/cloud/cluster", 10);
    pub_centroid = n.advertise<sensor_msgs::PointCloud2>("/cloud/centroid", 1);
    pub_points = n.advertise<sensor_msgs::PointCloud2>("/cloud/points", 1);
    ros::spin();

    return 0;
}
