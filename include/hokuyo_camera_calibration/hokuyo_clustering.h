#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>

#include <Eigen/Core>
#include <boost/thread.hpp>

using namespace std;
using namespace Eigen;

bool flag = false;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

struct Cluster{
    float x;
    float y;
    float z;

    float width;
    float height;
    float depth;


    Vector3f min_p;
    Vector3f max_p;
};

void getClusterInfo(CloudA pt, Cluster& cluster, PointA& point)
{
    Vector3f centroid;
	centroid[0]=pt.points[0].x;
	centroid[1]=pt.points[0].y;
	centroid[2]=pt.points[0].z;
    
	Vector3f min_p;
	min_p[0]=pt.points[0].x;
	min_p[1]=pt.points[0].y;
    min_p[2]=pt.points[0].z;
    
	Vector3f max_p;
	max_p[0]=pt.points[0].x;
	max_p[1]=pt.points[0].y;
	max_p[2]=pt.points[0].z;

	
	for(size_t i=1;i<pt.points.size();i++){
        centroid[0]+=pt.points[i].x;
        centroid[1]+=pt.points[i].y;
        centroid[2]+=pt.points[i].z;
		if (pt.points[i].x<min_p[0]) min_p[0]=pt.points[i].x;
		if (pt.points[i].y<min_p[1]) min_p[1]=pt.points[i].y;
        if (pt.points[i].z<min_p[2]) min_p[2]=pt.points[i].z;
		
		if (pt.points[i].x>max_p[0]) max_p[0]=pt.points[i].x;
		if (pt.points[i].y>max_p[1]) max_p[1]=pt.points[i].y;
		if (pt.points[i].z>max_p[2]) max_p[2]=pt.points[i].z;

    }
    
	cluster.x=centroid[0]/(float)pt.points.size();
    cluster.y=centroid[1]/(float)pt.points.size();
    cluster.z=centroid[2]/(float)pt.points.size();
    cluster.depth  = max_p[0]-min_p[0];
    cluster.width  = max_p[1]-min_p[1];
    cluster.height = max_p[2]-min_p[2]; 
    cluster.min_p = min_p;
    cluster.max_p = max_p;

	point.x=centroid[0]/(float)pt.points.size();
    point.y=centroid[1]/(float)pt.points.size();
    point.z=centroid[2]/(float)pt.points.size();
}

void clustering(CloudAPtr cloud,
                CloudA& cluster_centroid, 
                CloudA& cluster_points, 
                amsl_recog_msgs::ObjectInfoArray& cluster_array,
                std::string target_frame)
{
	//Clustering//
    printf("start clustering\n");
	pcl::search::KdTree<PointA>::Ptr tree (new pcl::search::KdTree<PointA>);
	tree->setInputCloud (cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointA> ec;
	ec.setClusterTolerance (0.05); // 10cm
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (1081);
	ec.setSearchMethod (tree);
	ec.setInputCloud(cloud);
	ec.extract (cluster_indices);

    printf("cluster num:%d\n", int(cluster_indices.size()));

    // get cluster infomation
    size_t num = 0;
    cluster_centroid.resize(cluster_indices.size());
    for(size_t iii=0;iii<cluster_indices.size();iii++){
        //cluster centroid
        CloudAPtr cloud_cluster (new CloudA);
        cloud_cluster->points.resize(cluster_indices[iii].indices.size());
        //cluster points
        cluster_points.points.resize(cluster_indices[iii].indices.size()+num);

        for(size_t jjj=0;jjj<cluster_indices[iii].indices.size();jjj++){
            int p_num = cluster_indices[iii].indices[jjj];
            cloud_cluster->points[jjj] = cloud->points[p_num];
            cluster_points.points[num+jjj] = cloud->points[p_num];
        }
        
        Cluster cluster;
        getClusterInfo(*cloud_cluster, cluster, cluster_centroid[iii]);
        
        // for amsl_recog_msgs
        amsl_recog_msgs::ObjectInfoWithROI data;

        data.header.frame_id = "camera_color_optical_frame";
        data.header.stamp = ros::Time::now();

        data.pose.position.x = cluster.x;
        data.pose.position.y = cluster.y;
        data.pose.position.z = cluster.z;
        data.pose.orientation.x = 0;
        data.pose.orientation.y = 0;
        data.pose.orientation.z = 0;
        data.pose.orientation.w = 1;
        data.width = cluster.width;
        data.height = cluster.height;
        data.depth = cluster.depth;

        sensor_msgs::PointCloud2 pc2_cloud;
        toROSMsg(*cloud_cluster, pc2_cloud);
        pc2_cloud.header.frame_id = target_frame;
        pc2_cloud.header.stamp = ros::Time::now();

        data.points = pc2_cloud;

        cluster_array.object_array.push_back(data);
    }
}

