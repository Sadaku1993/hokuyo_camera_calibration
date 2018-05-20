#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Core>
#include <boost/thread.hpp>
using namespace std;
using namespace Eigen;

bool flag = false;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

void getClusterInfo(CloudA pt, PointA &cluster)
{
    Vector3f centroid;
    centroid[0] = pt.points[0].x;
    centroid[1] = pt.points[0].y;
    centroid[2] = pt.points[0].z;

    Vector3f min_p;
    min_p[0] = pt.points[0].x;
    min_p[1] = pt.points[0].y;
    min_p[2] = pt.points[0].z;

    Vector3f max_p;
    max_p[0] = pt.points[0].x;
    max_p[1] = pt.points[0].y;
    max_p[2] = pt.points[0].z;

    for(size_t i=1;i<pt.points.size();i++){
        centroid[0] += pt.points[i].x;
        centroid[1] += pt.points[i].y;
        centroid[2] += pt.points[i].z;
        if (pt.points[i].x<min_p[0]) min_p[0] = pt.points[i].x;
        if (pt.points[i].y<min_p[1]) min_p[1] = pt.points[i].y;
        if (pt.points[i].x>max_p[0]) max_p[0] = pt.points[i].x;
        if (pt.points[i].y>max_p[1]) max_p[1] = pt.points[i].y;
        if (pt.points[i].z>max_p[2]) max_p[2] = pt.points[i].z;
    }

    cluster.x = centroid[0]/(float)pt.points.size();
    cluster.y = centroid[1]/(float)pt.points.size();
    cluster.z = centroid[2]/(float)pt.points.size();
}

void cpu_clustering(CloudAPtr cloud, CloudA& cluster_centroid, CloudA& cluster_points)
{
	//Clustering//
    printf("start clustering\n");
	pcl::search::KdTree<PointA>::Ptr tree (new pcl::search::KdTree<PointA>);
	tree->setInputCloud (cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
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

        getClusterInfo(*cloud_cluster, cluster_centroid[iii]);
        num = cluster_points.points.size();
    }
}

CloudAPtr cloud_ (new CloudA);
void Callback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    pcl::fromROSMsg(*msg, *cloud_);
    flag = true;
}

void pubPC2Msg(ros::Publisher& pub, CloudA& p_in, std::string& frame_id, ros::Time& time)
{
    sensor_msgs::PointCloud2 p_out;
    toROSMsg(p_in, p_out);
    p_out.header.frame_id = frame_id;
    p_out.header.stamp = time;
    pub.publish(p_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hokuyo_clustering");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/eth_221/cloud", 1, Callback);
    ros::Publisher pub_centroid = n.advertise<sensor_msgs::PointCloud2>("/cluster/centroid", 1);
    ros::Publisher pub_points = n.advertise<sensor_msgs::PointCloud2>("/cluster/points", 1);

    ros::Rate rate(40);

    while(ros::ok()){
        if(flag){
            CloudA cluster_centroid, cluster_points;
            cpu_clustering(cloud_, cluster_centroid, cluster_points);
            std::string frame_id = "laser";
            ros::Time time = ros::Time::now();
            pubPC2Msg(pub_centroid, cluster_centroid, frame_id, time);
            pubPC2Msg(pub_points, cluster_points, frame_id, time);
            flag = false;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return (0);
}
