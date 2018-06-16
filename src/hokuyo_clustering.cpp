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

#include <hokuyo_camera_calibration/hokuyo_clustering.h>

ros::Publisher pub_bbox;
ros::Publisher pub_cluster;
ros::Publisher pub_centroid;
ros::Publisher pub_points;

void pubPC2Msg(ros::Publisher pub, CloudA p_in, std::string frame_id, ros::Time time)
{
    sensor_msgs::PointCloud2 p_out;
    toROSMsg(p_in, p_out);
    p_out.header.frame_id = frame_id;
    p_out.header.stamp = time;
    pub.publish(p_out);
}

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);
    
    CloudA cluster_centroid;
    CloudA cluster_points;
    std::string target_frame = msg->header.frame_id;
    amsl_recog_msgs::ObjectInfoArray cluster_array;
    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    clustering(cloud, 
               cluster_centroid,
               cluster_points,
               cluster_array,
               bbox_array,
               target_frame);

    cout<<bbox_array<<endl;

    pub_bbox.publish(bbox_array);
    pub_cluster.publish(cluster_array);

    ros::Time time = ros::Time::now();
    pubPC2Msg(pub_centroid, cluster_centroid, target_frame, time);
    pubPC2Msg(pub_points, cluster_points, target_frame, time);
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
