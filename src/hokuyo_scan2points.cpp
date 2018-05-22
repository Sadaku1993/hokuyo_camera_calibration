/*
convert from sensor_msgs::Scan to sensor_msgs::PointCloud2

Subscribe : scan
Publish : pointcloud

author : Yudai Sadakuni
*/


#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<laser_geometry/laser_geometry.h>

class My_Filter{
    public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

My_Filter::My_Filter(){
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/eth_221/scan", 100, &My_Filter::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>("/eth_221/cloud", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("laser", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hokuyo_scan2points");

    My_Filter filter;

    ros::spin();

    return 0;
}
