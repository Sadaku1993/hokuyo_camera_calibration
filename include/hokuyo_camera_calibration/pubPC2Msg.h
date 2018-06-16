#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

void pubPC2Msg(ros::Publisher pub, CloudA p_in, std::string frame_id, ros::Time time)
{
    sensor_msgs::PointCloud2 p_out;
    toROSMsg(p_in, p_out);
    p_out.header.frame_id = frame_id;
    p_out.header.stamp = time;
    pub.publish(p_out);
}


