#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

ros::Publisher pub;

void bbox(){
    ros::Time t = ros::Time::now();
 
    jsk_recognition_msgs::BoundingBoxArray bbox_array;
    bbox_array.header.stamp = t;
    bbox_array.header.frame_id = "/map";
    bbox_array.header.seq = 0;
    
    for(int i=0;i<10;i++)
    {
        jsk_recognition_msgs::BoundingBox bbox;
        bbox.header.frame_id = "/map";
        bbox.header.stamp = t;
        bbox.header.seq = 0;
        bbox.pose.position.x = -5 + i * 1;
        bbox.pose.position.y = -5 + i * 1;
        bbox.pose.position.z = 0;
        bbox.pose.orientation.x = 0;
        bbox.pose.orientation.y = 0;
        bbox.pose.orientation.z = 0;
        bbox.pose.orientation.w = 1;
        bbox.dimensions.x = 0.5;
        bbox.dimensions.y = 0.5;
        bbox.dimensions.z = 0.5;
        bbox_array.boxes.push_back(bbox);
    }

    pub.publish(bbox_array);
}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "jsk_bbox_test");
    ros::NodeHandle n;

    pub = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bbox_array", 10);

    ros::Rate rate(20);
    while(ros::ok())
    {
        bbox();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

