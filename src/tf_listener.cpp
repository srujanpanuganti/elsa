//
// Created by srujan on 6/20/20.
//

#include <ros/ros.h>
//#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

void transformPose(const tf::TransformListener& listener){
    //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
//    geometry_msgs::PointStamped odometry_read;
    geometry_msgs::PoseStamped odometry_read;
    odometry_read.header.frame_id = "odom";

    //we'll just use the most recent transform available for our simple example
    odometry_read.header.stamp = ros::Time();

    //just an arbitrary point in space
//    odometry_read.point.x = 1.0;
//    odometry_read.point.y = 0.2;
//    odometry_read.point.z = 0.0;

    odometry_read.pose.position.x = 1.0;
    odometry_read.pose.position.y = 2.0;
    odometry_read.pose.position.z = 0.0;

    odometry_read.pose.orientation.x = 0.0;
    odometry_read.pose.orientation.y = 0.0;
    odometry_read.pose.orientation.z = 0.0;
    odometry_read.pose.orientation.w = 1.0;

    try{
        geometry_msgs::PoseStamped base_pose;
        listener.transformPose("base_link", odometry_read, base_pose);

        ROS_INFO("base_odom: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                 odometry_read.pose.position.x, odometry_read.pose.position.y, odometry_read.pose.position.z,
                 base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z, base_pose.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"odom\" to \"base_link\": %s", ex.what());
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_listener");
    ros::NodeHandle n;

    tf::TransformListener listener(ros::Duration(10));

    //we'll transform a point once every second
    ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPose, boost::ref(listener)));

    ros::spin();

}

