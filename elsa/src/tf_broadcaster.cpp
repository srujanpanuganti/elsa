//
// Created by srujan on 6/20/20.
//

#include "../include/tf_broadcaster.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;

    while(n.ok()){
        broadcaster.sendTransform(
                tf::StampedTransform(
//                        tf::Transform(tf::Quaternion(0, 0, 1, 0), tf::Vector3(-0.0145, -0.0765, -0.06)),
                        tf::Transform(tf::Quaternion(0, 0, 1, 0), tf::Vector3(-0.021, 0.0215, -0.06)),

                        ros::Time::now(),"odom", "base_link"));
//                        ros::Time::now(),"base_link", "odom"));
        r.sleep();
    }
}
