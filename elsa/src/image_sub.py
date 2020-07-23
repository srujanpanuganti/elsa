#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('hello', cv2_img)
        cv2.waitKey(10)

    except CvBridgeError as e:
        print(e)

    else:
        # Save your OpenCV2 image as a jpeg
        cv2.imwrite('camera_image.jpeg', cv2_img)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/image"

    print('started')
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()




# //#include <ros/ros.h>
# //#include <image_transport/image_transport.h>
# //#include <opencv2/highgui/highgui.hpp>
# //#include <cv_bridge/cv_bridge.h>
# //
# //
# //void imageCallback(const sensor_msgs::ImageConstPtr& msg){
#                                                            //
#                                                            //    try{
# //        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8") -> image);
# //
# //        cv::waitKey(30);
# //    }
# //    catch(cv_bridge::Exception& e){
#                                     //        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
# //    }
# //}
# //
# //
# //
# //int main(int argc, char **argv){
# //
# //    ros::init(argc, argv, "image_sub");
# //    ros::NodeHandle nh;
# //
# //    cv::namedWindow("view");
# //    cv::startWindowThread();
# //
# //    image_transport::ImageTransport it(nh);
# //    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
# //
# //    cv::startWindowThread();
# ////    cv::waitKey(5);
# //
# //    ros::spin();
# //    cv::destroyWindow("view");
# //}