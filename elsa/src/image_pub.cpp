/*
 * OpenCV Example using ROS and CPP
 */

// Include the ROS library
#include <ros/ros.h>

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV Window Name
static const std::string OPENCV_WINDOW = "Image window";

// Topics
static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
static const std::string PUBLISH_TOPIC = "/image_converter/output_video";

// Publisher
ros::Publisher pub;

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Draw an example crosshair
    cv::drawMarker(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    pub.publish(cv_ptr->toImageMsg());
}

