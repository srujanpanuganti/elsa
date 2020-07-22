//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//
//static const std::string OPENCV_WINDOW = "Image window";
//
//class ImageConverter
//{
//    ros::NodeHandle nh_;
//    image_transport::ImageTransport it_;
//    image_transport::Subscriber image_sub_;
//    image_transport::Publisher image_pub_;
//
//public:
//    ImageConverter()
//            : it_(nh_)
//    {
//        // Subscrive to input video feed and publish output video feed
//        image_sub_ = it_.subscribe("/camera/image_raw", 1,
//                                   &ImageConverter::imageCb, this);
//        image_pub_ = it_.advertise("/image_converter/output_video", 1);
//
//        cv::namedWindow(OPENCV_WINDOW);
//    }
//
//    ~ImageConverter()
//    {
//        cv::destroyWindow(OPENCV_WINDOW);
//    }
//
//    void imageCb(const sensor_msgs::ImageConstPtr& msg)
//    {
//        cv_bridge::CvImagePtr cv_ptr;
//        try
//        {
//            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//        }
//        catch (cv_bridge::Exception& e)
//        {
//            ROS_ERROR("cv_bridge exception: %s", e.what());
//            return;
//        }
//
//        // Draw an example circle on the video stream
//        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
//
//        // Update GUI Window
//        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//        cv::waitKey(3);
//
//        // Output modified video stream
//        image_pub_.publish(cv_ptr->toImageMsg());
//    }
//};
//
//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "image_pub");
//    ImageConverter ic;
//    ros::spin();
//    return 0;
//}

//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <sstream> // for converting the command line parameter to integer
//
//int main(int argc, char** argv)
//{
//    // Check if video source has been passed as a parameter
//    if(argv[1] == NULL) return 1;
//
//    ros::init(argc, argv, "image_publisher");
//    ros::NodeHandle nh;
//    image_transport::ImageTransport it(nh);
//    image_transport::Publisher pub = it.advertise("camera/image", 1);
//
//    // Convert the passed as command line parameter index for the video device to an integer
//    std::istringstream video_sourceCmd(argv[1]);
//    int video_source;
//    // Check if it is indeed a number
//    if(!(video_sourceCmd >> video_source)) return 1;
//
//    cv::VideoCapture cap(video_source);
//    // Check if video device can be opened with the given index
//    if(!cap.isOpened()) return 1;
//    cv::Mat frame;
//    sensor_msgs::ImagePtr msg;
//
//    ros::Rate loop_rate(5);
//    while (nh.ok()) {
//        cap >> frame;
//        // Check if grabbed frame is actually full with some content
//        if(!frame.empty()) {
//            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//            pub.publish(msg);
//            cv::waitKey(1);
//        }
//
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
//}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    cv::Mat image = cv::imread('home/pi/elsa_ws/src/elsa/images/tf.png', IMREAD_COLOR);
    cv::waitKey(30);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Rate loop_rate(5);
    while (nh.ok()) {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}