//
// Created by srujan on 6/24/20.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include <sstream>
#include <rosbag/bag.h>



int main(int argc, char **argv){

    ros::init(argc, argv, "imu_talker");

    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<sensor_msgs::Imu>("imu/raw_data", 4000);

    rosbag::Bag Imu_bag("Img_bag.bag", rosbag::bagmode::Write);

    ros::Rate loop_rate(31.5);

    std::string port = "/dev/ttyUSB0";
    uint32_t baudrate = 9600;

    serial::Serial mySerial;
    mySerial.setPort(port);
    mySerial.setBaudrate(baudrate);
    mySerial.open();

    int count = 0;
    int num = 0;


    while(ros::ok()){

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "odom";

        std::string line;
        line = mySerial.readline(1024,"\n");
//        line = mySerial.readline();

        mySerial.flush();

        if(num >10) {

            ROS_INFO("%s", line.c_str());

            char *pch;
            pch = std::strtok(&line[0], " ");
            std::string number;
            double data[6] = {0};
            int count = 0;
            while (pch != NULL) {
                number = pch;
                data[count] = atof(number.c_str());
                pch = std::strtok(NULL, " ");
                count++;
            }

            float angvel_conversion = .005 * 3.1415 / 180.0;
            float linacc_conversion = .00025 * 9.8;

            imu_msg.angular_velocity.x = data[0] * angvel_conversion;
            imu_msg.angular_velocity.y = data[1] * angvel_conversion;
            imu_msg.angular_velocity.z = data[2] * angvel_conversion;
            imu_msg.linear_acceleration.x = data[3] * linacc_conversion;
            imu_msg.linear_acceleration.y = data[4] * linacc_conversion;
            imu_msg.linear_acceleration.z = data[5] * linacc_conversion;

//            ROS_INFO_STREAM();
            chatter_pub.publish(imu_msg);
            Imu_bag.write("Imu", ros::Time::now(), imu_msg);
        }

        num++;

        ros::spinOnce();

        loop_rate.sleep();
    }

    Imu_bag.close();
    return 0;
}