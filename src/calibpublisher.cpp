#include "calibration.hpp"
#include "kittireader.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "std_msgs/String.h"

void callback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received calib matrix.");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        std::cout << cv_ptr->image << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge failure upon calib receipt: %s", e.what());
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitticalib");
    ros::NodeHandle nh;
    image_transport::ImageTransport xport(nh);

    image_transport::Publisher publisher = xport.advertise("/calibration/velotoimage", 1);
    image_transport::Subscriber subscriber = xport.subscribe("/calibration/velotoimage", 1, callback);

    std::string basedir = argv[1];
    Calibration calib = KittiReader::makeCalib(basedir);
    cv::Mat mat = calib.getVeloToImage();
    cv_bridge::CvImage calibMatrix;

    calibMatrix.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    calibMatrix.image = mat;

    ros::Rate loop_rate(0.2);
    int count = 0;
    while (ros::ok())
    {
        ROS_INFO("Publishing calib.");
        publisher.publish(calibMatrix.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
