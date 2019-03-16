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
    ROS_INFO("Received image.");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        auto image = cv_ptr->image;
        std::cout << "shape: (" << image.rows << ", " << image.cols << ")" << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge failure upon image receipt: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitticamera");
    ros::NodeHandle nh;
    image_transport::ImageTransport xport(nh);

    image_transport::Publisher publisher = xport.advertise("/camera/image", 1);
    image_transport::Subscriber subscriber = xport.subscribe("/camera/image", 1, callback);

    std::string globPattern = std::string(argv[1]) + "image_02/data/*.png";
    std::vector<cv::String> fpaths = KittiReader::globFilesHelper(globPattern);
    std::vector<cv::Mat> images = KittiReader::getImages(fpaths);

    cv_bridge::CvImage cameraImage;
    cameraImage.encoding = sensor_msgs::image_encodings::BGR8;
    cameraImage.image = images[0];

    ros::Rate loop_rate(0.05);
    int count = 0;
    while (ros::ok())
    {
        ROS_INFO("Publishing image.");
        publisher.publish(cameraImage.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
