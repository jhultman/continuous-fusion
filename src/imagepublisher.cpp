#include "kittireader.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitticamera");
    ros::NodeHandle nh;
    image_transport::ImageTransport xport(nh);
    image_transport::Publisher publisher = xport.advertise("/camera/image", 1);

    std::string globPattern = std::string(argv[1]) + "image_02/data/*.png";
    std::vector<cv::String> fpaths = KittiReader::globFilesHelper(globPattern);
//    std::vector<cv::Mat> images = KittiReader::getImages(fpaths);
    cv::Mat imageCv = cv::imread(fpaths[0], CV_LOAD_IMAGE_COLOR);

    cv_bridge::CvImage cameraImage;
    cameraImage.encoding = sensor_msgs::image_encodings::BGR8;
    cameraImage.image = imageCv;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ROS_INFO("Publishing image.");
        publisher.publish(cameraImage.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
