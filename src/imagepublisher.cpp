#include "imagepublisher.hpp"
#include "kittireader.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>

ImagePublisher::ImagePublisher() :
    _xport(_nh)
{
    _publisher = _xport.advertise("/camera/image", 1);
}

void ImagePublisher::publishImage(cv::String fpath)
{
    cv_bridge::CvImage cameraImage;
    cameraImage.encoding = sensor_msgs::image_encodings::BGR8;
    cameraImage.image = cv::imread(fpath, CV_LOAD_IMAGE_COLOR);
    _publisher.publish(cameraImage.toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitticamera");
    ImagePublisher node = ImagePublisher();

    std::string globPattern = std::string(argv[1]) + "image_02/data/*.png";
    std::vector<cv::String> fpaths = KittiReader::globFilesHelper(globPattern);
    
    int count = 0;
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ROS_INFO("Publishing image.");
        node.publishImage(fpaths[count]);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        if (count == fpaths.size())
        {
            ROS_INFO("Looping from beginning.");
            count = 0;
        }
    }
    return 0;
}
