#ifndef IMAGEPUBLISHER_HPP
#define IMAGEPUBLISHER_HPP

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>


class ImagePublisher
{
    private:
        ros::NodeHandle _nh;
        image_transport::ImageTransport _xport;
        image_transport::Publisher _publisher;

    public:
        ImagePublisher();
        void publishImage(cv::String fpath);
};

#endif