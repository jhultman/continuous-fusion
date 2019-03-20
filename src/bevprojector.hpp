#ifndef BEVPROJECTOR_HPP
#define BEVPROJECTOR_HPP

#include <opencv2/opencv.hpp>

class BevProjector
{

    private:
        cv::Mat _xform;
        cv::Mat lidarToImage(cv::Mat lidarPoints, cv::Mat PRT);
        cv::Mat divideRow(cv::Mat mat, cv::Mat row);

    public:
        BevProjector(cv::Mat PRT);
        cv::Mat getBevImage(cv::Mat fpvPixelVals, cv::Mat bevLidarPoints);
        static void fillBevImage(cv::Mat bevImage, cv::Mat fpvImage, cv::Mat indices, cv::Mat dists);
        static cv::Mat row_linspace(int start, int end, size_t n);
        static cv::Mat col_linspace(int start, int end, size_t n);
        static cv::Mat meshgrid(cv::Mat x_lin, cv::Mat y_lin);
        static cv::Mat makeRandBGR8(size_t nrows, size_t ncols);
};

#endif