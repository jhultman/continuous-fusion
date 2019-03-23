#ifndef BEVPROJECTOR_HPP
#define BEVPROJECTOR_HPP

#include <opencv2/opencv.hpp>

class BevProjector
{

    private:
        static cv::Mat initBevPixelLocs(size_t bevNumRows, size_t bevNumCols);
        static cv::Mat row_linspace(int start, int end, size_t n);
        static cv::Mat col_linspace(int start, int end, size_t n);
        static cv::Mat meshgrid(cv::Mat x_lin, cv::Mat y_lin);
        static cv::Mat makeRandBGR8(size_t nrows, size_t ncols);
        static cv::Point3f bilinearInterp(cv::Mat img, cv::Point2f pt);

    public:
        BevProjector();
        static cv::Mat divideRow(cv::Mat mat, cv::Mat row);
        static cv::Mat lidarToImage(cv::Mat lidarPoints, cv::Mat PRT);
        static cv::Mat getBevImage(cv::Mat fpvPixelVals, cv::Mat lidarPoints, cv::Mat PRT);
        static cv::Mat knnIndicesLidarToIndicesImage(cv::Mat indices, cv::Mat lidarPoints, cv::Mat PRT);
        static void fillBevImage(cv::Mat bevImage, cv::Mat fpvImage, cv::Mat indices, cv::Mat dists);
};

#endif