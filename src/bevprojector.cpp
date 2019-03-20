#include "bevprojector.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>

BevProjector::BevProjector(cv::Mat PRT)
{
    _xform = PRT;
}

cv::Mat BevProjector::row_linspace(int start, int end, size_t n)
{
    float step = (end - start + 1) / n;
    cv::Mat m(1, n, CV_32F);
    for (int j = 0; j < n; ++j)
    {
        m.at<float>(0, j) = j * step + 0.5;
    }
    return m;
}

cv::Mat BevProjector::col_linspace(int start, int end, size_t n)
{
    float step = (end - start + 1) / n;
    cv::Mat m(n, 1, CV_32F);
    for (int i = 0; i < n; ++i)
    {
        m.at<float>(i, 0) = i * step + 0.5;
    }
    return m;
}

cv::Mat BevProjector::meshgrid(cv::Mat x_lin, cv::Mat y_lin)
{
    int numel = x_lin.cols * y_lin.rows;
    cv::Mat x = cv::repeat(x_lin, y_lin.rows, 1).reshape(1, numel);
    cv::Mat y = cv::repeat(y_lin, 1, x_lin.cols).reshape(1, numel);
    cv::Mat xy;
    cv::hconcat(x, y, xy);
    return xy;
}

cv::Mat BevProjector::makeRandBGR8(size_t nrows, size_t ncols)
{
    cv::Mat mat;
    cv::Mat mat_float(nrows, ncols, CV_32FC3);
    cv::randu(mat_float, 0, 255);
    mat_float.convertTo(mat, CV_8UC3);
    return mat;
}

cv::Mat BevProjector::divideRow(cv::Mat mat, cv::Mat row)
{
    assert(mat.cols == row.cols);
    assert(mat.channels() == row.channels());
    return mat / cv::repeat(row, mat.cols, 1);
}

cv::Mat BevProjector::lidarToImage(cv::Mat lidarPoints, cv::Mat PRT)
{
    cv::Mat homogeneousLidar;
    lidarPoints.copyTo(homogeneousLidar);
    cv::Mat ones = cv::Mat::ones(1, lidarPoints.cols, CV_32F);
    ones.copyTo(homogeneousLidar.rowRange(3, 4));
    cv::Mat lidarImage = PRT * homogeneousLidar;
    lidarImage = divideRow(lidarImage, lidarImage.row(2));
    return lidarImage.rowRange(0, 2);
}

void BevProjector::fillBevImage(cv::Mat bevImage, cv::Mat fpvImage, cv::Mat indices, cv::Mat dists)
{
    float weight;
    float sumWeight;
    cv::Vec3f pixelVal;
    cv::Vec3f meanPixelVal;
    int indexFrom;
    for (int m = 0; m < indices.rows; ++m)
    {
        meanPixelVal = 0;
        sumWeight = 0;

        // Gather mean pixel val from knn
        for (int n = 0; n < indices.cols; ++n)
        {
            indexFrom = indices.at<int>(m, n);
            pixelVal = fpvImage.at<cv::Vec3b>(indexFrom);
            weight = dists.at<float>(m, n);
            meanPixelVal += weight * pixelVal;
            sumWeight += weight;
        }

        meanPixelVal /= sumWeight;
        bevImage.at<cv::Vec3f>(m) = meanPixelVal;
    }
}

cv::Mat BevProjector::getBevImage(cv::Mat fpvPixelVals, cv::Mat bevLidarPoints)
{
    size_t bevNumRows = 20;
    size_t bevNumCols = 40;

    cv::Mat x_lin = BevProjector::row_linspace(0, 30, bevNumRows);
    cv::Mat y_lin = BevProjector::col_linspace(0, 40, bevNumCols);
    cv::Mat bevPixelLocs = BevProjector::meshgrid(x_lin, y_lin); 
    cv::Mat bevPixelVals = cv::Mat(bevNumRows * bevNumCols, 1, CV_32FC3);

    unsigned int knn = 3;
    cv::Mat indices, dists;

    // TODO: Switch to cvflann::KDTreeSingleIndexParalidarms for performance
    cv::flann::Index flann_index(bevLidarPoints, cv::flann::KDTreeIndexParams(1));
    flann_index.knnSearch(bevPixelLocs, indices, dists, knn, cv::flann::SearchParams(32));
    BevProjector::fillBevImage(bevPixelVals, fpvPixelVals, indices, dists);
    return bevPixelVals;
}

int main()
{
    cv::Mat PRT = cv::Mat::eye(4, 4, CV_32F);
    BevProjector proj = BevProjector(PRT);

    // Can choose independently: num lidar points, FPV image size, BEV image size
    size_t fpvNumRows = 25;
    size_t fpvNumCols = 15;
    cv::Mat fpvPixelVals = BevProjector::makeRandBGR8(fpvNumRows * fpvNumCols, 1);

    size_t bevNumLidar = 1100;
    cv::Mat bevLidarPoints(bevNumLidar, 2, CV_32FC1);
    cv::randu(bevLidarPoints, 0, 20);
    cv::Mat bevImage = proj.getBevImage(fpvPixelVals, bevLidarPoints);
}