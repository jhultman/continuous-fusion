#include "bevprojector.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>
#include <math.h>

BevProjector::BevProjector(cv::Mat PRT)
{
    _xform = PRT;
}

cv::Mat BevProjector::row_linspace(int start, int end, size_t n)
{
    cv::Mat m(1, n, CV_32F);
    float step = (end - start) / static_cast<float>(n - 1);
    for (int j = 0; j < n; ++j)
    {
        m.at<float>(0, j) = start + j * step;
    }
    return m;
}

cv::Mat BevProjector::col_linspace(int start, int end, size_t n)
{
    cv::Mat m(n, 1, CV_32F);
    float step = (end - start) / static_cast<float>(n - 1);
    for (int i = 0; i < n; ++i)
    {
        m.at<float>(i, 0) = start + i * step;
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
    return mat / cv::repeat(row, mat.rows, 1);
}

cv::Mat BevProjector::lidarToImage(cv::Mat lidarPoints, cv::Mat PRT)
{
    cv::Mat homogeneousLidar(0, lidarPoints.cols, CV_32FC1);
    cv::Mat ones = cv::Mat::ones(1, lidarPoints.cols, CV_32FC1);
    homogeneousLidar.push_back(lidarPoints.rowRange(0, 3));
    homogeneousLidar.push_back(ones);
    homogeneousLidar = homogeneousLidar.reshape(1, 4);
    cv::Mat lidarImage = PRT * homogeneousLidar;
    cv::Mat row = lidarImage.row(2).reshape(1, 1);
    cv::Mat divided = divideRow(lidarImage, lidarImage.row(2));
    cv::Mat uvCoords;
    uvCoords.push_back(divided.rowRange(0, 2));
    uvCoords = uvCoords.reshape(1, 2);
    return uvCoords;
}

cv::Point3f BevProjector::bilinearInterp(cv::Mat img, cv::Point2f pt)
{
    float u = pt.y - int(pt.y);
    float d = int(1 + pt.y) - pt.y;
    float l = pt.x - int(pt.x);
    float r = int(1 + pt.x) - pt.x;
    return cv::Point3f(0, 0, 0);
}

cv::Mat BevProjector::knnIndicesLidarToIndicesImage(cv::Mat indicesLidar, cv::Mat lidarPoints, cv::Mat PRT)
{
    int rows = indicesLidar.rows;
    int cols = indicesLidar.cols;

    cv::Mat lidarPointsImage = BevProjector::lidarToImage(lidarPoints, PRT).t();

    cv::Mat indicesImage = cv::Mat(rows, cols, CV_32SC2);
    cv::Vec2i pointImage;
    int index;
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            index = indicesLidar.at<int>(i, j);
            pointImage = lidarPointsImage.row(index);
            indicesImage.at<cv::Vec2i>(i, j)[0] = static_cast<int>(pointImage[0]);
            indicesImage.at<cv::Vec2i>(i, j)[1] = static_cast<int>(pointImage[1]);
        }
    }
    return indicesImage;
}

void BevProjector::fillBevImage(cv::Mat bevImage, cv::Mat fpvImage, cv::Mat indicesImage, cv::Mat dists)
{
    float weight;
    float sumWeight;
    cv::Vec3f pixelVal;
    cv::Vec3f meanPixelVal;
    cv::Vec2i indexFrom;

    cv::Mat fpvImageF;
    fpvImage.convertTo(fpvImageF, CV_32FC3);

    for (int i = 0; i < indicesImage.rows; ++i)
    {
        meanPixelVal = 0;
        sumWeight = 0;

        // Gather mean pixel val from knn
        for (int j = 0; j < indicesImage.cols; ++j)
        {
            indexFrom = indicesImage.at<cv::Vec2i>(i, j);
            int r = static_cast<int>(indexFrom[0]);
            int c = static_cast<int>(indexFrom[1]);

            if ((r >= fpvImage.rows) || (r < 0) ||
                (c >= fpvImage.cols) || (c < 0))
            {
                continue;
            }

            pixelVal = fpvImageF.at<cv::Vec3f>(r, c);
            weight = dists.at<float>(i, j);
            meanPixelVal += weight * pixelVal;
            sumWeight += weight;
        }
        if (sumWeight > 0)
        {
            meanPixelVal /= sumWeight;
        }
        bevImage.at<cv::Vec3f>(i) = meanPixelVal;
    }
}

cv::Mat BevProjector::getBevImage(cv::Mat fpvPixelVals, cv::Mat lidarPoints, cv::Mat PRT)
{
    size_t bevNumRows = 400;
    size_t bevNumCols = 400;

    cv::Mat x_lin = BevProjector::row_linspace(0, 80, bevNumRows);
    cv::Mat y_lin = BevProjector::col_linspace(-40, 40, bevNumCols);
    cv::Mat bevPixelLocs = BevProjector::meshgrid(x_lin, y_lin); 
    cv::Mat bevPixelVals = cv::Mat::zeros(bevNumRows * bevNumCols, 1, CV_32FC3);

    // TODO: Switch to cvflann::KDTreeSingleIndexParalidarms for performance
    unsigned int knn = 5;
    cv::Mat indicesLidar, dists;
    cv::Mat bevLidarPoints = lidarPoints.rowRange(0, 2).t();

    cv::flann::Index flann_index(bevLidarPoints, cv::flann::KDTreeIndexParams(1));
    flann_index.knnSearch(bevPixelLocs, indicesLidar, dists, knn, cv::flann::SearchParams(32));
    cv::Mat indicesImage = knnIndicesLidarToIndicesImage(indicesLidar, lidarPoints, PRT);
    BevProjector::fillBevImage(bevPixelVals, fpvPixelVals, indicesImage, dists);
    bevPixelVals = bevPixelVals.reshape(3, bevNumRows);
    return bevPixelVals;
}