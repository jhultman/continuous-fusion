#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>

cv::Mat row_linspace(int start, int end, int n)
{
    float step = (end - start + 1) / n;
    cv::Mat m(1, n, CV_32F);
    for (int j = 0; j < n; ++j)
    {
        m.at<float>(0, j) = j * step + 0.5;
    }
    return m;
}

cv::Mat col_linspace(int start, int end, int n)
{
    float step = (end - start + 1) / n;
    cv::Mat m(n, 1, CV_32F);
    for (int i = 0; i < n; ++i)
    {
        m.at<float>(i, 0) = i * step + 0.5;
    }
    return m;
}

cv::Mat meshgrid(cv::Mat x_lin, cv::Mat y_lin)
{
    int numel = x_lin.cols * y_lin.rows;
    cv::Mat x = cv::repeat(x_lin, y_lin.rows, 1).reshape(1, numel);
    cv::Mat y = cv::repeat(y_lin, 1, x_lin.cols).reshape(1, numel);
    cv::Mat xy;
    cv::hconcat(x, y, xy);
    return xy;
}

cv::Mat makeRandBGR8(size_t nrows, size_t ncols)
{
    cv::Mat mat;
    cv::Mat mat_float(nrows, ncols, CV_32FC3);
    cv::randu(mat_float, 0, 255);
    mat_float.convertTo(mat, CV_8UC3);
    return mat;
}

int main()
{
    // Can choose independently: num lidar points, FPV image size, BEV image size
    size_t bev_nrows = 20;
    size_t bev_ncols = 40;

    cv::Mat x_lin = row_linspace(0, 30, bev_nrows);
    cv::Mat y_lin = col_linspace(0, 40, bev_ncols);
    cv::Mat bev_pixel_locs = meshgrid(x_lin, y_lin); 
    cv::Mat bev_pixel_vals = cv::Mat(bev_nrows * bev_ncols, 1, CV_32FC3);

    size_t fpv_nrows = 25;
    size_t fpv_ncols = 15;
    cv::Mat fpv_pixel_vals = makeRandBGR8(fpv_nrows * fpv_ncols, 1);

    size_t bev_npoints_lidar = 1100;
    cv::Mat bev_lidar_points(bev_npoints_lidar, 2, CV_32FC1);
    cv::randu(bev_lidar_points, 0, 20);

    // TODO: Switch to cvflann::KDTreeSingleIndexParalidarms for performance
    cv::flann::Index flann_index(bev_lidar_points, cv::flann::KDTreeIndexParams(1));

    unsigned int knn = 3;
    cv::Mat indices, dists;
    flann_index.knnSearch(bev_pixel_locs, indices, dists, knn, cv::flann::SearchParams(32));

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
        for (int n = 0; n < knn; ++n)
        {
            indexFrom = indices.at<int>(m, n);
            pixelVal = fpv_pixel_vals.at<cv::Vec3b>(indexFrom);
            weight = dists.at<float>(m, n);
            meanPixelVal += weight * pixelVal;
            sumWeight += weight;
        }

        meanPixelVal /= sumWeight;
        bev_pixel_vals.at<cv::Vec3f>(m) = meanPixelVal;
    }
}