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
    int numel = y_lin.rows * x_lin.cols;
    cv::Mat x = cv::repeat(x_lin, y_lin.rows, 1).reshape(1, numel);
    cv::Mat y = cv::repeat(y_lin, 1, x_lin.cols).reshape(1, numel);
    cv::Mat xy;
    cv::hconcat(x, y, xy);
    return xy;
}

int main()
{
    cv::Mat x_lin = row_linspace(0, 30, 20);
    cv::Mat y_lin = col_linspace(0, 40, 40);
    cv::Mat pixel_locs = meshgrid(x_lin, y_lin);

    std::cout << pixel_locs << std::endl;
    cv::flann::Index flann_index(pixel_locs, cv::flann::KDTreeIndexParams(1));

    unsigned int max_neighbours = 3;
    cv::Mat query = (cv::Mat_<float>(1, 2) << 12.205, 24.07);
    cv::Mat indices, dists;
    double radius= 2.0;

    flann_index.radiusSearch(query, indices, dists, radius, max_neighbours,
            cv::flann::SearchParams(32));

    std::cout << indices << std::endl;
    std::cout << dists << std::endl;

    int n;
    for (int i = 0; i < max_neighbours; ++i)
    {
        n = indices.at<int>(i);
        std::cout << "(" << pixel_locs.at<float>(n, 0) << ", " << pixel_locs.at<float>(n, 1) << ")" << std::endl;
    }
}