#include "catch.hpp"
#include "bevprojector.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

bool areEqual(cv::Mat mat1, cv::Mat mat2)
{
    if (mat1.size != mat2.size)
    {
        return false;
    }
    cv::Mat diff;
    cv::compare(mat1, mat2, diff, cv::CMP_NE);
    return (cv::countNonZero(diff) == 0);
}


TEST_CASE( "veloToImage", "" )
{
    cv::Mat PRT(0, 4, CV_32FC1);
    PRT.push_back(cv::Vec4f(609.69543, -721.42163, -1.2512569, -123.04182));
    PRT.push_back(cv::Vec4f(180.38422, 7.6447983, -719.65155, -101.0167));
    PRT.push_back(cv::Vec4f(0.99994534, 0.00012436556, 0.010451303, -0.26938692));
    PRT.push_back(cv::Vec4f(0, 0, 0, 1));
    PRT = PRT.reshape(1, 4);

    cv::Mat lidar(0, 3, CV_32F);
    lidar.push_back(cv::Vec3f(49.592, -14.242,  1.383));
    lidar.push_back(cv::Vec3f(21.725, -14.246,  0.001));
    lidar.push_back(cv::Vec3f(28.998,  16.504, -0.864));
    lidar.push_back(cv::Vec3f(23.721,  12.264, -1.137));
    lidar = lidar.reshape(1, 4).t();

    cv::Mat uv(0, 2, CV_32F);
    uv.push_back(cv::Vec2f(818.64200935,  156.90321062));
    uv.push_back(cv::Vec2f(1090.77177013, 172.85510121));
    uv.push_back(cv::Vec2f(196.78392563,  204.65537563));
    uv.push_back(cv::Vec2f(234.36266804,  217.14527217));
    uv = uv.reshape(1, 4).t();

    cv::Mat out = BevProjector::lidarToImage(lidar, PRT);
    REQUIRE(areEqual(uv, out));
}

TEST_CASE( "fillBevImage", "" )
{
    cv::Mat fpvImage(0, 4, CV_8UC3);
    fpvImage.push_back(cv::Vec4i(1, 4, 4, 5));
    fpvImage.push_back(cv::Vec4i(6, 2, 5, 9));
    fpvImage.push_back(cv::Vec4i(9, 0, 3, 5));
    fpvImage.push_back(cv::Vec4i(0, 5, 1, 5));
    fpvImage = fpvImage.reshape(1, 16);
    fpvImage = cv::repeat(fpvImage, 1, 3);
    fpvImage = fpvImage.reshape(3, 4);

    cv::Mat bevImage(2, 2, CV_32FC3);
    bevImage = bevImage.reshape(3, 4);

    cv::Mat indicesImage(0, 3, CV_32SC2);
    indicesImage.push_back(cv::Vec2i(0, 2));
    indicesImage.push_back(cv::Vec2i(1, 2));
    indicesImage.push_back(cv::Vec2i(0, 1));

    indicesImage.push_back(cv::Vec2i(0, 2));
    indicesImage.push_back(cv::Vec2i(1, 3));
    indicesImage.push_back(cv::Vec2i(1, 1));

    indicesImage.push_back(cv::Vec2i(3, 1));
    indicesImage.push_back(cv::Vec2i(3, 2));
    indicesImage.push_back(cv::Vec2i(3, 1));

    indicesImage.push_back(cv::Vec2i(0, 1));
    indicesImage.push_back(cv::Vec2i(0, 2));
    indicesImage.push_back(cv::Vec2i(3, 0));

    indicesImage = indicesImage.reshape(2, 4);

    cv::Mat dists(0, 3, CV_32FC1);
    dists.push_back(cv::Point3f(2, 10, 1));
    dists.push_back(cv::Point3f(3, 3, 100));
    dists.push_back(cv::Point3f(2, 2, 2));
    dists.push_back(cv::Point3f(20, 20, 1));
    dists = dists.reshape(1, 4);

    BevProjector::fillBevImage(bevImage, fpvImage, indicesImage, dists);
    bevImage = bevImage.reshape(3, 2);

    float r0Expected = ((2. * 4) + (10 * 5) + (1 * 4)) / (2 + 10 + 1.);
    float r0Actual = bevImage.at<cv::Vec3f>(0, 0)[0];
    REQUIRE(r0Actual == r0Expected);
}