#include "catch.hpp"
#include "calibration.hpp"
#include "kittireader.hpp"
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