#include "makerosbag.hpp"

int main(int argc, const char* argv[])
{
    std::string basedir = argv[1];
    Calibration calib = Calibration(basedir);
    return 0;
}
