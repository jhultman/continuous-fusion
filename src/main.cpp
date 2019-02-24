#include "kittireader.hpp"

int main(int argc, const char* argv[])
{
    std::string basedir = argv[1];
    KittiReader kitti = KittiReader(basedir);
    return 0;
}
