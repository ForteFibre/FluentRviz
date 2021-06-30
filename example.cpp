#include "FluentRviz.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fluent_rviz_example");
    flrv::Rviz rviz;
    rviz << flrv::CubeMarker(0)
        .position(0, 0, 0)
        .orientation(0, 0, 0, 0)
        .color(0, 0, 0);

    flrv::Vector3 v = { 1, 2, 3 };
    flrv::Quaternion q = { 1, 0, 0, 0 };

    return 0;
}
