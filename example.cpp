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

    geometry_msgs::Vector3 vector = v;
    geometry_msgs::Point point = v;
    geometry_msgs::Quaternion quaternion = q;

    flrv::RGBA rgba = { 1, 1, 0 };
    flrv::HSLA hsla = { 240, 1, 0.5 };

    std_msgs::ColorRGBA color_rgba = rgba;
    std_msgs::ColorRGBA color_hsla = hsla;

    flrv::RGBA rgba_hsla = hsla;
    flrv::HSLA hsla_rgba = rgba;

    return 0;
}
