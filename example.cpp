#include "FluentRviz.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fluent_rviz_example");
    flrv::Rviz rviz;
    rviz << flrv::marker::Cube(0)
        .position(0, 0, 0)
        .orientation(0, 0, 0, 0)
        .color(0, 0, 0);

    rviz << flrv::marker::LineList(2)
        .color(0, 0, 1)
        .scale(0.05)
        .each(flrv::util::Indices(10), [](size_t i, flrv::marker::LineList::Manipulator &manipulator) {
            manipulator.add_position(i, i, 0).add_position(i, -i, 0);
        });

    flrv::param::Vector3 v = { 1, 2, 3 };
    flrv::param::Quaternion q = { 1, 0, 0, 0 };

    geometry_msgs::Vector3 vector = v;
    geometry_msgs::Point point = v;
    geometry_msgs::Quaternion quaternion = q;

    flrv::param::RGBA rgba = { 1, 1, 0 };
    flrv::param::HSLA hsla = { 240, 100, 50 };

    std_msgs::ColorRGBA color_rgba = rgba;
    std_msgs::ColorRGBA color_hsla = hsla;

    flrv::param::RGBA rgba_hsla = hsla;
    flrv::param::HSLA hsla_rgba = rgba;

    return 0;
}
