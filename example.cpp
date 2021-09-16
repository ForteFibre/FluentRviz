#include "FluentRviz.hpp"

int main(int argc, char **argv)
{
    using namespace flrv;
    ros::init(argc, argv, "fluent_rviz_example");

    Rviz rviz;
    rviz << marker::Cube(0)
        .position(0, 0, 0)
        .orientation(param::Quaternion::AngleAxis(M_PI / 2))
        .color(param::Color::Aqua());

    rviz << marker::LineList(2)
        .color(0, 0, 1)
        .scale(0.05);

    param::Vector3 i = { 1, 0, 0 };
    param::Vector3 j = { 0, 1, 0 };
    param::Vector3 k = { 0, 0, 1 };

    param::Vector3 a = 3 * i + 4 * j + 5 * k;

    param::Quaternion q = { 0, a };
    param::Quaternion r = q / q.norm();

    return 0;
}
