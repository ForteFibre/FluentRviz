#include "FluentRviz.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fluent_rviz_example");
    flrv::Rviz rviz;
    rviz << flrv::marker::Cube(0)
        .position(0, 0, 0)
        .orientation(flrv::param::Rotation(M_PI / 2))
        .color(0, 0, 0);

    rviz << flrv::marker::LineList(2)
        .color(0, 0, 1)
        .scale(0.05);

    constexpr auto i = flrv::param::Vector3 { 1, 0, 0 };
    constexpr auto j = flrv::param::Vector3 { 0, 1, 0 };
    constexpr auto k = flrv::param::Vector3 { 0, 0, 1 };

    constexpr auto a = 3 * i + 4 * j + 5 * k;

    auto q = flrv::param::Quaternion { 0, a };
    auto r = q / flrv::param::norm(q);

    auto vector = flrv::util::convert<geometry_msgs::Vector3>(a);
    auto point = flrv::util::convert<geometry_msgs::Point>(a);
    auto quaternion = flrv::util::convert<geometry_msgs::Quaternion>(q);

    auto c = flrv::param::Color { 1, 0, 0 };
    auto d = c | flrv::param::hsla::saturation(20) | flrv::param::rgba::red(100) | flrv::param::color::alpha(0.4);

    return 0;
}
