#include "FluentRviz.hpp"

int main(int argc, char **argv)
{
    using namespace flrv;
    ros::init(argc, argv, "fluent_rviz_example");

    Rviz rviz;
    rviz << marker::Cube(0)
        .position(0, 0, 0)
        .orientation(math::Rotation(M_PI / 2))
        .color(color::Color::Aqua());

    rviz << marker::LineList(2)
        .color(0, 0, 1)
        .scale(0.05);

    constexpr auto i = math::Vector3 { 1, 0, 0 };
    constexpr auto j = math::Vector3 { 0, 1, 0 };
    constexpr auto k = math::Vector3 { 0, 0, 1 };

    constexpr auto a = 3 * i + 4 * j + 5 * k;

    auto q = math::Quaternion { 0, a };
    auto r = q / math::norm(q);

    auto vector = util::convert<geometry_msgs::Vector3>(a);
    auto point = util::convert<geometry_msgs::Point>(a);
    auto quaternion = util::convert<geometry_msgs::Quaternion>(q);

    auto c = color::Color { 1, 0, 0 };
    auto d = c | color::HSLA::saturation(20) | color::RGBA::red(100) | color::Color::alpha(0.4);

    return 0;
}
