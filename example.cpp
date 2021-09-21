#include "FluentRviz.hpp"

int main(int argc, char **argv)
{
    using namespace flrv;
    using namespace flrv::param;

    ros::init(argc, argv, "fluent_rviz_example");

    Rviz rviz;
    rviz << marker::Cube(0)
        .position(0, 0, 0)
        .orientation(Quaternion::AngleAxis(M_PI / 2))
        .color(Color::Aqua());

    rviz << marker::LineList(2)
        .color(0, 0, 1)
        .scale(0.05)
        .points({ { 1, 0, 0 }, { 0, 1, 0 } });

    rviz << marker::Points(3)
        .data(util::Index(10), [](double i) {
            return marker::Points()
                .color(1, 0, 0)
                .points({ { i, i, i }, { -i, -i, -i } });
        });

    Vector3 i = { 1, 0, 0 };
    Vector3 j = { 0, 1, 0 };
    Vector3 k = { 0, 0, 1 };

    Point a = 3 * i + 4 * j + 5 * k;
    Quaternion q = Quaternion::AngleAxis(M_PI / 2);
    Point rot = q * a;
    std::cout << rot << std::endl;

    geometry_msgs::Vector3 v, w;
    std::cout << v + w << std::endl;

    return 0;
}
