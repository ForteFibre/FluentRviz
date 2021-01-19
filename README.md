# FluentRviz

このライブラリはRvizを使いやすくするためのライブラリです．
**まだ開発段階です**

Before

```cpp
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("visualization_msgs", 1);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1;
    marker.color.r = 1;

    pub.publish(marker);
}
```

After

```cpp
#include <ros/ros.h>
#include <FluentRviz.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example");
    flrviz::Rviz rviz("map", "visualization_msgs");
    auto marker = flrviz::marker::Cube(0)
        .orientation(1)
        .scale(1, 1, 1)
        .color(1, 0, 0);
    rviz.add(marker);
}
```