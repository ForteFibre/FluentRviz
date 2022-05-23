# FluentRviz

このライブラリはRvizを使いやすくするためのライブラリです．
**まだ開発段階です**

Before

```cpp
class Node : public rclcpp::Node
{
public:
    Node(rclcpp::NodeOptions options) : rclcpp::Node("node",options)
    {
        publisher_ = node.create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = get_clock()->now();
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1;
        marker.color.r = 1;
        
        publisher.publish(marker);
    }
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};
```

After

```cpp
class Node : public rclcpp::Node
{
public:
    Node(rclcpp::NodeOptions options) : rclcpp::Node("node",options)
    {
        rviz_.init(*this, "visualization_marker", "map");
        rviz_ << flrv::marker::Cube(0)
                .color(1, 0, 0);
    }
private:
    flrv::Rviz rviz_;
};
```
