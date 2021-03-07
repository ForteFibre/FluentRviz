#pragma once

#include <string>
#include <vector>
#include <type_traits>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace flrviz {

namespace option {
    enum class Arrow {
        POSE, VECTOR,
    };

    enum class Color {
        SEPARATE
    };
} // namespace option

namespace param {
    class Vector3 {
        geometry_msgs::Vector3 vector3;

    public:
        Vector3(const geometry_msgs::Vector3 arg): vector3(arg)
        { }

        Vector3(const double x, const double y, const double z) noexcept
        {
            vector3.x = x;
            vector3.y = y;
            vector3.z = z;
        }

        static const Vector3 UNIT_X;
        static const Vector3 UNIT_Y;
        static const Vector3 UNIT_Z;

        operator const geometry_msgs::Vector3 &() const noexcept
        {
            return vector3;
        }
    };

    const Vector3 Vector3::UNIT_X = { 1, 0, 0 };
    const Vector3 Vector3::UNIT_Y = { 0, 1, 0 };
    const Vector3 Vector3::UNIT_Z = { 0, 0, 1 };

    class Scale {
        geometry_msgs::Vector3 vector3;

    public:
        Scale(const double x, const double y, const double z) noexcept
        {
            vector3.x = x;
            vector3.y = y;
            vector3.z = z;
        }

        operator const geometry_msgs::Vector3 &() const noexcept
        {
            return vector3;
        }
    };

    class PoseArrowScale : public Scale {
    public:
        PoseArrowScale(const double length, const double width, const double height) noexcept : Scale(length, width, height) { }
    };

    class VectorArrowScale : public Scale {
    public:
        VectorArrowScale(const double shaft_diameter, const double head_diameter, const double head_length) noexcept : Scale(shaft_diameter, head_diameter, head_length) { }
    };

    class PointScale : public Scale {
    public:
        PointScale(const double width, const double height) noexcept : Scale(width, height, 0) { }
    };

    class LineScale : public Scale {
    public:
        LineScale(const double width) noexcept : Scale(width, 0, 0) { }
    };

    class TextScale : public Scale {
    public:
        TextScale(const double height) noexcept : Scale(0, 0, height) { }
    };

    class Quaternion {
        geometry_msgs::Quaternion quaternion;

    public:
        Quaternion(const geometry_msgs::Quaternion arg) noexcept : quaternion(arg)
        { }

        Quaternion(const double x, const double y, const double z, const double w) noexcept
        {
            quaternion.x = x;
            quaternion.y = y;
            quaternion.z = z;
            quaternion.w = w;
        }

        static Quaternion from_angle_axis(const double theta, const Vector3 axis = Vector3::UNIT_Z) noexcept
        {
            geometry_msgs::Vector3 vector3 = axis;
            return {
                std::cos(theta / 2),
                vector3.x * std::sin(theta / 2),
                vector3.y * std::sin(theta / 2),
                vector3.z * std::sin(theta / 2)
            };
        }

        operator const geometry_msgs::Quaternion &() const noexcept
        {
            return quaternion;
        }
    };

    class Point {
        geometry_msgs::Point point;

    public:
        Point(const geometry_msgs::Point arg) noexcept : point(arg)
        { }

        Point(const double x, const double y, const double z = 0.0) noexcept
        {
            point.x = x;
            point.y = y;
            point.z = z;
        }

        operator const geometry_msgs::Point &() const noexcept
        {
            return point;
        }
    };

    class Color {
        std_msgs::ColorRGBA color;

    public:
        Color(const std_msgs::ColorRGBA arg): color(arg)
        { }

        Color(const float r, const float g, const float b, const float a = 1.0) noexcept
        {
            color.r = r;
            color.g = g;
            color.b = b;
            color.a = a;
        }

        static Color from_hex(const int32_t hex, const float a = 1.0) noexcept
        {
            return {
                ((hex >> 16) & 0xff) / 255.0f,
                ((hex >> 8) & 0xff) / 255.0f,
                (hex & 0xff) / 255.0f,
                a
            };
        }

        operator const std_msgs::ColorRGBA &() const noexcept
        {
            return color;
        }
    };

    class PointVector {
        std::vector<geometry_msgs::Point> points;

    public:
        PointVector &add_point(const Point point) noexcept
        {
            points.push_back(point);
            return *this;
        }

        operator const std::vector<geometry_msgs::Point> &() const noexcept
        {
            return points;
        }
    };

    class LineVector {
        std::vector<geometry_msgs::Point> points;

    public:
        LineVector &add_line(Point start, Point end) noexcept
        {
            points.push_back(start);
            points.push_back(end);
            return *this;
        }

        operator const std::vector<geometry_msgs::Point> &() const noexcept
        {
            return points;
        }
    };

    class TriangleVector {
        std::vector<geometry_msgs::Point> points;

    public:
        TriangleVector &add_triangle(Point a, Point b, Point c) noexcept
        {
            points.push_back(a);
            points.push_back(b);
            points.push_back(c);
            return *this;
        }

        operator const std::vector<geometry_msgs::Point> &() const noexcept
        {
            return points;
        }
    };

    class ColorVector {
        std::vector<std_msgs::ColorRGBA> colors;

    public:
        ColorVector &add_color(Color color) noexcept
        {
            colors.push_back(color);
            return *this;
        }

        operator const std::vector<std_msgs::ColorRGBA> &() const noexcept
        {
            return colors;
        }
    };
} // namespace param

namespace internal {
    template<typename T>
    struct position_helper {
        [[nodiscard]] T &&position(const param::Point position) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.pose.position = position;
            return std::move(self);
        }
    };

    template<typename T>
    struct orientation_helper {
        [[nodiscard]] T &&orientation(const param::Quaternion orientation) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.pose.orientation = orientation;
            return std::move(self);
        }
    };

    template<typename T, typename ScaleParamType = param::Scale>
    struct scale_helper {
        [[nodiscard]] T &&scale(const ScaleParamType scale) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.scale = scale;
            return std::move(self);
        }
    };

    template<typename T>
    using scale_pose_arrow_helper = scale_helper<T, param::PoseArrowScale>;
    template<typename T>
    using scale_vector_arrow_helper = scale_helper<T, param::VectorArrowScale>;
    template<typename T>
    using scale_point_helper = scale_helper<T, param::PointScale>;
    template<typename T>
    using scale_line_helper = scale_helper<T, param::LineScale>;
    template<typename T>
    using scale_text_helper = scale_helper<T, param::TextScale>;

    template<typename T>
    struct color_helper {
        [[nodiscard]] T &&color(const param::Color color) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.color = color;
            return std::move(self);
        }
    };

    template<typename T>
    struct lifetime_helper {
        [[nodiscard]] T &&lifetime(const double lifetime) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &Marker = self.msg();
            Marker.lifetime = ros::Duration(lifetime);
            return std::move(self);
        }
    };

    template<typename T>
    struct frame_locked_helper {
        [[nodiscard]] T &&frame_locked(const bool frame_locked) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.frame_locked = frame_locked;
            return std::move(self);
        }
    };

    template<typename T>
    struct points_helper {
        [[nodiscard]] T &&points(std::vector<geometry_msgs::Point> points) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.points = std::move(points);
            return std::move(self);
        }
    };

    template<typename T>
    struct arrow_point_helper {
        [[nodiscard]] T &&start(const param::Point point) && noexcept
        {
            T &self = static_cast<T &>(*this);
            return std::move(self).set_point(0, point);
        }

        [[nodiscard]] T &&end(const param::Point point) && noexcept
        {
            T &self = static_cast<T &>(*this);
            return std::move(self).set_point(1, point);
        }

    private:
        [[nodiscard]] T &&set_point(const size_t idx, const param::Point &point) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.points[idx] = point;
            return std::move(self);
        }
    };

    template<typename T>
    struct colors_helper {
        [[nodiscard]] T &&colors(std::vector<std_msgs::ColorRGBA> colors) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.colors = std::move(colors);
            return std::move(self);
        }
    };

    template<typename T>
    struct text_helper {
        [[nodiscard]] T &&text(std::string text) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.text = std::move(text);
            return std::move(self);
        }
    };

    template<typename T>
    struct mesh_resource_helper {
        [[nodiscard]] T &&mesh_resource(std::string mesh_resource) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.mesh_resource = std::move(mesh_resource);
            return std::move(self);
        }

        [[nodiscard]] T &&mesh_use_embedded_materials(const bool mesh_use_embedded_materials) && noexcept
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.mesh_use_embedded_materials = mesh_use_embedded_materials;
            return std::move(self);
        }
    };

    template<bool condition, typename Type>
    struct conditional_extend { };

    template<typename Type>
    struct conditional_extend<true, Type> : Type { };

    template<auto Val1, auto Val2>
    inline constexpr bool equals_v = false;

    template<auto Val>
    inline constexpr bool equals_v<Val, Val> = true;

    template<auto Val, auto... Vals>
    inline constexpr bool is_contained_v = (equals_v<Val, Vals> || ...);

    template<int32_t MarkerType>
    inline constexpr bool is_arrow_marker_v = MarkerType == visualization_msgs::Marker::ARROW;

    template<int32_t MarkerType>
    inline constexpr bool is_cube_marker_v = MarkerType == visualization_msgs::Marker::CUBE;

    template<int32_t MarkerType>
    inline constexpr bool is_sphere_marker_v = MarkerType == visualization_msgs::Marker::SPHERE;

    template<int32_t MarkerType>
    inline constexpr bool is_cylinder_marker_v = MarkerType == visualization_msgs::Marker::CYLINDER;

    template<int32_t MarkerType>
    inline constexpr bool is_line_strip_marker_v = MarkerType == visualization_msgs::Marker::LINE_STRIP;

    template<int32_t MarkerType>
    inline constexpr bool is_line_list_marker_v = MarkerType == visualization_msgs::Marker::LINE_LIST;

    template<int32_t MarkerType>
    inline constexpr bool is_cube_list_marker_v = MarkerType == visualization_msgs::Marker::CUBE_LIST;

    template<int32_t MarkerType>
    inline constexpr bool is_sphere_list_marker_v = MarkerType == visualization_msgs::Marker::SPHERE_LIST;

    template<int32_t MarkerType>
    inline constexpr bool is_points_marker_v = MarkerType == visualization_msgs::Marker::POINTS;

    template<int32_t MarkerType>
    inline constexpr bool is_text_view_facing_marker_v = MarkerType == visualization_msgs::Marker::TEXT_VIEW_FACING;

    template<int32_t MarkerType>
    inline constexpr bool is_mesh_resource_marker_v = MarkerType == visualization_msgs::Marker::MESH_RESOURCE;

    template<int32_t MarkerType>
    inline constexpr bool is_triangle_list_marker_v = MarkerType == visualization_msgs::Marker::TRIANGLE_LIST;

    template<int32_t MarkerType>
    inline constexpr bool is_scale_available_v =
        is_cube_marker_v<MarkerType>
        || is_sphere_marker_v<MarkerType>
        || is_cylinder_marker_v<MarkerType>
        || is_cube_list_marker_v<MarkerType>
        || is_sphere_list_marker_v<MarkerType>
        || is_mesh_resource_marker_v<MarkerType>
        || is_triangle_list_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_line_v =
        is_line_strip_marker_v<MarkerType>
        || is_line_list_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_points_available_by_point_v =
        is_line_strip_marker_v<MarkerType>
        || is_cube_list_marker_v<MarkerType>
        || is_sphere_list_marker_v<MarkerType>
        || is_points_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_points_available_by_line_v =
        is_line_list_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_points_available_by_triangle_v =
        is_triangle_list_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_points_available_v =
        is_line_strip_marker_v<MarkerType>
        || is_line_list_marker_v<MarkerType>
        || is_cube_list_marker_v<MarkerType>
        || is_sphere_list_marker_v<MarkerType>
        || is_points_marker_v<MarkerType>
        || is_triangle_list_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_colors_available_v =
        is_line_strip_marker_v<MarkerType>
        || is_line_list_marker_v<MarkerType>
        || is_cube_list_marker_v<MarkerType>
        || is_sphere_list_marker_v<MarkerType>
        || is_points_marker_v<MarkerType>
        || is_triangle_list_marker_v<MarkerType>;

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_position
        : position_helper<Marker<MarkerType, Options...>> { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_orientation
        : orientation_helper<Marker<MarkerType, Options...>> { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_color
        : conditional_extend<
            !is_colors_available_v<MarkerType> || !is_contained_v<option::Color::SEPARATE, Options...>,
            color_helper<Marker<MarkerType, Options...>>> { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_scale
        : conditional_extend<
            is_arrow_marker_v<MarkerType> && is_contained_v<option::Arrow::POSE, Options...>,
            scale_pose_arrow_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_arrow_marker_v<MarkerType> && is_contained_v<option::Arrow::VECTOR, Options...>,
            scale_vector_arrow_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_scale_available_v<MarkerType>,
            scale_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_points_marker_v<MarkerType>,
            scale_point_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_line_v<MarkerType>,
            scale_line_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_text_view_facing_marker_v<MarkerType>,
            scale_text_helper<Marker<MarkerType, Options...>>> { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_points
        : conditional_extend<
            is_arrow_marker_v<MarkerType> && is_contained_v<option::Arrow::VECTOR, Options...>,
            arrow_point_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_points_available_v<MarkerType>,
            points_helper<Marker<MarkerType, Options...>>> { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_colors
        : conditional_extend<
            is_colors_available_v<MarkerType> && is_contained_v<option::Color::SEPARATE, Options...>,
            colors_helper<Marker<MarkerType, Options...>>> { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_text
        : conditional_extend<
            is_text_view_facing_marker_v<MarkerType>,
            text_helper<Marker<MarkerType, Options...>>> { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_mesh_resource
        : conditional_extend<
            is_mesh_resource_marker_v<MarkerType>,
            mesh_resource_helper<Marker<MarkerType, Options...>>> { };

} // namespace internal

namespace color {
    inline constexpr static int32_t RED = 0xF44336;
    inline constexpr static int32_t PINK = 0xFF4081;
    inline constexpr static int32_t PURPLE = 0x9C27B0;
    inline constexpr static int32_t DEEP_PURPLE = 0x7B1FA2;
    inline constexpr static int32_t INDIGO = 0x3F51B5;
    inline constexpr static int32_t BRUE = 0x448aff;
    inline constexpr static int32_t LIGHT_BRUE = 0x03A9F4;
    inline constexpr static int32_t CYAN = 0x00BCD4;
    inline constexpr static int32_t TEAL = 0x009688;
    inline constexpr static int32_t GREEN = 0x4CAF50;
    inline constexpr static int32_t LIGHT_GREEN = 0x8BC34A;
    inline constexpr static int32_t LIME = 0xCDDC39;
    inline constexpr static int32_t YELLOW = 0xFFEB3B;
    inline constexpr static int32_t AMBER = 0xFFC107;
    inline constexpr static int32_t ORANGE = 0xFF9800;
    inline constexpr static int32_t DEEP_ORANGE = 0xFF5722;
    inline constexpr static int32_t BROWN = 0x795548;
    inline constexpr static int32_t GREY = 0x9E9E9E;
    inline constexpr static int32_t BLUE_GREY = 0x607D8B;
    inline constexpr static int32_t WHITE = 0xffffff;
    inline constexpr static int32_t BLACK = 0x000000;
} // namespace color

template<int32_t MarkerType, auto... Options>
class Marker
    : public internal::conditional_position<Marker, MarkerType, Options...>
    , public internal::conditional_orientation<Marker, MarkerType, Options...>
    , public internal::conditional_scale<Marker, MarkerType, Options...>
    , public internal::conditional_color<Marker, MarkerType, Options...>
    , public internal::lifetime_helper<Marker<MarkerType, Options...>>
    , public internal::frame_locked_helper<Marker<MarkerType, Options...>>
    , public internal::conditional_points<Marker, MarkerType, Options...>
    , public internal::conditional_colors<Marker, MarkerType, Options...>
    , public internal::conditional_text<Marker, MarkerType, Options...>
    , public internal::conditional_mesh_resource<Marker, MarkerType, Options...> {

    visualization_msgs::Marker marker;

public:
    Marker(int32_t id, std::string ns = "") noexcept
    {
        marker.id = id;
        marker.ns = std::move(ns);
        marker.type = MarkerType;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        if constexpr (internal::is_scale_available_v<MarkerType>) {
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
        }

        if constexpr (internal::is_points_marker_v<MarkerType>) {
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
        }

        if constexpr (internal::is_line_v<MarkerType>) {
            marker.scale.x = 0.05;
        }

        if constexpr (internal::is_arrow_marker_v<MarkerType>) {
            if constexpr (internal::is_contained_v<option::Arrow::POSE, Options...>) {
                marker.scale.x = 1;
                marker.scale.y = 1;
                marker.scale.z = 1;
            }
            if constexpr (internal::is_contained_v<option::Arrow::VECTOR, Options...>) {
                marker.scale.x = .2;
                marker.scale.y = .4;
                marker.scale.z = .4;
            }
        }

        if constexpr (
            internal::is_arrow_marker_v<MarkerType>
            && internal::is_contained_v<option::Arrow::VECTOR, Options...>) {
            marker.points.resize(2);
        }

        if constexpr (internal::is_text_view_facing_marker_v<MarkerType>) {
            marker.scale.z = 1;
            marker.text = "visualization_msgs::Marker::TEXT_VIEW_FACING";
        }
    }

    Marker(const Marker<MarkerType, Options...> &) = delete;
    Marker(Marker<MarkerType, Options...> &&) = default;

    visualization_msgs::Marker &msg() noexcept { return marker; }
};

namespace marker {
    using PoseArrow = Marker<visualization_msgs::Marker::ARROW, option::Arrow::POSE>;
    using VectorArrow = Marker<visualization_msgs::Marker::ARROW, option::Arrow::VECTOR>;
    using Arrow = VectorArrow;
    using Cube = Marker<visualization_msgs::Marker::CUBE>;
    using Sphere = Marker<visualization_msgs::Marker::SPHERE>;
    using Cylinder = Marker<visualization_msgs::Marker::CYLINDER>;
    using LineStrip = Marker<visualization_msgs::Marker::LINE_STRIP>;
    using ColorfulLineStrip = Marker<visualization_msgs::Marker::LINE_STRIP, option::Color::SEPARATE>;
    using LineList = Marker<visualization_msgs::Marker::LINE_LIST>;
    using ColorfulLineList = Marker<visualization_msgs::Marker::LINE_LIST, option::Color::SEPARATE>;
    using CubeList = Marker<visualization_msgs::Marker::CUBE_LIST>;
    using ColorfulCubeList = Marker<visualization_msgs::Marker::CUBE_LIST, option::Color::SEPARATE>;
    using SphereList = Marker<visualization_msgs::Marker::SPHERE_LIST>;
    using ColorfulSphereList = Marker<visualization_msgs::Marker::SPHERE_LIST, option::Color::SEPARATE>;
    using Points = Marker<visualization_msgs::Marker::POINTS>;
    using ColorfulPoints = Marker<visualization_msgs::Marker::POINTS, option::Color::SEPARATE>;
    using TextViewFacing = Marker<visualization_msgs::Marker::TEXT_VIEW_FACING>;
    using MeshResource = Marker<visualization_msgs::Marker::MESH_RESOURCE>;
    using TriangleList = Marker<visualization_msgs::Marker::TRIANGLE_LIST>;
    using ColorfulTriangleList = Marker<visualization_msgs::Marker::TRIANGLE_LIST, option::Color::SEPARATE>;
} // namespace marker

class Rviz {
    ros::NodeHandle nh;
    ros::Publisher pub;

    std::string frame_id_;

public:
    Rviz(const std::string frame_id = "map", const std::string topic = "visualization_marker") noexcept
        : pub(nh.advertise<visualization_msgs::Marker>(topic, 1))
        , frame_id_(frame_id)
    { }

    template<int32_t MarkerType, auto... Options>
    void add(Marker<MarkerType, Options...> &m)
    {
        visualization_msgs::Marker &marker = m.msg();
        marker.action = visualization_msgs::Marker::ADD;
        publish(marker);
    }

    template<int32_t MarkerType, auto... Options>
    void add(Marker<MarkerType, Options...> &&m) { add(m); }

    void delete_marker(const int32_t id, std::string ns = "") noexcept
    {
        visualization_msgs::Marker marker;
        marker.id = id;
        marker.ns = std::move(ns);
        marker.action = visualization_msgs::Marker::DELETE;
        publish(marker);
    }

    void delete_all_marker() noexcept
    {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        publish(marker);
    }

    void publish(visualization_msgs::Marker &marker) noexcept
    {
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        pub.publish(marker);
    }
};

} // namespace flrviz
