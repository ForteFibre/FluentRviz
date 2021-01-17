#pragma once

#include <string>
#include <vector>
#include <type_traits>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace flrviz {

enum class ArrowOption {
    POSE, VECTOR,
};

enum class PointsOption {
    SEPARATELY_UPDATE, BULK_UPDATE,
};

enum class ColorsOption {
    SEPARATELY_UPDATE, BULK_UPDATE,
};

namespace internal {
    template<typename T>
    struct position_helper {
        T &position(const geometry_msgs::Point &position)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.pose.position = position;
            return self;
        }

        T &position(double x, double y, double z)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            return self;
        }
    };

    template<typename T>
    struct orientation_helper {
        T &orientation(const geometry_msgs::Quaternion &orientation)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.pose.orientation = orientation;
            return self;
        }

        T &orientation(double w, double x = 0, double y = 0, double z = 0)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.pose.orientation.w = w;
            marker.pose.orientation.x = x;
            marker.pose.orientation.y = y;
            marker.pose.orientation.z = z;
            return self;
        }
    };

    template<typename T>
    struct scale3_helper {
        T &scale(double x, double y, double z)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.scale.x = x;
            marker.scale.y = y;
            marker.scale.z = z;
            return self;
        }
    };

    template<typename T>
    struct scale2_helper {
        T &size(double x, double y)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.scale.x = x;
            marker.scale.y = y;
            return self;
        }
    };

    template<typename T>
    struct scale_width_helper {
        T &scale(double width)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.scale.x = width;
            return self;
        }
    };

    template<typename T>
    struct scale_height_helper {
        T &scale(double height)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.scale.z = height;
            return self;
        }
    };

    template<typename T>
    struct color_helper {
        T &color(const std_msgs::ColorRGBA &color)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.color = color;
            return self;
        }

        T &color(double r, double g, double b, double a = 1.0)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = a;
            return self;
        }
    };

    template<typename T>
    struct lifetime_helper {
        T &lifetime(const ros::Duration &lifetime)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.lifetime = lifetime;
            return self;
        }

        T &lifetime(double lifetime)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &Marker = self.msg();
            Marker.lifetime = ros::Duration(lifetime);
            return self;
        }
    };

    template<typename T>
    struct frame_locked_helper {
        T &frame_locked(bool frame_locked)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.frame_locked = frame_locked;
            return self;
        }
    };

    template<typename T>
    struct points_helper {
        T &points(const std::vector<geometry_msgs::Point> &points)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.points = points;
            return self;
        }

        T &points(const std::vector<geometry_msgs::Point> &&points)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.points = std::move(points);
            return self;
        }
    };

    template<typename T>
    struct arrow_point_helper {
        T &start(const geometry_msgs::Point &start)
        {
            return set_point(start, 0);
        }

        T &end(const geometry_msgs::Point &end)
        {
            return set_point(end, 1);
        }

    private:
        T &set_point(const geometry_msgs::Point &point, size_t idx)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.points[idx] = point;
            return self;
        }
    };

    template<typename T>
    struct push_point_helper {
        T &push_point(const geometry_msgs::Point &point)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.points.push_back(point);
            return self;
        }
    };

    template<typename T>
    struct push_line_helper {
        T &push_line(const geometry_msgs::Point &start, const geometry_msgs::Point &end)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.points.push_back(start);
            marker.points.push_back(end);
            return self;
        }
    };

    template<typename T>
    struct push_triangle_helper {
        T &push_triangle(const geometry_msgs::Point &a, const geometry_msgs::Point &b, const geometry_msgs::Point &c)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.points.push_back(a);
            marker.points.push_back(b);
            marker.points.push_back(c);
            return self;
        }
    };

    template<typename T>
    struct colors_helper {
        T &colors(const std::vector<std_msgs::ColorRGBA> &colors)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.colors = colors;
            return self;
        }

        T &colors(const std::vector<std_msgs::ColorRGBA> &&colors)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.colors = std::move(colors);
            return self;
        }
    };

    template<typename T>
    struct text_helper {
        T &text(const std::string &text)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.text = text;
            return self;
        }

        T &text(const std::string &&text)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.text = std::move(text);
            return self;
        }
    };

    template<typename T>
    struct mesh_resource_helper {
        T &mesh_resource(const std::string &mesh_resource)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.mesh_resource = mesh_resource;
            return self;
        }

        T &mesh_resource(const std::string &&mesh_resource)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.mesh_resource = std::move(mesh_resource);
            return self;
        }

        T &mesh_use_embedded_materials(bool mesh_use_embedded_materials)
        {
            T &self = static_cast<T &>(*this);
            visualization_msgs::Marker &marker = self.msg();
            marker.mesh_use_embedded_materials = mesh_use_embedded_materials;
            return self;
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
    inline constexpr bool is_scale3_available_v =
        is_arrow_marker_v<MarkerType>
        || is_cube_marker_v<MarkerType>
        || is_sphere_marker_v<MarkerType>
        || is_cylinder_marker_v<MarkerType>
        || is_cube_list_marker_v<MarkerType>
        || is_sphere_list_marker_v<MarkerType>
        || is_mesh_resource_marker_v<MarkerType>
        || is_triangle_list_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_scale2_available_v =
        is_points_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_line_scale_available_v =
        is_line_strip_marker_v<MarkerType>
        || is_line_list_marker_v<MarkerType>;

    template<int32_t MarkerType>
    inline constexpr bool is_text_scale_available_v =
        is_text_view_facing_marker_v<MarkerType>;

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
        is_points_available_by_point_v<MarkerType>
        || is_points_available_by_line_v<MarkerType>
        || is_points_available_by_triangle_v<MarkerType>;

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
        : conditional_extend<
            is_arrow_marker_v<MarkerType> ? is_contained_v<ArrowOption::POSE, Options...> : true,
            position_helper<Marker<MarkerType, Options...>>>
    { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_orientation
        : conditional_extend<
            is_arrow_marker_v<MarkerType> ? is_contained_v<ArrowOption::POSE, Options...> : true,
            orientation_helper<Marker<MarkerType, Options...>>>
    { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_scale
        : conditional_extend<
            is_scale3_available_v<MarkerType>,
            scale3_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_scale2_available_v<MarkerType>,
            scale2_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_line_scale_available_v<MarkerType>,
            scale_width_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_text_scale_available_v<MarkerType>,
            scale_height_helper<Marker<MarkerType, Options...>>>
    { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_points
        : conditional_extend<
            is_arrow_marker_v<MarkerType> && is_contained_v<ArrowOption::VECTOR, Options...>,
            arrow_point_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_points_available_v<MarkerType> && is_contained_v<PointsOption::BULK_UPDATE, Options...>,
            points_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_points_available_by_point_v<MarkerType> && is_contained_v<PointsOption::SEPARATELY_UPDATE, Options...>,
            push_point_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_points_available_by_line_v<MarkerType> && is_contained_v<PointsOption::SEPARATELY_UPDATE, Options...>,
            push_line_helper<Marker<MarkerType, Options...>>>
        , conditional_extend<
            is_points_available_by_triangle_v<MarkerType> && is_contained_v<PointsOption::SEPARATELY_UPDATE, Options...>,
            push_triangle_helper<Marker<MarkerType, Options...>>>
    { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_colors
        : conditional_extend<
            is_colors_available_v<MarkerType> && is_contained_v<ColorsOption::BULK_UPDATE, Options...>,
            colors_helper<Marker<MarkerType, Options...>>>
    { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_text
        : conditional_extend<
            is_text_view_facing_marker_v<MarkerType>,
            text_helper<Marker<MarkerType, Options...>>>
    { };

    template<template<int32_t, auto...> typename Marker, int32_t MarkerType, auto... Options>
    struct conditional_mesh_resource
        : conditional_extend<
            is_mesh_resource_marker_v<MarkerType>,
            mesh_resource_helper<Marker<MarkerType, Options...>>>
    { };

} // namespace internal

namespace color {
    std_msgs::ColorRGBA generate_color(int32_t color, double opacity = 1)
    {
        std_msgs::ColorRGBA res;
        int32_t mask = (1 << 8) - 1;
        res.r = ((color >> 16) & mask) / 255.0;
        res.g = ((color >> 8) & mask) / 255.0;
        res.b = (color & mask) / 255.0;
        res.a = opacity;
        return res;
    }

    inline const static std_msgs::ColorRGBA RED = generate_color(0xF44336);
    inline const static std_msgs::ColorRGBA PINK = generate_color(0xFF4081);
    inline const static std_msgs::ColorRGBA PURPLE = generate_color(0x9C27B0);
    inline const static std_msgs::ColorRGBA DEEP_PURPLE = generate_color(0x7B1FA2);
    inline const static std_msgs::ColorRGBA INDIGO = generate_color(0x3F51B5);
    inline const static std_msgs::ColorRGBA BRUE = generate_color(0x448aff);
    inline const static std_msgs::ColorRGBA LIGHT_BRUE = generate_color(0x03A9F4);
    inline const static std_msgs::ColorRGBA CYAN = generate_color(0x00BCD4);
    inline const static std_msgs::ColorRGBA TEAL = generate_color(0x009688);
    inline const static std_msgs::ColorRGBA GREEN = generate_color(0x4CAF50);
    inline const static std_msgs::ColorRGBA LIGHT_GREEN = generate_color(0x8BC34A);
    inline const static std_msgs::ColorRGBA LIME = generate_color(0xCDDC39);
    inline const static std_msgs::ColorRGBA YELLOW = generate_color(0xFFEB3B);
    inline const static std_msgs::ColorRGBA AMBER = generate_color(0xFFC107);
    inline const static std_msgs::ColorRGBA ORANGE = generate_color(0xFF9800);
    inline const static std_msgs::ColorRGBA DEEP_ORANGE = generate_color(0xFF5722);
    inline const static std_msgs::ColorRGBA BROWN = generate_color(0x795548);
    inline const static std_msgs::ColorRGBA GREY = generate_color(0x9E9E9E);
    inline const static std_msgs::ColorRGBA BLUE_GREY = generate_color(0x607D8B);
    inline const static std_msgs::ColorRGBA WHITE = generate_color(0xffffff);
    inline const static std_msgs::ColorRGBA BLACK = generate_color(0x000000);

} // namespace color

template<int32_t MarkerType, auto... Options>
class Marker
    : public internal::conditional_position<Marker, MarkerType, Options...>
    , public internal::conditional_orientation<Marker, MarkerType, Options...>
    , public internal::conditional_scale<Marker, MarkerType, Options...>
    , public internal::color_helper<Marker<MarkerType, Options...>>
    , public internal::lifetime_helper<Marker<MarkerType, Options...>>
    , public internal::frame_locked_helper<Marker<MarkerType, Options...>>
    , public internal::conditional_points<Marker, MarkerType, Options...>
    , public internal::conditional_colors<Marker, MarkerType, Options...>
    , public internal::conditional_text<Marker, MarkerType, Options...>
    , public internal::conditional_mesh_resource<Marker, MarkerType, Options...> {

    visualization_msgs::Marker marker;

public:
    Marker(int32_t id, std::string ns = "")
    {
        marker.id = id;
        marker.ns = ns;
        marker.type = MarkerType;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        if constexpr (internal::is_scale3_available_v<MarkerType>) {
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
        }

        if constexpr (internal::is_scale2_available_v<MarkerType>) {
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
        }

        if constexpr (internal::is_line_scale_available_v<MarkerType>) {
            marker.scale.x = 0.05;
        }

        if constexpr (
            internal::is_arrow_marker_v<MarkerType>
            && internal::is_contained_v<ArrowOption::VECTOR, Options...>) {
            marker.points.resize(2);
        }

        if constexpr (internal::is_text_view_facing_marker_v<MarkerType>) {
            marker.scale.z = 1;
            marker.text = "visualization_msgs::Marker::TEXT_VIEW_FACING";
        }
    }

    visualization_msgs::Marker &msg() { return marker; }
};

using ArrowMarker = Marker<
    visualization_msgs::Marker::ARROW,
    ArrowOption::VECTOR>;

using CubeMarker = Marker<visualization_msgs::Marker::CUBE>;

using SphereMarker = Marker<visualization_msgs::Marker::SPHERE>;

using CylinderMarker = Marker<visualization_msgs::Marker::CYLINDER>;

using LineStripMarker = Marker<
    visualization_msgs::Marker::LINE_STRIP,
    PointsOption::BULK_UPDATE>;

using LineListMarker = Marker<
    visualization_msgs::Marker::LINE_LIST,
    PointsOption::BULK_UPDATE>;

using CubeListMarker = Marker<
    visualization_msgs::Marker::CUBE_LIST,
    PointsOption::BULK_UPDATE>;

using SphereListMarker = Marker<
    visualization_msgs::Marker::SPHERE_LIST,
    PointsOption::BULK_UPDATE>;

using PointsMarker = Marker<
    visualization_msgs::Marker::POINTS,
    PointsOption::BULK_UPDATE>;

using TextViewFacingMarker = Marker<visualization_msgs::Marker::TEXT_VIEW_FACING>;

using MeshResourceMarker = Marker<visualization_msgs::Marker::MESH_RESOURCE>;

using TriangleListMarker = Marker<
    visualization_msgs::Marker::TRIANGLE_LIST,
    PointsOption::BULK_UPDATE>;

class Rviz {
    ros::NodeHandle nh;
    ros::Publisher pub;

    std::string frame_id_;

public:
    Rviz(std::string frame_id = "map", std::string topic = "visualization_marker")
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

    void delete_marker(int32_t id, std::string ns = "")
    {
        visualization_msgs::Marker marker;
        marker.id = id;
        marker.ns = ns;
        marker.action = visualization_msgs::Marker::DELETE;
        publish(marker);
    }

    void delete_all_marker()
    {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        publish(marker);
    }

    void publish(visualization_msgs::Marker &marker)
    {
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        pub.publish(marker);
    }
};

} // namespace flrviz
