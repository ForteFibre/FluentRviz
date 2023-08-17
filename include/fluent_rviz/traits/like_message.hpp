#pragma once

#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/uv_coordinate.hpp>

#include "fluent_rviz/traits/convert.hpp"

namespace flrv::traits
{
template <typename T>
inline constexpr bool like_point = is_convertible<geometry_msgs::msg::Point, T>;

template <typename T>
inline constexpr bool like_quaternion = is_convertible<geometry_msgs::msg::Quaternion, T>;

template <typename T>
inline constexpr bool like_pose = is_convertible<geometry_msgs::msg::Pose, T>;

template <typename T>
inline constexpr bool like_color = is_convertible<std_msgs::msg::ColorRGBA, T>;

template <typename T>
inline constexpr bool like_uv_coordinate = is_convertible<visualization_msgs::msg::UVCoordinate, T>;


template <typename T>
inline constexpr bool like_points = is_convertible<std::vector<geometry_msgs::msg::Point>, T>;

template <typename T>
inline constexpr bool like_colors = is_convertible<std::vector<std_msgs::msg::ColorRGBA>, T>;

template <typename T>
inline constexpr bool like_uv_coordinates = is_convertible<std::vector<visualization_msgs::msg::UVCoordinate>, T>;
}  // namespace flrv::traits
