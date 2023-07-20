#pragma once

#include <cstddef>
#include <memory>

#include <geometry_msgs/msg/point.hpp>

#include "fluent_rviz/point/access.hpp"

namespace flrv::point
{
struct Point : public geometry_msgs::msg::Point
{
  using geometry_msgs::msg::Point::Point;

  Point(double x, double y, double z) noexcept
  { this->x = x, this->y = y, this->z = z; }
};

template <>
struct Access<geometry_msgs::msg::Point>
{
  template <std::size_t Index>
  static auto get(const geometry_msgs::msg::Point &point) noexcept -> double
  {
    if constexpr (Index == 0) return point.x;
    if constexpr (Index == 1) return point.y;
    if constexpr (Index == 2) return point.z;
  }

  template <std::size_t Index>
  static auto set(geometry_msgs::msg::Point &point, double value) noexcept -> void
  {
    if constexpr (Index == 0) point.x = value;
    if constexpr (Index == 1) point.y = value;
    if constexpr (Index == 2) point.z = value;
  }
};

template <>
struct Access<Point> : public Access<geometry_msgs::msg::Point>
{ };
}  // namespace flrv::point
