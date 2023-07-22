#pragma once

#include <cmath>

#include <geometry_msgs/msg/point.hpp>

#include "fluent_rviz/traits/convert.hpp"

namespace flrv::point
{
struct Point : public geometry_msgs::msg::Point
{
  using geometry_msgs::msg::Point::Point;

  template <
    typename PointLike,
    typename = decltype(traits::Convert<geometry_msgs::msg::Point, PointLike>{ })>
  Point(const PointLike &p)
    : geometry_msgs::msg::Point{ traits::convert<geometry_msgs::msg::Point>(p) }
  { }

  Point(const double x, const double y, const double z) noexcept
  { this->x = x, this->y = y, this->z = z; }
};

[[nodiscard]]
inline auto Zero() noexcept -> Point
{ return { 0, 0, 0 }; }

[[nodiscard]]
inline auto UnitX() noexcept -> Point
{ return { 1, 0, 0 }; }

[[nodiscard]]
inline auto UnitY() noexcept -> Point
{ return { 0, 1, 0 }; }

[[nodiscard]]
inline auto UnitZ() noexcept -> Point
{ return { 0, 0, 1 }; }

[[nodiscard]]
inline auto operator==(const Point &l, const Point &r) noexcept -> bool
{ return l.x == r.x and l.y == r.y and l.z == r.z; }

[[nodiscard]]
inline auto operator!=(const Point &l, const Point &r) noexcept -> bool
{ return l.x != r.x or l.y != r.y or l.z != r.z; }

[[nodiscard]]
inline auto operator+(const Point &p) noexcept -> Point
{ return p; }

[[nodiscard]]
inline auto operator-(const Point &p) noexcept -> Point
{ return { -p.x, -p.y, -p.z }; }

[[nodiscard]]
inline auto operator+(const Point &l, const Point &r) noexcept -> Point
{ return { l.x + r.x, l.y + r.y, l.z + r.z }; }

[[nodiscard]]
inline auto operator-(const Point &l, const Point &r) noexcept -> Point
{ return { l.x - r.x, l.y - r.y, l.z - r.z }; }

[[nodiscard]]
inline auto operator*(const Point &l, double r) noexcept -> Point
{ return { l.x * r, l.y * r, l.z * r }; }

[[nodiscard]]
inline auto operator*(double l, const Point &r) noexcept -> Point
{ return { l * r.x, l * r.y, l * r.z }; }

[[nodiscard]]
inline auto operator/(const Point &l, double r) noexcept -> Point
{ return { l.x / r, l.y / r, l.z / r }; }

[[nodiscard]]
inline auto squared_norm(const Point &p) noexcept -> double
{ return p.x * p.x + p.y * p.y + p.z * p.z; }

[[nodiscard]]
inline auto norm(const Point &p) noexcept -> double
{ return std::sqrt(squared_norm(p)); }

[[nodiscard]]
inline auto normalized(const Point &p) noexcept -> Point
{ return p / norm(p); }

[[nodiscard]]
inline auto dot(const Point &l, const Point &r) noexcept -> double
{ return l.x * r.x + l.y * r.y + l.z * r.z; }

[[nodiscard]]
inline auto cross(const Point &l, const Point &r) noexcept -> Point
{ return { l.y * r.z - l.z * r.y, l.z * r.x - l.x * r.z, l.x * r.y - l.y * r.x }; }
}  // namespace flrv::point

template <>
struct flrv::traits::Convert<geometry_msgs::msg::Point, flrv::point::Point>
{
  static auto do_convert(const flrv::point::Point &point) noexcept -> geometry_msgs::msg::Point
  { return point; }
};
