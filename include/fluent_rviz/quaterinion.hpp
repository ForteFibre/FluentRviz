#pragma once

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>

#include "fluent_rviz/point.hpp"
#include "fluent_rviz/traits.hpp"

namespace flrv::quaternion
{
struct Quaternion : public geometry_msgs::msg::Quaternion
{
  Quaternion() = default;

  template <
    typename QuaternionLike,
    traits::Require<
      traits::ConversionDefined<geometry_msgs::msg::Quaternion, QuaternionLike>> = nullptr>
  Quaternion(const QuaternionLike &quaternion)
    : geometry_msgs::msg::Quaternion{ traits::convert<geometry_msgs::msg::Quaternion>(quaternion) }
  { }

  Quaternion(const double x, const double y, const double z, const double w) noexcept
  { this->x = x, this->y = y, this->z = z, this->w = w; }
};

[[nodiscard]]
inline auto Zero() noexcept -> Quaternion
{ return { 0, 0, 0, 0 }; }

[[nodiscard]]
inline auto Identity() noexcept -> Quaternion
{ return { 0, 0, 0, 1 }; }

[[nodiscard]]
inline auto ScalarVector(double scalar, const point::Point &vector) noexcept -> Quaternion
{ return { vector.x, vector.y, vector.z, scalar }; }

[[nodiscard]]
inline auto AngleAxis(double angle, const point::Point &axis) noexcept -> Quaternion
{ return ScalarVector(std::cos(angle / 2), axis * std::sin(angle / 2)); }

[[nodiscard]]
inline auto AngleAxisX(double angle) noexcept -> Quaternion
{ return AngleAxis(angle, { 1, 0, 0 }); }

[[nodiscard]]
inline auto AngleAxisY(double angle) noexcept -> Quaternion
{ return AngleAxis(angle, { 0, 1, 0 }); }

[[nodiscard]]
inline auto AngleAxisZ(double angle) noexcept -> Quaternion
{ return AngleAxis(angle, { 0, 0, 1 }); }

[[nodiscard]]
inline auto scalar(const Quaternion &q) noexcept -> double
{ return q.w; }

[[nodiscard]]
inline auto vector(const Quaternion &q) noexcept -> point::Point
{ return { q.x, q.y, q.z }; }

[[nodiscard]]
inline auto operator==(const Quaternion &l, const Quaternion &r) noexcept -> bool
{ return l.x == r.x and l.y == r.y and l.z == r.z and l.w == r.w; }

[[nodiscard]]
inline auto operator!=(const Quaternion &l, const Quaternion &r) noexcept -> bool
{ return l.x != r.x or l.y != r.y or l.z != r.z or l.w != r.w; }

[[nodiscard]]
inline auto operator+(const Quaternion &q) noexcept -> Quaternion
{ return q; }

[[nodiscard]]
inline auto operator-(const Quaternion &q) noexcept -> Quaternion
{ return { -q.x, -q.y, -q.z, -q.w }; }

[[nodiscard]]
inline auto operator+(const Quaternion &l, const Quaternion &r) noexcept -> Quaternion
{ return { l.x + r.x, l.y + r.y, l.z + r.z, l.w + r.w }; }

[[nodiscard]]
inline auto operator-(const Quaternion &l, const Quaternion &r) noexcept -> Quaternion
{ return { l.x - r.x, l.y - r.y, l.z - r.z, l.w - r.w }; }

[[nodiscard]]
inline auto operator*(const Quaternion &l, const Quaternion &r) noexcept -> Quaternion
{
  return ScalarVector(
    scalar(l) * scalar(r) - dot(vector(l), vector(r)),
    scalar(l) * vector(r) + scalar(r) * vector(l) + cross(vector(l), vector(r)));
}

[[nodiscard]]
inline auto operator*(const Quaternion &l, double r) noexcept -> Quaternion
{ return { l.x * r, l.y * r, l.z * r, l.w * r }; }

[[nodiscard]]
inline auto operator*(double l, const Quaternion &r) noexcept -> Quaternion
{ return { l * r.x, l * r.y, l * r.z, l * r.w }; }

[[nodiscard]]
inline auto operator/(const Quaternion &l, double r) noexcept -> Quaternion
{ return { l.x / r, l.y / r, l.z / r, l.w / r }; }

[[nodiscard]]
inline auto squared_norm(const Quaternion &q) noexcept -> double
{ return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w; }

[[nodiscard]]
inline auto norm(const Quaternion &q) noexcept -> double
{ return std::sqrt(squared_norm(q)); }

[[nodiscard]]
inline auto normalized(const Quaternion &q) noexcept -> Quaternion
{ return q / norm(q); }

[[nodiscard]]
inline auto dot(const Quaternion &l, const Quaternion &r) noexcept -> double
{ return l.x * r.x + l.y * r.y + l.z * r.z + l.w * r.w; }

[[nodiscard]]
inline auto conjugate(const Quaternion &q) noexcept -> Quaternion
{ return { -q.x, -q.y, -q.z, q.w }; }

[[nodiscard]]
inline auto inverse(const Quaternion &q) noexcept -> Quaternion
{ return conjugate(q) / norm(q); }

[[nodiscard]]
inline auto rotate(const Quaternion &q, const point::Point &p) noexcept -> point::Point
{ return vector(q * ScalarVector(0, p) * inverse(q)); }
}  // flrv::quaternion

template <>
struct flrv::traits::Convert<geometry_msgs::msg::Quaternion, flrv::quaternion::Quaternion>
{
  static auto do_convert(const flrv::quaternion::Quaternion &quaternion) noexcept -> geometry_msgs::msg::Quaternion
  { return quaternion; }
};
