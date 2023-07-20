#pragma once

#include <cstddef>
#include <geometry_msgs/msg/quaternion.hpp>

#include "fluent_rviz/quaternion/access.hpp"

namespace flrv::quaternion
{
struct Quaternion : public geometry_msgs::msg::Quaternion
{
  using geometry_msgs::msg::Quaternion::Quaternion;

  Quaternion(double x, double y, double z, double w) noexcept
  { this->x = x, this->y = y, this->z = z, this->w = w; }
};

template <>
struct Access<geometry_msgs::msg::Quaternion>
{
  template <std::size_t Index>
  static auto get(const geometry_msgs::msg::Quaternion &quaternion) noexcept -> double
  {
    if constexpr (Index == 0) return quaternion.x;
    if constexpr (Index == 1) return quaternion.y;
    if constexpr (Index == 2) return quaternion.z;
    if constexpr (Index == 3) return quaternion.w;
  }

  template <std::size_t Index>
  static auto set(geometry_msgs::msg::Quaternion &quaternion, double value) noexcept -> void
  {
    if constexpr (Index == 0) quaternion.x = value;
    if constexpr (Index == 1) quaternion.y = value;
    if constexpr (Index == 2) quaternion.z = value;
    if constexpr (Index == 3) quaternion.w = value;
  }
};

template <>
struct Access<Quaternion> : public Access<geometry_msgs::msg::Quaternion>
{ };
}  // flrv::quaternion
