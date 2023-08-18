#pragma once

#include <cstddef>
#include <type_traits>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "fluent_rviz/point.hpp"
#include "fluent_rviz/quaterinion.hpp"
#include "fluent_rviz/traits/convert.hpp"
#include "fluent_rviz/traits/like_message.hpp"

namespace flrv::pose
{
struct Pose : public geometry_msgs::msg::Pose
{
  Pose() = default;

  template <
    typename PoseLike,
    std::enable_if_t<traits::like_pose<PoseLike>, std::nullptr_t> = nullptr>
  Pose(const PoseLike &pose)
    : geometry_msgs::msg::Pose{ traits::convert<geometry_msgs::msg::Pose>(pose) }
  { }

  template <
    typename PointLike = point::Point,
    typename QuaternionLike = quaternion::Quaternion,
    std::enable_if_t<
      traits::like_point<PointLike>
        and traits::like_quaternion<QuaternionLike>, std::nullptr_t> = nullptr>
  Pose(const PointLike &position, const QuaternionLike &orientation)
  {
    this->position = traits::convert<geometry_msgs::msg::Point>(position);
    this->orientation = traits::convert<geometry_msgs::msg::Quaternion>(orientation);
  }
};

[[nodiscard]]
inline auto Identity() noexcept -> Pose
{ return { point::Zero(), quaternion::Identity() }; }

template <
  typename PointLike = point::Point,
  std::enable_if_t<traits::like_point<PointLike>, std::nullptr_t> = nullptr>
[[nodiscard]]
auto Position(const PointLike &point) noexcept -> Pose
{ return { point, quaternion::Identity() }; }

template <
  typename QuaternionLike = quaternion::Quaternion,
  std::enable_if_t<traits::like_quaternion<QuaternionLike>, std::nullptr_t> = nullptr>
[[nodiscard]]
auto Orientation(const QuaternionLike &quaternion) noexcept -> Pose
{ return { point::Zero(), quaternion }; }

[[nodiscard]]
inline auto position(const Pose &p) noexcept -> point::Point
{ return p.position; }

[[nodiscard]]
inline auto orientation(const Pose &p) noexcept -> quaternion::Quaternion
{ return p.orientation; }

[[nodiscard]]
inline auto operator==(const Pose &l, const Pose &r) noexcept -> bool
{ return position(l) == position(r) and orientation(l) == orientation(r); }

[[nodiscard]]
inline auto operator!=(const Pose &l, const Pose &r) noexcept -> bool
{ return position(l) != position(r) or orientation(l) != orientation(r); }

[[nodiscard]]
inline auto operator*(const Pose &l, const Pose &r) noexcept -> Pose
{ return { rotate(orientation(l), position(r)) + position(l), orientation(l) * orientation(r) }; }

[[nodiscard]]
inline auto inverse(const Pose &p) noexcept -> Pose
{ return { -rotate(inverse(orientation(p)), position(p)), inverse(orientation(p)) }; }

[[nodiscard]]
inline auto transform(const Pose &l, const point::Point &r) noexcept -> point::Point
{ return { rotate(orientation(l), r) + position(l) }; }
}  // namespace flrv::pose

template <>
struct flrv::traits::Converter<geometry_msgs::msg::Pose, flrv::pose::Pose>
{
  static auto do_convert(const flrv::pose::Pose &pose) noexcept -> geometry_msgs::msg::Pose
  { return pose; }
};
