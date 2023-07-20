#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include "fluent_rviz/point/point.hpp"
#include "fluent_rviz/quaternion/quaterinion.hpp"

namespace flrv::pose
{
struct Pose : public geometry_msgs::msg::Pose
{
  using geometry_msgs::msg::Pose::Pose;

  template <
    typename PointLike = point::Point,
    typename QuaternionLike = quaternion::Quaternion>
  Pose(const PointLike &position, const QuaternionLike &orientation)
  {
    this->position = position, this->orientation = orientation;
  }
};
}  // namespace flrv::pose
