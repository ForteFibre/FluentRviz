#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"

namespace flrv::marker
{
template <typename Derived>
struct PoseArrow : public MarkerBase<Derived>
{
private:
  using Base = MarkerBase<Derived>;

public:
  PoseArrow()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::ARROW);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;

  auto scale(double length, double width, double height) && noexcept
  -> PoseArrow &&
  {
    return Base::scale(length, width, height);
  }
};
}  // namespace flrv::marker
