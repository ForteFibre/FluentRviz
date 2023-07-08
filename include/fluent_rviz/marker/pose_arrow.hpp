#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename Derived>
struct PoseArrowProperty : public MarkerPropertyBase<Derived>
{
private:
  using Base = MarkerPropertyBase<Derived>;

public:
  PoseArrowProperty()
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

  auto scale(double length, double width, double height) noexcept
  -> Derived &
  {
    return Base::scale(length, width, height);
  }
};

template <typename MarkerToken = UseTemporal>
auto PoseArrow(MarkerToken && token = UseTemporal{})
{
  return compose<PoseArrowProperty>(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
