#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename Derived>
struct LineArrowProperty : public MarkerPropertyBase<Derived>
{
private:
  using Base = MarkerPropertyBase<Derived>;

public:
  LineArrowProperty()
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

  auto scale(double shaft_diameter, double head_diameter, double head_length) noexcept
  -> Derived &
  {
    return Base::scale(shaft_diameter, head_diameter, head_length);
  }

  auto points(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end) noexcept
  -> Derived &
  {
    return Base::points({ start, end });
  }
};

template <typename MarkerToken = UseTemporal>
auto LineArrow(MarkerToken && token = UseTemporal{})
{
  return compose<LineArrowProperty>(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
