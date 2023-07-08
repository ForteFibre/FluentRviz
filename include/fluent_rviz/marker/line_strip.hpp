#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename Derived>
struct LineStripProperty : public MarkerPropertyBase<Derived>
{
private:
  using Base = MarkerPropertyBase<Derived>;

public:
  LineStripProperty()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::LINE_STRIP);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::colors;

  auto scale(double width) noexcept
  -> Derived &
  {
    return Base::scale(width, 0, 0);
  }
};

template <typename MarkerToken = UseTemporal>
auto LineStrip(MarkerToken && token = UseTemporal{})
{
  return compose_marker<LineStripProperty>(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
