#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename Derived>
struct CylinderProperty : public MarkerPropertyBase<Derived>
{
private:
  using Base = MarkerPropertyBase<Derived>;

public:
  CylinderProperty()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::CYLINDER);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::scale;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
};

template <typename MarkerToken = UseTemporal>
auto Cylinder(MarkerToken && token = MarkerToken{})
{
  return compose_marker<CylinderProperty>(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
