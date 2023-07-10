#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename Derived>
struct TriangleListProperty : public MarkerPropertyBase<Derived>
{
private:
  using Base = MarkerPropertyBase<Derived>;

public:
  TriangleListProperty()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::TRIANGLE_LIST);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::scale;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::points;
  using Base::colors;
};

template <typename MarkerToken = UseTemporal>
auto TriangleList(MarkerToken && token = MarkerToken{})
{
  return compose_marker<TriangleListProperty>(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
