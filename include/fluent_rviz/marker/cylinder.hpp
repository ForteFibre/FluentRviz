#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct CylinderMarker : public MarkerBase<MarkerToken, CylinderMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, CylinderMarker<MarkerToken>>;

public:
  explicit CylinderMarker(MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
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
auto Cylinder(MarkerToken &&token = { }) -> CylinderMarker<MarkerToken>
{
  return { std::forward<MarkerToken>(token) };
}
}  // namespace flrv::marker
