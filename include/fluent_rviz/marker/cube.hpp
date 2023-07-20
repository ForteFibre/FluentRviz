#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct CubeMarker : public MarkerBase<MarkerToken, CubeMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, CubeMarker<MarkerToken>>;

public:
  CubeMarker(MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::CUBE);
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
auto Cube(MarkerToken &&token = { }) -> CubeMarker<MarkerToken>
{
  return { std::forward<MarkerToken>(token) };
}
}  // namespace flrv::marker
