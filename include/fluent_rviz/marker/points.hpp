#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct PointsMarker : public MarkerBase<MarkerToken, PointsMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, PointsMarker<MarkerToken>>;

public:
  explicit PointsMarker(MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::POINTS);
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
auto Points(MarkerToken &&token = { })
{
  return PointsMarker<MarkerToken>{ std::forward<MarkerToken>(token) };
}
}  // namespace flrv::marker
