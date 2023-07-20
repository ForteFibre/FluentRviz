#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct PoseArrowMarker : public MarkerBase<MarkerToken, PoseArrowMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, PoseArrowMarker<MarkerToken>>;

public:
  explicit PoseArrowMarker(MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
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
  -> PoseArrowMarker &
  {
    return Base::scale(length, width, height);
  }
};

template <typename MarkerToken = UseTemporal>
auto PoseArrow(MarkerToken &&token = { }) -> PoseArrowMarker<MarkerToken>
{
  return { std::forward<MarkerToken>(token) };
}
}  // namespace flrv::marker
