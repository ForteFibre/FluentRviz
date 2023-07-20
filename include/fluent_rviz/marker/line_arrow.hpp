#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct LineArrowMarker : public MarkerBase<MarkerToken, LineArrowMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, LineArrowMarker<MarkerToken>>;

public:
  explicit LineArrowMarker(MarkerToken token = { })
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

  auto scale(double shaft_diameter, double head_diameter, double head_length) && noexcept
  -> LineArrowMarker &&
  {
    return Base::scale(shaft_diameter, head_diameter, head_length);
  }

  auto points(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end) && noexcept
  -> LineArrowMarker &&
  {
    return Base::points({ start, end });
  }
};

template <typename MarkerToken = UseTemporal>
auto LineArrow(MarkerToken &&token = { }) -> LineArrowMarker<MarkerToken>
{
  return { std::forward<MarkerToken>(token) };
}
}  // namespace flrv::marker
