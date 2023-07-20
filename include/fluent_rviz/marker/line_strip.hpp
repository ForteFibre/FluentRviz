#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct LineStripMarker : public MarkerBase<MarkerToken, LineStripMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, LineStripMarker<MarkerToken>>;

public:
  explicit LineStripMarker(MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
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

  auto scale(double width) && noexcept
  -> LineStripMarker &&
  {
    return Base::scale(width, 0, 0);
  }
};

template <typename MarkerToken = UseTemporal>
auto LineStrip(MarkerToken &&token = { }) -> LineStripMarker<MarkerToken>
{
  return { std::forward<MarkerToken>(token) };
}
}  // namespace flrv::marker
