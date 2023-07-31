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
  explicit LineStripMarker(std::string frame_id, MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  {
    this->marker().header.frame_id = std::move(frame_id);
    this->marker().action = visualization_msgs::msg::Marker::ADD;
    this->marker().type = visualization_msgs::msg::Marker::LINE_STRIP;
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::colors;

  auto scale(double width) && noexcept -> LineStripMarker &&
  { return Base::scale(width, 0, 0); }
};

template <typename MarkerToken = UseTemporal>
auto LineStrip(std::string frame_id, MarkerToken &&token = { })
{ return LineStripMarker<MarkerToken>{ std::move(frame_id), std::forward<MarkerToken>(token) }; }
}  // namespace flrv::marker
