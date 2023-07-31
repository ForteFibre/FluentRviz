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
  explicit LineArrowMarker(std::string frame_id, MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  {
    this->marker().header.frame_id = std::move(frame_id);
    this->marker().action = visualization_msgs::msg::Marker::ADD;
    this->marker().type = visualization_msgs::msg::Marker::ARROW;
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;

  auto scale(double shaft_diameter, double head_diameter, double head_length) && noexcept -> LineArrowMarker &&
  { return Base::scale(shaft_diameter, head_diameter, head_length); }

  auto points(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end) && noexcept -> LineArrowMarker &&
  { return Base::points({ start, end }); }
};

template <typename MarkerToken = UseTemporal>
auto LineArrow(std::string frame_id, MarkerToken &&token = { })
{ return LineArrowMarker<MarkerToken>{ std::move(frame_id), std::forward<MarkerToken>(token) }; }
}  // namespace flrv::marker
