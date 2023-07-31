#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct TextViewFacingMarker : public MarkerBase<MarkerToken, TextViewFacingMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, TextViewFacingMarker<MarkerToken>>;

public:
  explicit TextViewFacingMarker(std::string frame_id, MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  {
    this->marker().header.frame_id = std::move(frame_id);
    this->marker().action = visualization_msgs::msg::Marker::ADD;
    this->marker().type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::text;

  auto scale(double height) && noexcept -> TextViewFacingMarker &&
  { return Base::scale(0, 0, height); }
};

template <typename MarkerToken = UseTemporal>
auto TextViewFacing(std::string frame_id, MarkerToken &&token = { })
{ return TextViewFacingMarker<MarkerToken>{ std::move(frame_id), std::forward<MarkerToken>(token) }; }
}  // namespace flrv::marker
