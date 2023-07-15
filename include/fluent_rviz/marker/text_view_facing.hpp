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
  explicit TextViewFacingMarker(MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::text;

  auto scale(double height) && noexcept
  -> TextViewFacingMarker &&
  {
    return Base::scale(0, 0, height);
  }
};

template <typename MarkerToken = UseTemporal>
auto TextViewFacing(MarkerToken &&token = { })
{
  return TextViewFacingMarker<MarkerToken>{ std::forward<MarkerToken>(token) };
}
}  // namespace flrv::marker
