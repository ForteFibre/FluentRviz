#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct LineListMarker : public MarkerBase<MarkerToken, LineListMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, LineListMarker<MarkerToken>>;

public:
  explicit LineListMarker(MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::LINE_LIST);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::points;
  using Base::colors;

  auto scale(double width) && noexcept
  -> LineListMarker &&
  {
    return Base::scale(width, 0, 0);
  }
};

template <typename MarkerToken = UseTemporal>
auto LineList(MarkerToken &&token)
{
  return LineListMarker<MarkerToken>{ std::forward<MarkerToken>(token) };
}
}  // namespace flrv::marker
