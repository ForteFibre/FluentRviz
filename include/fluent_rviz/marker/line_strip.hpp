#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/marker_wrapper.hpp"

namespace flrv::marker
{
template <typename MarkerToken = MarkerWrapper>
struct LineStrip : public MarkerBase<MarkerToken, LineStrip<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, LineStrip<MarkerToken>>;

public:
  LineStrip()
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
  -> LineStrip &&
  {
    return Base::scale(width, 0, 0);
  }
};
}  // namespace flrv::marker
