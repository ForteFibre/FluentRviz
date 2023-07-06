#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/marker_wrapper.hpp"

namespace flrv::marker
{
template <typename MarkerToken = MarkerWrapper>
struct LineArrow : public MarkerBase<MarkerToken, LineArrow<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, LineArrow<MarkerToken>>;

public:
  LineArrow()
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
  -> LineArrow &&
  {
    return Base::scale(shaft_diameter, head_diameter, head_length);
  }

  auto points(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end) && noexcept
  -> LineArrow &&
  {
    return Base::points({ start, end });
  }
};
}  // namespace flrv::marker
