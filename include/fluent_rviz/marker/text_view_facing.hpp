#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/marker_wrapper.hpp"

namespace flrv::marker
{
template <typename MarkerToken = MarkerWrapper>
struct TextViewFacing : public MarkerBase<MarkerToken, TextViewFacing<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, TextViewFacing<MarkerToken>>;

public:
  TextViewFacing()
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
  -> TextViewFacing &&
  {
    return Base::scale(0, 0, height);
  }
};
}  // namespace flrv::marker
