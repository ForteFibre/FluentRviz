#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"

namespace flrv::marker
{
template <typename Derived>
struct TextViewFacing : public MarkerBase<Derived>
{
private:
  using Base = MarkerBase<Derived>;

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
  -> Derived &&
  {
    return Base::scale(0, 0, height);
  }
};
}  // namespace flrv::marker
