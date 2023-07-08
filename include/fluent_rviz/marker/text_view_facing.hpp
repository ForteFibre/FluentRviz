#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename Derived>
struct TextViewFacingProperty : public MarkerPropertyBase<Derived>
{
private:
  using Base = MarkerPropertyBase<Derived>;

public:
  TextViewFacingProperty()
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

  auto scale(double height) noexcept
  -> Derived &
  {
    return Base::scale(0, 0, height);
  }
};

template <typename MarkerToken = UseTemporal>
auto TextViewFacing(MarkerToken && token = UseTemporal{})
{
  return compose_marker<TextViewFacingProperty>(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
