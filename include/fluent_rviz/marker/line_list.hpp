#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename Derived>
struct LineListProperty : public MarkerPropertyBase<Derived>
{
private:
  using Base = MarkerPropertyBase<Derived>;

public:
  LineListProperty()
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

  auto scale(double width) noexcept
  -> Derived &
  {
    return Base::scale(width, 0, 0);
  }
};

template <typename MarkerToken = UseTemporal>
auto LineList(MarkerToken && token = UseTemporal{})
{
  return compose<LineList>(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
