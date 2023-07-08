#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename Derived>
struct DeleteProperty : public MarkerPropertyBase<Derived>
{
private:
  using Base = MarkerPropertyBase<Derived>;

public:
  DeleteProperty()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::DELETE);
  }

  using Base::ns;
  using Base::id;
};

template <typename MarkerToken = UseTemporal>
auto Delete(MarkerToken && token = UseTemporal{})
{
  return compose_marker<DeleteProperty>(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
