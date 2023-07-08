#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename Derived>
struct DeleteAllProperty : public MarkerPropertyBase<Derived>
{
private:
  using Base = MarkerPropertyBase<Derived>;

public:
  DeleteAllProperty()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::DELETEALL);
  }

  using Base::ns;
};

template <typename MarkerToken = UseTemporal>
auto DeleteAll(MarkerToken && token = UseTemporal{})
{
  return compose_marker<DeleteAllProperty>(std::forward<MarkerToken>(token));
}
}  // namespace flrv::marker
