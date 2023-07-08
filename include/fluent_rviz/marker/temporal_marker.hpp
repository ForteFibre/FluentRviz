#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"

namespace flrv::marker
{
template <template <typename Derived> typename MarkerProperty>
struct TemporalMarker : public MarkerProperty<TemporalMarker<MarkerProperty>>
{
private:
  friend MarkerPropertyBase<TemporalMarker<MarkerProperty>>;

  visualization_msgs::msg::Marker _marker;

  auto get() noexcept -> visualization_msgs::msg::Marker &
  {
    return _marker;
  }

public:
  operator visualization_msgs::msg::Marker &() noexcept
  {
    return _marker;
  }
};

struct UseTemporal { };

template <template <typename Derived> typename MarkerProperty>
struct detail::MarkerComposition<MarkerProperty, UseTemporal>
{
  static auto get(UseTemporal)
  {
    return TemporalMarker<MarkerProperty>();
  }
};
}  // namespace flrv::marker
