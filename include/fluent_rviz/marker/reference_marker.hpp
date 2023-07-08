#pragma once

#include <functional>
#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"

namespace flrv::marker
{
template <template <typename Derived> typename MarkerProperty>
struct ReferenceMarker : public MarkerProperty<ReferenceMarker<MarkerProperty>>
{
private:
  friend MarkerPropertyBase<ReferenceMarker<MarkerProperty>>;

  visualization_msgs::msg::Marker & _marker;

  auto get() noexcept -> visualization_msgs::msg::Marker &
  {
    return _marker;
  }

public:
  ReferenceMarker(const ReferenceMarker &) = delete;

  ReferenceMarker(ReferenceMarker &&) = delete;

  ReferenceMarker(visualization_msgs::msg::Marker & marker)
    : _marker(marker)
  { }

  operator visualization_msgs::msg::Marker &() noexcept
  {
    return _marker;
  }
};

template <template <typename Derived> typename MarkerProperty>
struct detail::MarkerComposition<MarkerProperty, visualization_msgs::msg::Marker &>
{
  static auto get(visualization_msgs::msg::Marker & marker)
  {
    return ReferenceMarker<MarkerProperty>(marker);
  }
};

template <template <typename Derived> typename MarkerProperty>
struct detail::MarkerComposition<MarkerProperty, std::reference_wrapper<visualization_msgs::msg::Marker>>
{
  static auto get(visualization_msgs::msg::Marker & marker)
  {
    return ReferenceMarker<MarkerProperty>(marker);
  }
};
}  // namespace flrv::marker
