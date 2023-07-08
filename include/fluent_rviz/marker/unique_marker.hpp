#pragma once

#include <algorithm>
#include <memory>
#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"

namespace flrv::marker
{
template <template <typename Derived> typename MarkerProperty>
struct UniqueMarker : public MarkerProperty<UniqueMarker<MarkerProperty>>
{
private:
  friend MarkerPropertyBase<UniqueMarker<MarkerProperty>>;

  std::unique_ptr<visualization_msgs::msg::Marker> _marker;

  auto get() noexcept -> visualization_msgs::msg::Marker &
  {
    return *_marker;
  }

public:
  UniqueMarker()
    : _marker(std::make_unique<visualization_msgs::msg::Marker>())
  { }

  UniqueMarker(std::unique_ptr<visualization_msgs::msg::Marker> marker)
    : _marker(std::move(marker))
  { }

  operator std::unique_ptr<visualization_msgs::msg::Marker>() && noexcept
  {
    return std::move(_marker);
  }
};

struct UseUnique { };

template <template <typename Derived> typename MarkerProperty>
struct detail::MarkerComposition<MarkerProperty, UseUnique>
{
  static auto get(UseUnique)
  {
    return UniqueMarker<MarkerProperty>();
  }
};

template <template <typename Derived> typename MarkerProperty>
struct detail::MarkerComposition<MarkerProperty, std::unique_ptr<visualization_msgs::msg::Marker>>
{
  static auto get(std::unique_ptr<visualization_msgs::msg::Marker> marker)
  {
    return UniqueMarker<MarkerProperty>(std::move(marker));
  }
};
}  // namespace flrv::marker
