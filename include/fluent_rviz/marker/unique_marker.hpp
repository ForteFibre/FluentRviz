#pragma once

#include <algorithm>
#include <memory>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_selector.hpp"

namespace flrv::marker
{
struct UniqueMarker
{
private:
  std::unique_ptr<visualization_msgs::msg::Marker> _marker;

public:
  UniqueMarker(std::unique_ptr<visualization_msgs::msg::Marker> marker)
    : _marker(std::move(marker))
  { }

  [[nodiscard]]
  auto get() && noexcept -> std::unique_ptr<visualization_msgs::msg::Marker>
  { return std::move(_marker); }

  [[nodiscard]]
  friend auto get_marker(UniqueMarker &marker) noexcept -> visualization_msgs::msg::Marker &
  { return *marker._marker; }
};

struct UseUnique { };

template <>
struct MarkerSelector<UseUnique> : public UniqueMarker
{
  MarkerSelector(UseUnique)
    : UniqueMarker(std::make_unique<visualization_msgs::msg::Marker>())
  { }
};

template <>
struct MarkerSelector<std::unique_ptr<visualization_msgs::msg::Marker>> : public UniqueMarker
{
  MarkerSelector(std::unique_ptr<visualization_msgs::msg::Marker> marker)
    : UniqueMarker(std::move(marker))
  { }
};
}  // namespace flrv::marker
