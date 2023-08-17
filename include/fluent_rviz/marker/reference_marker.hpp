#pragma once

#include <functional>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/plain_marker_base.hpp"

namespace flrv::marker
{
struct ReferenceMarker
{
private:
  visualization_msgs::msg::Marker &_marker;

public:
  ReferenceMarker(visualization_msgs::msg::Marker &marker)
    : _marker(marker)
  { }

  ReferenceMarker(const ReferenceMarker &) = delete;
  ReferenceMarker(ReferenceMarker &&) = delete;

  auto operator=(const ReferenceMarker &) = delete;
  auto operator=(ReferenceMarker &&) = delete;

  [[nodiscard]]
  auto get() noexcept -> const visualization_msgs::msg::Marker &
  { return _marker; }

  [[nodiscard]]
  friend auto get_marker(ReferenceMarker &marker) noexcept -> visualization_msgs::msg::Marker &
  { return marker._marker; }
};

template <>
struct PlainMarkerBase<visualization_msgs::msg::Marker &> : public ReferenceMarker
{
  using ReferenceMarker::ReferenceMarker;
};

template <>
struct PlainMarkerBase<std::reference_wrapper<visualization_msgs::msg::Marker>> : public ReferenceMarker
{
  using ReferenceMarker::ReferenceMarker;
};
}  // namespace flrv::marker
