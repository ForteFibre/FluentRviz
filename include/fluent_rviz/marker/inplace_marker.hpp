#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/plain_marker_base.hpp"

namespace flrv::marker
{
struct InplaceMarker
{
private:
  visualization_msgs::msg::Marker _marker;

public:
  [[nodiscard]]
  auto get() const & noexcept -> visualization_msgs::msg::Marker
  { return _marker; }

  [[nodiscard]]
  auto get() && noexcept -> visualization_msgs::msg::Marker &&
  { return std::move(_marker); }

  [[nodiscard]]
  friend auto get_marker(InplaceMarker &marker) noexcept -> visualization_msgs::msg::Marker &
  { return marker._marker; }
};

struct UseInplace { };

template <>
struct PlainMarkerBase<UseInplace> : public InplaceMarker
{
  PlainMarkerBase(UseInplace) { }
};
}  // namespace flrv::marker
