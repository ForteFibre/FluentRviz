#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/plain_marker_base.hpp"

namespace flrv::marker
{
struct TemporalMarker
{
private:
  visualization_msgs::msg::Marker _marker;

protected:
  auto marker() noexcept -> visualization_msgs::msg::Marker &
  { return _marker; }

public:
  auto get() const & noexcept -> visualization_msgs::msg::Marker
  { return _marker; }

  auto get() && noexcept -> visualization_msgs::msg::Marker &&
  { return std::move(_marker); }

  operator visualization_msgs::msg::Marker() const & noexcept
  { return _marker; }

  operator visualization_msgs::msg::Marker &&() && noexcept
  { return std::move(_marker); }
};

struct UseTemporal { };

template <>
struct PlainMarkerBase<UseTemporal> : public TemporalMarker
{
  PlainMarkerBase(UseTemporal) { }
};
}  // namespace flrv::marker
