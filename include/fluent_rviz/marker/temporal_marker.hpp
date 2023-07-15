#pragma once

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
  {
    return _marker;
  }

public:
  auto get() noexcept -> const visualization_msgs::msg::Marker &
  {
    return _marker;
  }

  operator const visualization_msgs::msg::Marker &() const noexcept
  {
    return _marker;
  }
};

struct UseTemporal { };

template <>
struct PlainMarkerBase<UseTemporal> : public TemporalMarker
{
  PlainMarkerBase(UseTemporal) { }
};
}  // namespace flrv::marker
