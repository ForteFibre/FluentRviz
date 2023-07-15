#pragma once

#include <algorithm>
#include <memory>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/plain_marker_base.hpp"

namespace flrv::marker
{
struct UniqueMarker
{
private:
  std::unique_ptr<visualization_msgs::msg::Marker> _marker;

protected:
  UniqueMarker()
    : _marker(std::make_unique<visualization_msgs::msg::Marker>())
  { }

  UniqueMarker(std::unique_ptr<visualization_msgs::msg::Marker> marker)
    : _marker(std::move(marker))
  { }

  auto marker() noexcept -> visualization_msgs::msg::Marker &
  {
    return *_marker;
  }

public:
  auto get() && noexcept -> std::unique_ptr<visualization_msgs::msg::Marker>
  {
    return std::move(_marker);
  }

  operator std::unique_ptr<visualization_msgs::msg::Marker>() && noexcept
  {
    return std::move(_marker);
  }
};

struct UseUnique { };

template <>
struct PlainMarkerBase<UseUnique> : public UniqueMarker
{
  PlainMarkerBase(UseUnique)
  { }
};

template <>
struct PlainMarkerBase<std::unique_ptr<visualization_msgs::msg::Marker>> : public UniqueMarker
{
  PlainMarkerBase(std::unique_ptr<visualization_msgs::msg::Marker> marker)
    : UniqueMarker(std::move(marker))
  { }
};
}  // namespace flrv::marker
