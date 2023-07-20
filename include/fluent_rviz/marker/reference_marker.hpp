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

protected:
  ReferenceMarker(visualization_msgs::msg::Marker &marker)
    : _marker(marker)
  { }

public:
  ReferenceMarker(const ReferenceMarker &) = delete;
  ReferenceMarker(ReferenceMarker &&) = delete;

  auto operator=(const ReferenceMarker &) = delete;
  auto operator=(ReferenceMarker &&) = delete;

protected:
  auto marker() noexcept -> visualization_msgs::msg::Marker &
  { return _marker; }

public:
  auto get() noexcept -> const visualization_msgs::msg::Marker &
  { return _marker; }

  operator const visualization_msgs::msg::Marker &() noexcept
  { return _marker; }
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
