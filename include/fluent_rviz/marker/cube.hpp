#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct CubeMarker : public MarkerBase<MarkerToken, CubeMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, CubeMarker<MarkerToken>>;

public:
  explicit CubeMarker(std::string frame_id, MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  {
    this->marker().header.frame_id = std::move(frame_id);
    this->marker().action = visualization_msgs::msg::Marker::ADD;
    this->marker().type = visualization_msgs::msg::Marker::CUBE;
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::scale;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
};

template <typename MarkerToken = UseTemporal>
auto Cube(std::string frame_id, MarkerToken &&token = { })
{ return CubeMarker<MarkerToken>{ std::move(frame_id), std::forward<MarkerToken>(token) }; }
}  // namespace flrv::marker
