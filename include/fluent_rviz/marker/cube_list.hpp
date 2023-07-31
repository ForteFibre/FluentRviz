#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct CubeListMarker : public MarkerBase<MarkerToken, CubeListMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, CubeListMarker<MarkerToken>>;

public:
  explicit CubeListMarker(std::string frame_id, MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  {
    this->marker().header.frame_id = std::move(frame_id);
    this->marker().action = visualization_msgs::msg::Marker::ADD;
    this->marker().type = visualization_msgs::msg::Marker::CUBE_LIST;
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::scale;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::points;
  using Base::colors;
};

template <typename MarkerToken = UseTemporal>
auto CubeList(std::string frame_id, MarkerToken &&token = { })
{ return CubeListMarker<MarkerToken>{ std::move(frame_id), std::forward<MarkerToken>(token) }; }
}  // namespace flrv::marker
