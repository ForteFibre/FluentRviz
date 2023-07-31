#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct DeleteMarker : public MarkerBase<MarkerToken, DeleteMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, DeleteMarker<MarkerToken>>;

public:
  explicit DeleteMarker(MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  { this->marker().action = visualization_msgs::msg::Marker::DELETE; }

  using Base::ns;
  using Base::id;
};

template <typename MarkerToken = UseTemporal>
auto Delete(MarkerToken &&token = { })
{ return DeleteMarker<MarkerToken>{ std::forward<MarkerToken>(token) }; }
}  // namespace flrv::marker
