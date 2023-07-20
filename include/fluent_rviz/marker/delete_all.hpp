#pragma once

#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/temporal_marker.hpp"

namespace flrv::marker
{
template <typename MarkerToken = UseTemporal>
struct DeleteAllMarker : public MarkerBase<MarkerToken, DeleteAllMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, DeleteAllMarker<MarkerToken>>;

public:
  explicit DeleteAllMarker(MarkerToken token = { })
    : Base(std::forward<MarkerToken>(token))
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::DELETEALL);
  }

  using Base::ns;
};

template <typename MarkerToken = UseTemporal>
auto DeleteAll(MarkerToken &&token = { }) -> DeleteAllMarker<MarkerToken>
{
  return { std::forward<MarkerToken>(token) };
}
}  // namespace flrv::marker
