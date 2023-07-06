#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/marker_wrapper.hpp"

namespace flrv::marker
{
template <typename MarkerToken = MarkerWrapper>
struct CubeList : public MarkerBase<MarkerToken, CubeList<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, CubeList<MarkerToken>>;

public:
  CubeList()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::CUBE_LIST);
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
}  // namespace flrv::marker
