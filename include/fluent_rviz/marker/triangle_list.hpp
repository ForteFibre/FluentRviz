#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"

namespace flrv::marker
{
template <typename Derived>
struct TriangleList : public MarkerBase<Derived>
{
private:
  using Base = MarkerBase<Derived>;

public:
  TriangleList()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::TRIANGLE_LIST);
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
