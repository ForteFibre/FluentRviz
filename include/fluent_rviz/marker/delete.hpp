#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"

namespace flrv::marker
{
template <typename Derived>
struct Delete : public MarkerBase<Derived>
{
private:
  using Base = MarkerBase<Derived>;

public:
  Delete()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::DELETE);
  }

  using Base::ns;
  using Base::id;
};
}  // namespace flrv::marker
