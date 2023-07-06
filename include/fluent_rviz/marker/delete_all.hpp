#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"

namespace flrv::marker
{
template <typename Derived>
struct DeleteAll : public MarkerBase<Derived>
{
private:
  using Base = MarkerBase<Derived>;

public:
  DeleteAll()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::DELETEALL);
  }

  using Base::ns;
};
}  // namespace flrv::marker
