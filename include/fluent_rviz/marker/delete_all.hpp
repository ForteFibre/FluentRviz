#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/marker_wrapper.hpp"

namespace flrv::marker
{
template <typename MarkerToken = MarkerWrapper>
struct DeleteAll : public MarkerBase<MarkerToken, DeleteAll<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, DeleteAll<MarkerToken>>;

public:
  DeleteAll()
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::DELETEALL);
  }

  using Base::ns;
};
}  // namespace flrv::marker
