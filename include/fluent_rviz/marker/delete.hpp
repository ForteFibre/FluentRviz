#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_base.hpp"
#include "fluent_rviz/marker/marker_wrapper.hpp"

namespace flrv::marker
{
template <typename MarkerToken = MarkerWrapper>
struct Delete : public MarkerBase<MarkerToken, Delete<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, Delete<MarkerToken>>;

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
