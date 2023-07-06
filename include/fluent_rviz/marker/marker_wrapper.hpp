#pragma once

#include <visualization_msgs/msg/marker.hpp>

namespace flrv::marker
{
struct MarkerWrapper
{
  visualization_msgs::msg::Marker marker;

  auto get() -> visualization_msgs::msg::Marker &
  {
    return marker;
  }
};
}
