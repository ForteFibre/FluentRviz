#pragma once

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/line_arrow.hpp"
#include "fluent_rviz/marker/pose_arrow.hpp"

namespace flrv::marker
{
template <typename Derived>
using Arrow = LineArrowProperty<Derived>;
}  // namespace flrv::marker
