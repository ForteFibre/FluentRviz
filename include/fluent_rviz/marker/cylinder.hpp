#pragma once

#include <string>
#include <utility>

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/inplace_marker.hpp"
#include "fluent_rviz/marker/marker_base.hpp"

namespace flrv::marker
{
template <typename MarkerToken>
struct CylinderMarker : public MarkerBase<MarkerToken, CylinderMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, CylinderMarker<MarkerToken>>;

public:
  explicit CylinderMarker(MarkerToken token, std::string frame_id, const rclcpp::Time &stamp) noexcept
    : Base(std::forward<MarkerToken>(token), std::move(frame_id), stamp)
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::CYLINDER);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::scale;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
};

template <typename MarkerToken>
[[nodiscard]]
auto Cylinder(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp) noexcept
{ return CylinderMarker<MarkerToken>{ std::forward<MarkerToken>(token), std::move(frame_id), stamp }; }

[[nodiscard]]
inline auto Cylinder(std::string frame_id, const rclcpp::Time &stamp) noexcept
{ return Cylinder(UseInplace{ }, std::move(frame_id), stamp); }
}  // namespace flrv::marker
