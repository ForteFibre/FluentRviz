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
struct SphereMarker : public MarkerBase<MarkerToken, SphereMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, SphereMarker<MarkerToken>>;

public:
  explicit SphereMarker(MarkerToken token, std::string frame_id, const rclcpp::Time &stamp) noexcept
    : Base(std::forward<MarkerToken>(token), std::move(frame_id), stamp)
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::SPHERE);
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
auto Sphere(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return SphereMarker<MarkerToken>{ std::forward<MarkerToken>(token), std::move(frame_id), stamp }; }

[[nodiscard]]
inline auto Sphere(std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return Sphere(UseInplace{ }, std::move(frame_id), stamp); }
}  // namespace flrv::marker
