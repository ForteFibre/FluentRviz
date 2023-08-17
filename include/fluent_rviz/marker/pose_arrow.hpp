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
struct PoseArrowMarker : public MarkerBase<MarkerToken, PoseArrowMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, PoseArrowMarker<MarkerToken>>;

public:
  explicit PoseArrowMarker(MarkerToken token, std::string frame_id, const rclcpp::Time &stamp) noexcept
    : Base(std::forward<MarkerToken>(token), std::move(frame_id), stamp)
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::ARROW);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;

  auto scale(double length, double width, double height) noexcept -> PoseArrowMarker &
  { return Base::scale(length, width, height); }
};

template <typename MarkerToken>
[[nodiscard]]
auto PoseArrow(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return PoseArrowMarker<MarkerToken>{ std::forward<MarkerToken>(token), std::move(frame_id), stamp }; }

[[nodiscard]]
inline auto PoseArrow(std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return PoseArrow(UseInplace{ }, std::move(frame_id), stamp); }
}  // namespace flrv::marker
