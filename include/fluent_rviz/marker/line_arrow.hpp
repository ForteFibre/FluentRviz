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
struct LineArrowMarker : public MarkerBase<MarkerToken, LineArrowMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, LineArrowMarker<MarkerToken>>;

public:
  explicit LineArrowMarker(MarkerToken token, std::string frame_id, const rclcpp::Time &stamp) noexcept
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

  auto scale(double shaft_diameter, double head_diameter, double head_length) && noexcept -> LineArrowMarker &&
  { return Base::scale(shaft_diameter, head_diameter, head_length); }

  auto points(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end) && noexcept -> LineArrowMarker &&
  { return Base::points({ start, end }); }
};

template <typename MarkerToken>
[[nodiscard]]
auto LineArrow(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return LineArrowMarker<MarkerToken>{ std::forward<MarkerToken>(token), std::move(frame_id), stamp }; }

[[nodiscard]]
inline auto LineArrow(std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return LineArrow(UseInplace{ }, std::move(frame_id), stamp); }
}  // namespace flrv::marker
