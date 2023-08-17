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
struct LineListMarker : public MarkerBase<MarkerToken, LineListMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, LineListMarker<MarkerToken>>;

public:
  explicit LineListMarker(MarkerToken token, std::string frame_id, const rclcpp::Time &stamp) noexcept
    : Base(std::forward<MarkerToken>(token), std::move(frame_id), stamp)
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::LINE_LIST);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::points;
  using Base::colors;

  auto scale(double width) && noexcept -> LineListMarker &&
  { return Base::scale(width, 0, 0); }
};

template <typename MarkerToken>
[[nodiscard]]
auto LineList(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return LineListMarker<MarkerToken>{ std::forward<MarkerToken>(token), std::move(frame_id), stamp }; }

[[nodiscard]]
inline auto LineList(std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return LineList(UseInplace{ }, std::move(frame_id), stamp); }
}  // namespace flrv::marker
