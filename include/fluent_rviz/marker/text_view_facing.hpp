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
struct TextViewFacingMarker : public MarkerBase<MarkerToken, TextViewFacingMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, TextViewFacingMarker<MarkerToken>>;

public:
  explicit TextViewFacingMarker(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp) noexcept
    : Base(std::forward<MarkerToken>(token), std::move(frame_id), stamp)
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::text;

  auto scale(double height) && noexcept -> TextViewFacingMarker &&
  { return Base::scale(0, 0, height); }
};

template <typename MarkerToken>
[[nodiscard]]
auto TextViewFacing(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return TextViewFacingMarker<MarkerToken>{ std::forward<MarkerToken>(token), std::move(frame_id), stamp }; }

[[nodiscard]]
inline auto TextViewFacing(std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return TextViewFacing(UseInplace{ }, std::move(frame_id), stamp); }
}  // namespace flrv::marker
