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
struct DeleteMarker : public MarkerBase<MarkerToken, DeleteMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, DeleteMarker<MarkerToken>>;

public:
  explicit DeleteMarker(MarkerToken token, std::string frame_id, const rclcpp::Time &stamp) noexcept
    : Base(std::forward<MarkerToken>(token), std::move(frame_id), stamp)
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::DELETE);
  }

  using Base::ns;
  using Base::id;
};

template <typename MarkerToken>
[[nodiscard]]
auto Delete(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return DeleteMarker<MarkerToken>{ std::forward<MarkerToken>(token), std::move(frame_id), stamp }; }

[[nodiscard]]
inline auto Delete(std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return Delete(UseInplace{ }, std::move(frame_id), stamp); }
}  // namespace flrv::marker
