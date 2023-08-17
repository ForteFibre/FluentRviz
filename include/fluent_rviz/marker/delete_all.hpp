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
struct DeleteAllMarker : public MarkerBase<MarkerToken, DeleteAllMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, DeleteAllMarker<MarkerToken>>;

public:
  explicit DeleteAllMarker(MarkerToken token, std::string frame_id, const rclcpp::Time &stamp) noexcept
    : Base(std::forward<MarkerToken>(token), std::move(frame_id), stamp)
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::DELETEALL);
  }

  using Base::ns;
};

template <typename MarkerToken>
[[nodiscard]]
auto DeleteAll(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return DeleteAllMarker<MarkerToken>{ std::forward<MarkerToken>(token), std::move(frame_id), stamp }; }

[[nodiscard]]
inline auto DeleteAll(std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return DeleteAll(UseInplace{ }, std::move(frame_id), stamp); }
}  // namespace flrv::marker
