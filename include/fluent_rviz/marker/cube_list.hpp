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
struct CubeListMarker : public MarkerBase<MarkerToken, CubeListMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, CubeListMarker<MarkerToken>>;

public:
  explicit CubeListMarker(MarkerToken token, std::string frame_id, const rclcpp::Time &stamp) noexcept
    : Base(std::forward<MarkerToken>(token), std::move(frame_id), stamp)
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::CUBE_LIST);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::scale;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::points;
  using Base::colors;
};

template <typename MarkerToken>
[[nodiscard]]
auto CubeList(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return CubeListMarker<MarkerToken>{ std::forward<MarkerToken>(token), std::move(frame_id), stamp }; }

[[nodiscard]]
inline auto CubeList(std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return CubeList(UseInplace{ }, std::move(frame_id), stamp); }
}  // namespace flrv::marker
