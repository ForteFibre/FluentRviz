#pragma once

#include "fluent_rviz/marker/marker_selector.hpp"
#include <utility>

#include <rclcpp/loaned_message.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace flrv::marker
{
struct LoanedMarker
{
private:
  rclcpp::LoanedMessage<visualization_msgs::msg::Marker> _marker;

public:
  LoanedMarker(rclcpp::LoanedMessage<visualization_msgs::msg::Marker> marker)
    : _marker(std::move(marker))
  { }

  [[nodiscard]]
  auto get() && noexcept -> rclcpp::LoanedMessage<visualization_msgs::msg::Marker>
  { return std::move(_marker); }

  [[nodiscard]]
  friend auto get_marker(LoanedMarker &marker) noexcept -> visualization_msgs::msg::Marker &
  { return marker._marker.get(); }
};

template <>
struct MarkerSelector<rclcpp::LoanedMessage<visualization_msgs::msg::Marker>> : public LoanedMarker
{
  using LoanedMarker::LoanedMarker;
};
}  // namespace flrv::marker
