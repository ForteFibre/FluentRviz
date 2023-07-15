#pragma once

#include "fluent_rviz/marker/plain_marker_base.hpp"
#include <utility>

#include <rclcpp/loaned_message.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace flrv::marker
{
struct LoanedMessage
{
private:
  rclcpp::LoanedMessage<visualization_msgs::msg::Marker> _marker;

protected:
  LoanedMessage(rclcpp::LoanedMessage<visualization_msgs::msg::Marker> marker)
    : _marker(std::move(marker))
  { }

  auto get() -> visualization_msgs::msg::Marker &
  {
    return _marker.get();
  }

public:
  auto build() && noexcept -> rclcpp::LoanedMessage<visualization_msgs::msg::Marker>
  {
    return std::move(_marker);
  }

  operator rclcpp::LoanedMessage<visualization_msgs::msg::Marker>() && noexcept
  {
    return std::move(_marker);
  }
};

template <>
struct PlainMarkerBase<rclcpp::LoanedMessage<visualization_msgs::msg::Marker>> : public LoanedMessage
{
  using LoanedMessage::LoanedMessage;
};
}  // namespace flrv::marker
