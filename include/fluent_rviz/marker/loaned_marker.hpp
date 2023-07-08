#pragma once

#include <rclcpp/loaned_message.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/marker_composition.hpp"
#include "fluent_rviz/marker/marker_property_base.hpp"

namespace flrv::marker
{
template <template <typename Derived> typename MarkerProperty>
struct LoanedMarker : public MarkerProperty<LoanedMarker<MarkerProperty>>
{
private:
  friend MarkerPropertyBase<LoanedMarker<MarkerProperty>>;

  rclcpp::LoanedMessage<visualization_msgs::msg::Marker> _marker;

  auto get() noexcept -> visualization_msgs::msg::Marker &
  {
    return _marker.get();
  }

public:
  LoanedMarker(rclcpp::LoanedMessage<visualization_msgs::msg::Marker> marker)
    : _marker(std::move(marker))
  { }

  operator rclcpp::LoanedMessage<visualization_msgs::msg::Marker>() && noexcept
  {
    return std::move(_marker);
  }
};

template <template <typename Derived> typename MarkerProperty>
struct detail::MarkerComposition<MarkerProperty, rclcpp::LoanedMessage<visualization_msgs::msg::Marker>>
{
  static auto get(rclcpp::LoanedMessage<visualization_msgs::msg::Marker> marker)
  {
    return LoanedMarker<MarkerProperty>(std::move(marker));
  }
};
}  // namespace flrv::marker
