#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/mesh_file.hpp>
#include <visualization_msgs/msg/uv_coordinate.hpp>

#include "fluent_rviz/marker/marker_selector.hpp"
#include "fluent_rviz/color.hpp"
#include "fluent_rviz/point.hpp"
#include "fluent_rviz/pose.hpp"
#include "fluent_rviz/traits/convert.hpp"
#include "fluent_rviz/traits/like_message.hpp"

namespace flrv::marker
{
template <typename MarkerToken, typename Derived>
class MarkerBase : public MarkerSelector<MarkerToken>
{
  using Base = MarkerSelector<MarkerToken>;

  auto self() -> Derived &
  { return static_cast<Derived &>(*this); }

  auto marker() -> visualization_msgs::msg::Marker &
  { return get_marker(*this); }

protected:
  MarkerBase(MarkerToken token, std::string frame_id, const rclcpp::Time &stamp)
    : Base(std::forward<MarkerToken>(token))
  {
    marker().header.frame_id = std::move(frame_id);
    marker().header.stamp = stamp;
  }

  auto ns(std::string ns) && noexcept -> Derived &&
  {
    marker().ns = std::move(ns);
    return std::move(self());
  }

  auto id(const std::int32_t id) && noexcept -> Derived &&
  {
    marker().id = id;
    return std::move(self());
  }

  auto type(const std::int32_t type) && noexcept -> Derived &&
  {
    marker().type = type;
    return std::move(self());
  }

  auto action(const std::int32_t action) && noexcept -> Derived &&
  {
    marker().action = action;
    return std::move(self());
  }

  template <
    typename PoseLike = pose::Pose,
    std::enable_if_t<traits::like_pose<PoseLike>, std::nullptr_t> = nullptr>
  auto pose(const PoseLike &pose) && noexcept -> Derived &&
  {
    marker().pose = traits::convert<geometry_msgs::msg::Pose>(pose);
    return std::move(self());
  }

  auto scale(const double x, const double y, const double z) && noexcept -> Derived &&
  {
    marker().scale.x = x;
    marker().scale.y = y;
    marker().scale.z = z;
    return std::move(self());
  }

  template <
    typename ColorLike = color::Color,
    std::enable_if_t<traits::like_color<ColorLike>, std::nullptr_t> = nullptr>
  auto color(const ColorLike &color) && noexcept -> Derived &&
  {
    marker().color = traits::convert<std_msgs::msg::ColorRGBA>(color);
    return std::move(self());
  }

  auto lifetime(rclcpp::Duration lifetime) && noexcept -> Derived &&
  {
    marker().lifetime = lifetime;
    return std::move(self());
  }

  auto frame_locked(const bool frame_locked) && noexcept -> Derived &&
  {
    marker().frame_locked = frame_locked;
    return std::move(self());
  }

  template <
    typename PointArrayLike = std::initializer_list<point::Point>,
    std::enable_if_t<traits::like_points<PointArrayLike>, std::nullptr_t> = nullptr>
  auto points(const PointArrayLike &points) && noexcept -> Derived &&
  {
    marker().points = traits::convert<std::vector<geometry_msgs::msg::Point>>(points);
    return std::move(self());
  }

  template <
    typename ColorArrayLike = std::initializer_list<color::Color>,
    std::enable_if_t<traits::like_colors<ColorArrayLike>, std::nullptr_t> = nullptr>
  auto colors(const ColorArrayLike &colors) && noexcept -> Derived &&
  {
    marker().colors = traits::convert<std::vector<std_msgs::msg::ColorRGBA>>(colors);
    return std::move(self());
  }

  auto texture_resource(std::string texture_resource) && noexcept -> Derived &&
  {
    marker().texture_resource = std::move(texture_resource);
    return std::move(self());
  }

  auto texture(sensor_msgs::msg::CompressedImage texture) && noexcept -> Derived &&
  {
    marker().texture = std::move(texture);
    return std::move(self());
  }

  template <
    typename UVCoordinateArrayLike = std::initializer_list<visualization_msgs::msg::UVCoordinate>,
    std::enable_if_t<traits::like_uv_coordinates<UVCoordinateArrayLike>, std::nullptr_t> = nullptr>
  auto uv_coordinates(const UVCoordinateArrayLike &uv_coordinates) && noexcept -> Derived &&
  {
    marker().uv_coordinates = traits::convert<std::vector<visualization_msgs::msg::UVCoordinate>>(uv_coordinates);
    return std::move(*this);
  }

  auto text(std::string text) && noexcept -> Derived &&
  {
    marker().text = std::move(text);
    return std::move(self());
  }

  auto mesh_resource(std::string mesh_resource) && noexcept -> Derived &&
  {
    marker().mesh_resource = std::move(mesh_resource);
    return std::move(self());
  }

  auto mesh_file(visualization_msgs::msg::MeshFile mesh_file) && noexcept -> Derived &&
  {
    marker().mesh_file = std::move(mesh_file);
    return std::move(self());
  }

  auto mesh_use_embedded_materials(const bool mesh_use_embedded_materials) && noexcept -> Derived &&
  {
    marker().mesh_use_embedded_materials = mesh_use_embedded_materials;
    return std::move(self());
  }
};
}  // namespace flrv::marker
