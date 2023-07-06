#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/duration.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/mesh_file.hpp>
#include <visualization_msgs/msg/uv_coordinate.hpp>

namespace flrv::marker
{
template <typename MarkerToken, typename Derived>
struct MarkerBase
{
protected:
  MarkerToken _token;

  [[nodiscard]]
  auto derived() noexcept
  -> Derived &
  {
    return static_cast<Derived &>(*this);
  }

  auto ns(std::string ns) && noexcept
  -> Derived &&
  {
    _token.get().ns = std::move(ns);
    return std::move(derived());
  }

  auto id(std::int32_t id) && noexcept
  -> Derived &&
  {
    _token.get().id = id;
    return std::move(derived());
  }

  auto type(std::int32_t type) && noexcept
  -> Derived &&
  {
    _token.get().type = type;
    return std::move(derived());
  }

  auto action(std::int32_t action) && noexcept
  -> Derived &&
  {
    _token.get().action = action;
    return std::move(derived());
  }

  auto pose(geometry_msgs::msg::Pose pose) && noexcept
  -> Derived &&
  {
    _token.get().pose = pose;
    return std::move(derived());
  }

  auto scale(double x, double y, double z) && noexcept
  -> Derived &&
  {
    auto scale = geometry_msgs::msg::Vector3();
    scale.x = x, scale.y = y, scale.z = z;
    _token.get().scale = scale;
    return std::move(derived());
  }

  auto color(std_msgs::msg::ColorRGBA color) && noexcept
  -> Derived &&
  {
    _token.get().color = color;
    return std::move(derived());
  }

  auto lifetime(rclcpp::Duration lifetime) && noexcept
  -> Derived &&
  {
    _token.get().lifetime = lifetime;
    return std::move(derived());
  }

  auto frame_locked(bool frame_locked) && noexcept
  -> Derived &&
  {
    _token.get().frame_locked = frame_locked;
    return std::move(derived());
  }

  auto points(std::vector<geometry_msgs::msg::Point> points) && noexcept
  -> Derived &&
  {
    _token.get().points = std::move(points);
    return std::move(derived());
  }

  auto colors(std::vector<std_msgs::msg::ColorRGBA> colors) && noexcept
  -> Derived &&
  {
    _token.get().colors = std::move(colors);
    return std::move(derived());
  }

  auto texture_resource(std::string texture_resource) && noexcept
  -> Derived &&
  {
    _token.get().texture_resource = std::move(texture_resource);
    return std::move(derived());
  }

  auto texture(const sensor_msgs::msg::CompressedImage & texture) && noexcept
  -> Derived &&
  {
    _token.get().texture = texture;
    return std::move(derived());
  }

  auto uv_coordinates(std::vector<visualization_msgs::msg::UVCoordinate> uv_coordinates) && noexcept
  -> Derived &&
  {
    _token.get().uv_coordinates = std::move(uv_coordinates);
    return std::move(derived());
  }

  auto text(std::string text) && noexcept
  -> Derived &&
  {
    _token.get().text = std::move(text);
    return std::move(derived());
  }

  auto mesh_resource(std::string mesh_resource) && noexcept
  -> Derived &&
  {
    _token.get().mesh_resource = std::move(mesh_resource);
    return std::move(derived());
  }

  auto mesh_file(const visualization_msgs::msg::MeshFile & mesh_file) && noexcept
  -> Derived &&
  {
    _token.get().mesh_file = mesh_file;
    return std::move(derived());
  }

  auto mesh_use_embedded_materials(bool mesh_use_embedded_materials) && noexcept
  -> Derived &&
  {
    _token.get().mesh_use_embedded_materials = mesh_use_embedded_materials;
    return std::move(derived());
  }
};
}  // namespace flrv::marker
