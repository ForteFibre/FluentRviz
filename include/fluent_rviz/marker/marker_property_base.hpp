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
template <typename Derived>
struct MarkerPropertyBase
{
protected:
  [[nodiscard]]
  auto derived() noexcept
  -> Derived &
  {
    return static_cast<Derived &>(*this);
  }

  auto ns(std::string ns) noexcept
  -> Derived &
  {
    derived().get().ns = std::move(ns);
    return derived();
  }

  auto id(std::int32_t id) noexcept
  -> Derived &
  {
    derived().get().id = id;
    return derived();
  }

  auto type(std::int32_t type) noexcept
  -> Derived &
  {
    derived().get().type = type;
    return derived();
  }

  auto action(std::int32_t action) noexcept
  -> Derived &
  {
    derived().get().action = action;
    return derived();
  }

  auto pose(geometry_msgs::msg::Pose pose) noexcept
  -> Derived &
  {
    derived().get().pose = pose;
    return derived();
  }

  auto scale(double x, double y, double z) noexcept
  -> Derived &
  {
    derived().get().scale.x = x;
    derived().get().scale.y = y;
    derived().get().scale.z = z;
    return derived();
  }

  auto color(std_msgs::msg::ColorRGBA color) noexcept
  -> Derived &
  {
    derived().get().color = color;
    return derived();
  }

  auto lifetime(rclcpp::Duration lifetime) noexcept
  -> Derived &
  {
    derived().get().lifetime = lifetime;
    return derived();
  }

  auto frame_locked(bool frame_locked) noexcept
  -> Derived &
  {
    derived().get().frame_locked = frame_locked;
    return derived();
  }

  auto points(std::vector<geometry_msgs::msg::Point> points) noexcept
  -> Derived &
  {
    derived().get().points = std::move(points);
    return derived();
  }

  auto colors(std::vector<std_msgs::msg::ColorRGBA> colors) noexcept
  -> Derived &
  {
    derived().get().colors = std::move(colors);
    return derived();
  }

  auto texture_resource(std::string texture_resource) noexcept
  -> Derived &
  {
    derived().get().texture_resource = std::move(texture_resource);
    return derived();
  }

  auto texture(sensor_msgs::msg::CompressedImage texture) noexcept
  -> Derived &
  {
    derived().get().texture = std::move(texture);
    return derived();
  }

  auto uv_coordinates(std::vector<visualization_msgs::msg::UVCoordinate> uv_coordinates) noexcept
  -> Derived &
  {
    derived().get().uv_coordinates = std::move(uv_coordinates);
    return derived();
  }

  auto text(std::string text) noexcept
  -> Derived &
  {
    derived().get().text = std::move(text);
    return derived();
  }

  auto mesh_resource(std::string mesh_resource) noexcept
  -> Derived &
  {
    derived().get().mesh_resource = std::move(mesh_resource);
    return derived();
  }

  auto mesh_file(visualization_msgs::msg::MeshFile mesh_file) noexcept
  -> Derived &
  {
    derived().get().mesh_file = std::move(mesh_file);
    return derived();
  }

  auto mesh_use_embedded_materials(bool mesh_use_embedded_materials) noexcept
  -> Derived &
  {
    derived().get().mesh_use_embedded_materials = mesh_use_embedded_materials;
    return derived();
  }
};
}  // namespace flrv::marker
