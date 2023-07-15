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

#include "fluent_rviz/marker/plain_marker_base.hpp"

namespace flrv::marker
{
template <typename MarkerToken, typename Derived>
struct MarkerBase : public PlainMarkerBase<MarkerToken>
{
private:
  using Base = PlainMarkerBase<MarkerToken>;

protected:
  using Base::Base;

  auto ns(std::string ns) && noexcept
  -> Derived &&
  {
    this->get().ns = std::move(ns);
    return static_cast<Derived &&>(*this);
  }

  auto id(std::int32_t id) && noexcept
  -> Derived &&
  {
    this->get().id = id;
    return static_cast<Derived &&>(*this);
  }

  auto type(std::int32_t type) && noexcept
  -> Derived &&
  {
    this->get().type = type;
    return static_cast<Derived &&>(*this);
  }

  auto action(std::int32_t action) && noexcept
  -> Derived &&
  {
    this->get().action = action;
    return static_cast<Derived &&>(*this);
  }

  auto pose(geometry_msgs::msg::Pose pose) && noexcept
  -> Derived &&
  {
    this->get().pose = pose;
    return static_cast<Derived &&>(*this);
  }

  auto scale(double x, double y, double z) && noexcept
  -> Derived &&
  {
    this->get().scale.x = x;
    this->get().scale.y = y;
    this->get().scale.z = z;
    return static_cast<Derived &&>(*this);
  }

  auto color(std_msgs::msg::ColorRGBA color) && noexcept
  -> Derived &&
  {
    this->get().color = color;
    return static_cast<Derived &&>(*this);
  }

  auto lifetime(rclcpp::Duration lifetime) && noexcept
  -> Derived &&
  {
    this->get().lifetime = lifetime;
    return static_cast<Derived &&>(*this);
  }

  auto frame_locked(bool frame_locked) && noexcept
  -> Derived &&
  {
    this->get().frame_locked = frame_locked;
    return static_cast<Derived &&>(*this);
  }

  auto points(std::vector<geometry_msgs::msg::Point> points) && noexcept
  -> Derived &&
  {
    this->get().points = std::move(points);
    return static_cast<Derived &&>(*this);
  }

  auto colors(std::vector<std_msgs::msg::ColorRGBA> colors) && noexcept
  -> Derived &&
  {
    this->get().colors = std::move(colors);
    return static_cast<Derived &&>(*this);
  }

  auto texture_resource(std::string texture_resource) && noexcept
  -> Derived &&
  {
    this->get().texture_resource = std::move(texture_resource);
    return static_cast<Derived &&>(*this);
  }

  auto texture(sensor_msgs::msg::CompressedImage texture) && noexcept
  -> Derived &&
  {
    this->get().texture = std::move(texture);
    return static_cast<Derived &&>(*this);
  }

  auto uv_coordinates(std::vector<visualization_msgs::msg::UVCoordinate> uv_coordinates) && noexcept
  -> Derived &&
  {
    this->get().uv_coordinates = std::move(uv_coordinates);
    return static_cast<Derived &&>(*this);
  }

  auto text(std::string text) && noexcept
  -> Derived &&
  {
    this->get().text = std::move(text);
    return static_cast<Derived &&>(*this);
  }

  auto mesh_resource(std::string mesh_resource) && noexcept
  -> Derived &&
  {
    this->get().mesh_resource = std::move(mesh_resource);
    return static_cast<Derived &&>(*this);
  }

  auto mesh_file(visualization_msgs::msg::MeshFile mesh_file) && noexcept
  -> Derived &&
  {
    this->get().mesh_file = std::move(mesh_file);
    return static_cast<Derived &&>(*this);
  }

  auto mesh_use_embedded_materials(bool mesh_use_embedded_materials) && noexcept
  -> Derived &&
  {
    this->get().mesh_use_embedded_materials = mesh_use_embedded_materials;
    return static_cast<Derived &&>(*this);
  }
};
}  // namespace flrv::marker
