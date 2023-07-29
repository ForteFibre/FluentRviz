#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <string>
#include <utility>
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
#include "fluent_rviz/color.hpp"
#include "fluent_rviz/point.hpp"
#include "fluent_rviz/pose.hpp"
#include "fluent_rviz/traits.hpp"

namespace flrv::marker
{
template <typename MarkerToken, typename Derived>
struct MarkerBase : public PlainMarkerBase<MarkerToken>
{
private:
  using Base = PlainMarkerBase<MarkerToken>;

protected:
  using Base::Base;

  auto ns(std::string ns) && noexcept -> Derived &&
  {
    this->marker().ns = std::move(ns);
    return static_cast<Derived &&>(*this);
  }

  auto id(const std::int32_t id) && noexcept -> Derived &&
  {
    this->marker().id = id;
    return static_cast<Derived &&>(*this);
  }

  auto type(const std::int32_t type) && noexcept -> Derived &&
  {
    this->marker().type = type;
    return static_cast<Derived &&>(*this);
  }

  auto action(const std::int32_t action) && noexcept -> Derived &&
  {
    this->marker().action = action;
    return static_cast<Derived &&>(*this);
  }

  template <
    typename PoseLike = pose::Pose,
    traits::Require<
      traits::ConversionDefined<geometry_msgs::msg::Pose, PoseLike>> = nullptr>
  auto pose(const PoseLike &pose) && noexcept -> Derived &&
  {
    this->marker().pose = traits::convert<geometry_msgs::msg::Pose>(pose);
    return static_cast<Derived &&>(*this);
  }

  auto scale(const double x, const double y, const double z) && noexcept -> Derived &&
  {
    this->marker().scale.x = x;
    this->marker().scale.y = y;
    this->marker().scale.z = z;
    return static_cast<Derived &&>(*this);
  }

  template <
    typename ColorLike = color::Color,
    traits::Require<
      traits::ConversionDefined<std_msgs::msg::ColorRGBA, ColorLike>> = nullptr>
  auto color(const ColorLike &color) && noexcept -> Derived &&
  {
    this->marker().color = traits::convert<std_msgs::msg::ColorRGBA>(color);
    return static_cast<Derived &&>(*this);
  }

  auto lifetime(rclcpp::Duration lifetime) && noexcept -> Derived &&
  {
    this->marker().lifetime = lifetime;
    return static_cast<Derived &&>(*this);
  }

  auto frame_locked(const bool frame_locked) && noexcept -> Derived &&
  {
    this->marker().frame_locked = frame_locked;
    return static_cast<Derived &&>(*this);
  }

  auto points(std::vector<geometry_msgs::msg::Point> points) && noexcept -> Derived &&
  {
    this->marker().points = std::move(points);
    return static_cast<Derived &&>(*this);
  }

  template <
    typename PointArrayLike = std::initializer_list<point::Point>,
    traits::Require<
      traits::ConversionDefined<std::vector<geometry_msgs::msg::Point>, PointArrayLike>> = nullptr>
  auto points(const PointArrayLike &points) && noexcept -> Derived &&
  { return std::move(*this).points(traits::convert<std::vector<geometry_msgs::msg::Point>>(points)); }

  auto colors(std::vector<std_msgs::msg::ColorRGBA> colors) && noexcept -> Derived &&
  {
    this->marker().colors = std::move(colors);
    return static_cast<Derived &&>(*this);
  }

  template <
    typename ColorArrayLike = std::initializer_list<color::Color>,
    traits::Require<
      traits::ConversionDefined<std::vector<std_msgs::msg::ColorRGBA>, ColorArrayLike>> = nullptr>
  auto colors(const ColorArrayLike &colors) && noexcept -> Derived &&
  { return std::move(*this).colors(traits::convert<std::vector<std_msgs::msg::ColorRGBA>>(colors)); }

  auto texture_resource(std::string texture_resource) && noexcept -> Derived &&
  {
    this->marker().texture_resource = std::move(texture_resource);
    return static_cast<Derived &&>(*this);
  }

  auto texture(sensor_msgs::msg::CompressedImage texture) && noexcept -> Derived &&
  {
    this->marker().texture = std::move(texture);
    return static_cast<Derived &&>(*this);
  }

  auto uv_coordinates(std::vector<visualization_msgs::msg::UVCoordinate> uv_coordinates) && noexcept -> Derived &&
  {
    this->marker().uv_coordinates = std::move(uv_coordinates);
    return static_cast<Derived &&>(*this);
  }

  template <
    typename UVCoordinateArrayLike = std::initializer_list<visualization_msgs::msg::UVCoordinate>,
    traits::Require<
      traits::ConversionDefined<std::vector<visualization_msgs::msg::UVCoordinate>, UVCoordinateArrayLike>> = nullptr>
  auto uv_coordinates(const UVCoordinateArrayLike &uv_coordinates) && noexcept -> Derived &&
  { return std::move(*this).uv_coordinates(traits::convert<std::vector<visualization_msgs::msg::UVCoordinate>>(uv_coordinates)); }

  auto text(std::string text) && noexcept -> Derived &&
  {
    this->marker().text = std::move(text);
    return static_cast<Derived &&>(*this);
  }

  auto mesh_resource(std::string mesh_resource) && noexcept -> Derived &&
  {
    this->marker().mesh_resource = std::move(mesh_resource);
    return static_cast<Derived &&>(*this);
  }

  auto mesh_file(visualization_msgs::msg::MeshFile mesh_file) && noexcept -> Derived &&
  {
    this->marker().mesh_file = std::move(mesh_file);
    return static_cast<Derived &&>(*this);
  }

  auto mesh_use_embedded_materials(const bool mesh_use_embedded_materials) && noexcept -> Derived &&
  {
    this->marker().mesh_use_embedded_materials = mesh_use_embedded_materials;
    return static_cast<Derived &&>(*this);
  }
};
}  // namespace flrv::marker
