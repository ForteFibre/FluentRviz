#pragma once

#include <rclcpp/time.hpp>
#include <utility>

#include <visualization_msgs/msg/marker.hpp>

#include "fluent_rviz/marker/inplace_marker.hpp"
#include "fluent_rviz/marker/marker_base.hpp"

namespace flrv::marker
{
template <typename MarkerToken>
struct MeshResourceMarker : public MarkerBase<MarkerToken, MeshResourceMarker<MarkerToken>>
{
private:
  using Base = MarkerBase<MarkerToken, MeshResourceMarker<MarkerToken>>;

public:
  explicit MeshResourceMarker(MarkerToken token, std::string frame_id, const rclcpp::Time &stamp) noexcept
    : Base(std::forward<MarkerToken>(token), std::move(frame_id), stamp)
  {
    std::move(*this)
      .action(visualization_msgs::msg::Marker::ADD)
      .type(visualization_msgs::msg::Marker::MESH_RESOURCE);
  }

  using Base::ns;
  using Base::id;
  using Base::pose;
  using Base::scale;
  using Base::color;
  using Base::lifetime;
  using Base::frame_locked;
  using Base::mesh_resource;
  using Base::mesh_file;
  using Base::mesh_use_embedded_materials;
};

template <typename MarkerToken>
[[nodiscard]]
auto MeshResource(MarkerToken &&token, std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return MeshResourceMarker<MarkerToken>{ std::forward<MarkerToken>(token), std::move(frame_id), stamp }; }

[[nodiscard]]
inline auto MeshResource(std::string frame_id, const rclcpp::Time &stamp = rclcpp::Time{ }) noexcept
{ return MeshResource(UseInplace{ }, std::move(frame_id), stamp); }
}  // namespace flrv::marker
